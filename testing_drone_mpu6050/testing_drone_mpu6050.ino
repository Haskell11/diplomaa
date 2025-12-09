/*  
    - MPU6050 DMP (рыскание/тангаж/крен)
    - UDP-стриминг (START / STOP)
    - Управление 4 ESC через Servo.writeMicroseconds
    - PID по углам (всегда ВКЛ)
    - UDP команда: SET <рыскание> <тангаж> <крен> <газ>
      пример: "SET 0 10 0 1300"  -> желаемые рыскание=0°, тангаж=10°, крен=0°, газ=1300мкс
    
*/
// Библиотеки
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <helper_3dmath.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h> 

// ----------------- WiFi / UDP ---------------------
const char* ssid = "iphoneMax11";
const char* password = "haskellq";
WiFiUDP udp;
const char* pcIP = "172.20.10.2"; // IP ПК 
const int pcPort = 3333;
const int localPort = 3333;

// ----------------- MPU6050 (DMP) -------------------
MPU6050 mpu(0x68);
bool dmpReady = false;
uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3]; // рыскание, тангаж, крен (радианы)

// Опционально использовать прерывания для DMP 
volatile bool mpuFlag = false;
#define INTERRUPT_PIN 2
void IRAM_ATTR dmpReadyISR() { mpuFlag = true; }

// ----------------- Моторы / Сервоприводы ----------------
Servo motor1, motor2, motor3, motor4;

// Пины моторов  
const int MOTOR_PIN_1 = 13;
const int MOTOR_PIN_2 = 12;
const int MOTOR_PIN_3 = 14;
const int MOTOR_PIN_4 = 27;

// Границы ШИМ для ESC (микросекунды)
const int THROTTLE_MIN = 1000;
const int THROTTLE_MAX = 2000;
int throttleSet = 1100; // значение газа —  изменяется через SET

// Флаг аварийной остановки: при true моторы остановлены 
bool emergencyStop = false;

// ----------------- Управляющие переменные ----------------
bool streamActive = false;      // START/STOP стриминг углов
unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL_MS = 50; // ~20Гц

// Заданные значения (градусы)
float yawDesired   = 0.0;
float pitchDesired = 0.0;
float rollDesired  = 0.0;

// PID коэффициенты для каждой оси 
struct PID {
  float Kp;
  float Ki;
  float Kd;
  float integral;
  float prevErr;
  float outLimit; // абсолютное ограничение выхода 
};
PID pidPitch = { 4.0, 0.02, 0.5, 0.0, 0.0, 1.5 }; 
PID pidRoll  = { 4.0, 0.02, 0.5, 0.0, 0.0, 1.5 };
PID pidYaw   = { 2.0, 0.005, 0.3, 0.0, 0.0, 1.0 };

// Коэффициент преобразования выхода PID в микросекунды мотора
const float MOTOR_SCALE_US = 180.0; // 1.0 единица PID (180 мкс на ESC) 

// Время для цикла PID
uint32_t prevMicros = 0;
const uint32_t PID_PERIOD_US = 8000; // 8 мс (125 Гц)

// ШИМ моторов (микросекунды)
int PWM1=THROTTLE_MIN, PWM2=THROTTLE_MIN, PWM3=THROTTLE_MIN, PWM4=THROTTLE_MIN;

// ---------- Прототипы функций ----------
void armMotors();
void stopAllMotors();
int constrainPWM(int pwm);
float runPID(PID &pid, float setpoint, float measurement, float dt);
void applyMotorMix(float uPitch, float uRoll, float uYaw, int throttleUs);
void processUdpCommand(const String &cmd);

// ----------------- Настройка --------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n\n=== Инициализация коптера ESP32 ===");
  Serial.println("Версия: с PID и управлением моторами");

  // I2C
  Wire.begin(21, 22);
  Wire.setClock(400000);
  Serial.println("I2C инициализирован (SDA=21, SCL=22)");

  // Инициализация MPU
  Serial.println(F("Инициализация MPU6050..."));
  mpu.initialize();

  uint8_t devStatus = mpu.dmpInitialize();
  Serial.print(F("Статус dmpInitialize: "));
  Serial.println(devStatus);

  // Калибровка (
  Serial.println(F("Калибровка (акселерометр+гироскоп) ... Не двигайте датчик!"));
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.PrintActiveOffsets();

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
    Serial.println(F("DMP готов"));
  } else {
    Serial.println(F("Ошибка инициализации DMP"));
    while (1) { delay(1000); }
  }

  pinMode(INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpReadyISR, RISING);

  // Подключение моторов
  motor1.attach(MOTOR_PIN_1, THROTTLE_MIN, THROTTLE_MAX);
  motor2.attach(MOTOR_PIN_2, THROTTLE_MIN, THROTTLE_MAX);
  motor3.attach(MOTOR_PIN_3, THROTTLE_MIN, THROTTLE_MAX);
  motor4.attach(MOTOR_PIN_4, THROTTLE_MIN, THROTTLE_MAX);
 Serial.printf("Моторы подключены: пины %d,%d,%d,%d\n", 
                MOTOR_PIN_1, MOTOR_PIN_2, MOTOR_PIN_3, MOTOR_PIN_4);
  // Начинаем с отключенных моторов 
  stopAllMotors();

  // WiFi
  Serial.print("Подключение WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300); Serial.print(".");
  }
  Serial.println("\nWiFi подключен: " + WiFi.localIP().toString());
  udp.begin(localPort);
  Serial.println("UDP запущен на порту " + String(localPort));

  prevMicros = micros();
  Serial.println("Настройка завершена");
  Serial.println("Команды UDP:");
  Serial.println("  START/STOP - стриминг углов");
  Serial.println("  ARM/DISARM - управление моторами");
  Serial.println("  SET y p r t - установка углов и газа");
  Serial.println("Пример: SET 0 10 0 1300");
  Serial.println("----------------------------\n");
}

// ----------------- Основной цикл ---------------------------
void loop() {
  // Обработка UDP команд
  int packetSizeUdp = udp.parsePacket();
  if (packetSizeUdp) {
    char buf[128];
    int len = udp.read(buf, sizeof(buf)-1);
    if (len > 0) buf[len] = 0;
    String cmd = String(buf);
    cmd.trim();
    Serial.println("UDP получено: " + cmd);
    processUdpCommand(cmd);
  }

  // Чтение DMP когда доступно
  if (!dmpReady) return;

  // Опрос DMP: проверка наличия пакета
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    float yaw  = ypr[0] * 180.0 / M_PI;
    float pitch= ypr[1] * 180.0 / M_PI;
    float roll = ypr[2] * 180.0 / M_PI;

    static unsigned long lastPrint = 0;
      if (millis() - lastPrint > 100) { // Выводим каждые 100 мс
        Serial.print("Yaw:"); Serial.print(yaw, 2);
        Serial.print(", Pitch:"); Serial.print(pitch, 2);
        Serial.print(", Roll:"); Serial.println(roll, 2);
        lastPrint = millis();
      }

    // если углы слишком большие - отключение 
    if (abs(pitch) > 60.0 || abs(roll) > 60.0 || abs(yaw) > 120.0) {
      emergencyStop = true;
      Serial.println("Аварийная остановка: превышен предельный угол");
    }

    // Обновление PID с фиксированной частотой (PID_PERIOD_US)
    uint32_t now = micros();
    uint32_t dt_us = now - prevMicros;
    if (dt_us >= PID_PERIOD_US) {
      float dt = dt_us * 1e-6f; // секунды

      // вычисление выходов PID 
      float uPitch = runPID(pidPitch, pitchDesired, pitch, dt);
      float uRoll  = runPID(pidRoll,  rollDesired,  roll,  dt);
      float uYaw   = runPID(pidYaw,   yawDesired,   yaw,   dt);

      if (millis() - lastPrint > 100) { // Тот же интервал
              Serial.printf("PID: P=%.3f, R=%.3f, Y=%.3f | ", uPitch, uRoll, uYaw);
              Serial.printf("PWM: %d %d %d %d\n", PWM1, PWM2, PWM3, PWM4);
            }
      // применение микширования моторов -> вычисление PWM1..4
      applyMotorMix(uPitch, uRoll, uYaw, throttleSet);

      // запись в моторы (если нет аварийной остановки)
      if (!emergencyStop) {
        motor1.writeMicroseconds(PWM1);
        motor2.writeMicroseconds(PWM2);
        motor3.writeMicroseconds(PWM3);
        motor4.writeMicroseconds(PWM4);
      } else {
        stopAllMotors();
      }

      prevMicros = now;
    }

    // Отправка телеметрии по UDP если стриминг включен
    if (streamActive && (millis() - lastSend >= SEND_INTERVAL_MS)) {
      char msg[80];
      snprintf(msg, sizeof(msg), "%.2f,%.2f,%.2f", yaw, pitch, roll);
      udp.beginPacket(pcIP, pcPort);
      udp.print(msg);
      udp.endPacket();
      lastSend = millis();
    }
  }
}

// ----------------- Функции ----------------------

// Взвод моторов
void armMotors() {
  Serial.println("Взвод моторов...");
  // Установить MIN, подождать, затем плавно поднять до throttleSet
  motor1.writeMicroseconds(THROTTLE_MIN);
  motor2.writeMicroseconds(THROTTLE_MIN);
  motor3.writeMicroseconds(THROTTLE_MIN);
  motor4.writeMicroseconds(THROTTLE_MIN);
  delay(1000);

  // плавный подъем до throttleSet 
  int steps = 20;
  int start = THROTTLE_MIN;
  for (int i=1;i<=steps;i++){
    int t = start + (throttleSet - start) * i / steps;
    motor1.writeMicroseconds(t);
    motor2.writeMicroseconds(t);
    motor3.writeMicroseconds(t);
    motor4.writeMicroseconds(t);
    delay(60);
  }
  Serial.println("Моторы взведены");
}

// установить моторы на MIN
void stopAllMotors() {
  motor1.writeMicroseconds(THROTTLE_MIN);
  motor2.writeMicroseconds(THROTTLE_MIN);
  motor3.writeMicroseconds(THROTTLE_MIN);
  motor4.writeMicroseconds(THROTTLE_MIN);
  PWM1 = PWM2 = PWM3 = PWM4 = THROTTLE_MIN;
}

// ограничение ШИМ
int constrainPWM(int pwm) {
  if (pwm < THROTTLE_MIN) return THROTTLE_MIN;
  if (pwm > THROTTLE_MAX) return THROTTLE_MAX;
  return pwm;
}

// Простая реализация PID (регулятор угла)
float runPID(PID &pid, float setpoint, float measurement, float dt) {
  float err = setpoint - measurement; // градусы
  pid.integral += err * dt;
  float derivative = 0;
  if (dt > 0) derivative = (err - pid.prevErr) / dt;
  pid.prevErr = err;

  float out = pid.Kp * err + pid.Ki * pid.integral + pid.Kd * derivative;

  // ограничение выхода outLimit
  if (out > pid.outLimit) out = pid.outLimit;
  if (out < -pid.outLimit) out = -pid.outLimit;
  return out;
}

// Применение микширования для создания индивидуальных ШИМ моторов
// Микширование предполагает X конфигурацию с нумерацией моторов:
// 1: передний-правый, 2: задний-правый, 3: задний-левый, 4: передний-левый
// Отрегулируйте знаки если ваша нумерация отличается.
void applyMotorMix(float uPitch, float uRoll, float uYaw, int throttleUs) {
  // u* в единицах управления -> преобразовать в микросекунды
  int dPitch = int(uPitch * MOTOR_SCALE_US);
  int dRoll  = int(uRoll  * MOTOR_SCALE_US);
  int dYaw   = int(uYaw   * MOTOR_SCALE_US);

  // Общее микширование (отрегулируйте соглашение о знаках при необходимости)
  // Используется отображение аналогичное исходному коду:
  // THRST1 = 0.25*uh + 0.25*uP + 0.25*uR - 0.25*uY
  // но здесь мы преобразуем разницы непосредственно в микросекунды:
  // вычисляем ШИМ каждого мотора: базовый газ + комбинация
  // Примечание: выберите знаки для получения ожидаемых направлений на стенде
  PWM1 = throttleUs + ( dPitch + dRoll - dYaw ); // мотор 1
  PWM2 = throttleUs + ( dPitch - dRoll + dYaw ); // мотор 2
  PWM3 = throttleUs + ( -dPitch - dRoll - dYaw );// мотор 3
  PWM4 = throttleUs + ( -dPitch + dRoll + dYaw );// мотор 4

  PWM1 = constrainPWM(PWM1);
  PWM2 = constrainPWM(PWM2);
  PWM3 = constrainPWM(PWM3);
  PWM4 = constrainPWM(PWM4);
}

// Обработка UDP команд
// Поддерживаются:
//   START        -> включить стриминг
//   STOP         -> выключить стриминг
//   ARM          -> взвести моторы (плавный подъем)
//   DISARM       -> остановить моторы
//   SET y p r t  -> установить рыскание тангаж крен газ  (градусы, градусы, градусы, микросекунды)
// Пример: "SET 0 10 0 1300"
void processUdpCommand(const String &cmd) {
  if (cmd.equalsIgnoreCase("START")) {
    streamActive = true;
    Serial.println("СТРИМИНГ СТАРТ");
    return;
  }
  if (cmd.equalsIgnoreCase("STOP")) {
    streamActive = false;
    Serial.println("СТРИМИНГ СТОП");
    return;
  }
  if (cmd.equalsIgnoreCase("ARM")) {
    emergencyStop = false;
    armMotors();
    return;
  }
  if (cmd.equalsIgnoreCase("DISARM")) {
    emergencyStop = true;
    stopAllMotors();
    Serial.println("МОТОРЫ ОТКЛЮЧЕНЫ");
    return;
  }

  // разбор SET
  if (cmd.startsWith("SET")) {
    // токенизация
    // ожидается: SET рыскание тангаж крен газ
    float y=0,p=0,r=0;
    int t=throttleSet;
    // используем sscanf
    int parsed = sscanf(cmd.c_str(), "SET %f %f %f %d", &y, &p, &r, &t);
    if (parsed >= 3) {
      yawDesired = y;
      pitchDesired = p;
      rollDesired = r;
      if (parsed == 4) {
        // ограничение газа
        if (t < THROTTLE_MIN) t = THROTTLE_MIN;
        if (t > THROTTLE_MAX) t = THROTTLE_MAX;
        throttleSet = t;
      }
      Serial.printf("SET -> рыскание=%.2f тангаж=%.2f крен=%.2f газ=%d\n", yawDesired, pitchDesired, rollDesired, throttleSet);
    } else {
      Serial.println("Ошибка разбора SET. Используйте: SET рыскание тангаж крен газ");
    }
    return;
  }

  // Опционально: разбор команд настройки PID (например "PID Pp Pi Pd")
  if (cmd.startsWith("PIDPITCH")) {
    float kp, ki, kd;
    int cnt = sscanf(cmd.c_str(), "PIDPITCH %f %f %f", &kp, &ki, &kd);
    if (cnt==3) { pidPitch.Kp=kp; pidPitch.Ki=ki; pidPitch.Kd=kd; Serial.println("PIDPITCH обновлен"); }
    return;
  }
  if (cmd.startsWith("PIDROLL")) {
    float kp, ki, kd;
    int cnt = sscanf(cmd.c_str(), "PIDROLL %f %f %f", &kp, &ki, &kd);
    if (cnt==3) { pidRoll.Kp=kp; pidRoll.Ki=ki; pidRoll.Kd=kd; Serial.println("PIDROLL обновлен"); }
    return;
  }
  if (cmd.startsWith("PIDYAW")) {
    float kp, ki, kd;
    int cnt = sscanf(cmd.c_str(), "PIDYAW %f %f %f", &kp, &ki, &kd);
    if (cnt==3) { pidYaw.Kp=kp; pidYaw.Ki=ki; pidYaw.Kd=kd; Serial.println("PIDYAW обновлен"); }
    return;
  }

  // Если команда не распознана:
  Serial.println("Неизвестная команда");
}
