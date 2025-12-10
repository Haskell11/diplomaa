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
const char* pcIP = "172.20.10.2";  // IP компьютера 
const int pcPort = 3333;
const int localPort = 3333;

// ----------------- MPU6050 (DMP) -------------------
MPU6050 mpu(0x68);
bool dmpReady = false;
uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;

#define YAW 0
#define PITCH 1
#define ROLL 2

// Глобальные переменные 
float theta[3] = {0.0};       // [yaw, pitch, roll] в радианах
float omg[3] = {0.0};         // угловая скорость [yaw, pitch, roll] rad/s

// ----------------- Моторы / Сервоприводы ----------------
Servo motor1, motor2, motor3, motor4;
const int MOTOR_PIN_1 = 26;
const int MOTOR_PIN_2 = 25;
const int MOTOR_PIN_3 = 33;
const int MOTOR_PIN_4 = 32;

// Границы ШИМ для ESC
const int THROTTLE_MIN = 1000;
const int THROTTLE_MAX = 2000;
int THROTTLE0 = 1000;   // программная тяга

// Флаги управления моторами 
bool motorsArmed = false;
bool flag = false;  // простой флаг вместо emergencyStop

// ----------------- Управляющие переменные ----------------
bool streamActive = false;
unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL_MS = 50;

// Заданные значения (в градусах для SET команды)
float yawDesired = 0.0;       // в градусах
float pitchDesired = 0.0;     // в градусах  
float rollDesired = 0.0;      // в градусах
float omgDesired[3] = {0.0};  // заданная угловая скорость (радианы)

// ---------- ПИД коэффициенты ----------
float kYAW = 3.14;     // П-коэффициент по рысканию
float kPITCH = 3.14;   // П-коэффициент по тангажу  
float kROLL = 5.13;    // П-коэффициент по крену 

// Коэффициенты цифрового ПИД регулятора
float qP[5] = {0.0};
float qR[5] = {0.0};
float qY[5] = {0.0};

// Ошибки управления
float thetaErrs[3] = {0.0};  // ошибки по углам (радианы)
float omgErrs[3] = {0.0};    // ошибки по угловой скорости

// Векторы ошибок для цифрового ПИД (3 последних значения)
float yawRateErr[3] = {0.0};
float pitchRateErr[3] = {0.0};
float rollRateErr[3] = {0.0};

// Выходы ПИД регуляторов
float uY[3] = {0.0};  // yaw
float uP[3] = {0.0};  // pitch
float uR[3] = {0.0};  // roll

// Выходы управления (после насыщения)
float uRotation[3] = {0.0}; // [yaw, pitch, roll]

// Силы тяги моторов
float THRST1 = 0.0, THRST2 = 0.0, THRST3 = 0.0, THRST4 = 0.0;

// ШИМ сигналы
int PWM1 = THROTTLE_MIN, PWM2 = THROTTLE_MIN, PWM3 = THROTTLE_MIN, PWM4 = THROTTLE_MIN;

// Параметры кривой PWM-T 
float pMotor[3] = {1.17E-6, 0.91E-3, -52.88E-3};

// Таймеры
uint32_t startMillis;
uint32_t tmr;
const uint32_t PID_PERIOD_US = 8000; // 8 мс (TPID)
float TAU = 0.01; // 10 мс в секундах

// Ограничения 
float att_threshold[3] = {45*PI/180, 45*PI/180, 45*PI/180}; // для плавного полета
float upitch_max = 2.0, uroll_max = 2.0, uyaw_max = 2.0/3.0;

// Прототипы функций
void armMotors();
void stopAllMotors();
void processUdpCommand(const String &cmd);
void PID_DIGITAL(float q[], float err[], float u[]);
void vectorstore(float arr[], float val);
float SATURATION(float x, float upper, float lower);
float LMT(float x, float threshold);
int T2PWM(float T, int T0, int Tmin, int Tmax, float p[]);
void GetErrs(float xd[], float x[], float dx[]);
void FindAngleRateDesired(float angleErr[], float angleRated[], float kY, float kP, float kR);
void ANGLERATEFILTER(int16_t GYR[], float omg[], float dt, float cutoff);
void DIGPIDCOEFF(float q[], float kPa, float kIa, float kDa, float TAU);
float DigLowPassFil(float x_prev, float u, float u_prev, float DT, float w0);
float PREGULATOR(float Err, float k, float dxmax);  
void printAngles();
void SignalLMT(float arr[], float threshold[]);
float DEADZONE(float x, float threshold);
void SignalDEADZONE(float x[], float threshold[]);
void StopDrone(int PWM0);

// ----------------- Настройка --------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n\n=== Инициализация коптера ESP32 ===");
  Serial.println("Моторы заблокированы до команды ARM");

  // I2C
  Wire.begin(21, 22);
  Wire.setClock(400000);
  Serial.println("I2C инициализирован (SDA=21, SCL=22)");

  // Инициализация MPU
  Serial.println(F("Инициализация MPU6050..."));
  mpu.initialize();

  Serial.print(F("Тестирование соединения с MPU6050: "));
  Serial.println(mpu.testConnection() ? F("УСПЕШНО") : F("ОШИБКА"));

  uint8_t devStatus = mpu.dmpInitialize();
  Serial.print(F("Статус dmpInitialize: "));
  Serial.println(devStatus);

  // Ручная калибровка (как в рабочем коде)
  mpu.setXAccelOffset(-5182);
  mpu.setYAccelOffset(-5396);
  mpu.setZAccelOffset(9056);
  mpu.setXGyroOffset(106);
  mpu.setYGyroOffset(-44);
  mpu.setZGyroOffset(1);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
    Serial.println(F("DMP готов"));
  } else {
    Serial.println(F("Ошибка инициализации DMP"));
    while (1) { delay(1000); }
  }

  // Подключение моторов
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  motor1.attach(MOTOR_PIN_1);
  delay(300);
  motor2.attach(MOTOR_PIN_2);
  delay(300);
  motor3.attach(MOTOR_PIN_3);
  delay(300);
  motor4.attach(MOTOR_PIN_4);
  delay(300);
  
  Serial.printf("Моторы подключены: пины %d,%d,%d,%d\n", 
                MOTOR_PIN_1, MOTOR_PIN_2, MOTOR_PIN_3, MOTOR_PIN_4);
  
  // Моторы заблокированы
  motor1.writeMicroseconds(THROTTLE_MIN);
  motor2.writeMicroseconds(THROTTLE_MIN);
  motor3.writeMicroseconds(THROTTLE_MIN);
  motor4.writeMicroseconds(THROTTLE_MIN);
  
  Serial.println("Моторы заблокированы. Отправьте ARM из Python для разблокировки.");

  // WiFi
  Serial.print("Подключение WiFi...");
  WiFi.begin(ssid, password);
  int wifiTimeout = 0;
  while (WiFi.status() != WL_CONNECTED && wifiTimeout < 20) {
    delay(500); 
    Serial.print(".");
    wifiTimeout++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi подключен: " + WiFi.localIP().toString());
    udp.begin(localPort);
    Serial.println("UDP запущен на порту " + String(localPort));
  } else {
    Serial.println("\n❌ Не удалось подключиться к WiFi!");
  }

  // Инициализация таймеров
  startMillis = millis();
  tmr = micros();

  // ------ Инициализация параметров ПИД регуляторов (ИСПРАВЛЕНО) ------
  DIGPIDCOEFF(qP, 0.195, 1.01, 0.0006, TAU);   // Pitch (было 0.017)
  DIGPIDCOEFF(qR, 0.18, 0.4, 0.0006, TAU);     // Roll (было 0.0005)       
  DIGPIDCOEFF(qY, 0.195, 1.01, 0.034, TAU);    // Yaw (0.017*2 = 0.034)

  Serial.println("\n=== НАСТРОЙКА ЗАВЕРШЕНА ===");
}

// ----------------- Основной цикл ---------------------------
void loop() {
  // Обработка UDP команд от Python-программы
  int packetSizeUdp = udp.parsePacket();
  if (packetSizeUdp) {
    char buf[128];
    int len = udp.read(buf, sizeof(buf)-1);
    if (len > 0) buf[len] = 0;
    String cmd = String(buf);
    cmd.trim();
    processUdpCommand(cmd);
  }

  if (!dmpReady) return;

  // Время с последнего обновления
  uint32_t DLT = micros() - tmr;
  
  // Основной цикл управления с периодом 8 мс
  if (DLT >= PID_PERIOD_US && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    float DT = DLT * 1e-6;  // Время в секундах
    tmr = micros();
    
    // Получение данных с DMP
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(theta, &q, &gravity);
    
    // Получение сырых данных гироскопа
    int16_t GYR[3];
    mpu.dmpGetGyro(GYR, fifoBuffer);
    
    // Фильтрация угловой скорости 
    ANGLERATEFILTER(GYR, omg, DT, 40.0);
    
    // Время с момента старта 
    uint32_t TM = millis() - startMillis;
    const uint32_t TNORMENGINE = 4000;  // 4 секунды на разгон
    
    // Плавный набор газа 
    if (!flag && motorsArmed) {
      if (TM < TNORMENGINE) {
        THROTTLE0 = 1100 + int(200.0 / TNORMENGINE * TM);  // 1100 → 1300 за 4 секунды
      } else {
        THROTTLE0 = 1300;  // установившаяся тяга
      }
    } else if (flag) {
      THROTTLE0 = 1000;  // если выключено
    }
    
    // Преобразуем желаемые углы из градусов в радианы
    float thetaDesiredRad[3] = {
      yawDesired * PI / 180.0,    // yaw в радианы
      pitchDesired * PI / 180.0,  // pitch в радианы
      rollDesired * PI / 180.0    // roll в радианы
    };
    
    // Расчет отклонений
    GetErrs(thetaDesiredRad, theta, thetaErrs);
    
    // Ограничение отклонений 
    SignalLMT(thetaErrs, att_threshold);
    
    // Вычисление заданной угловой скорости через П-регулятор
    FindAngleRateDesired(thetaErrs, omgDesired, kYAW, kPITCH, kROLL);
    
    // Ошибка по угловой скорости
    GetErrs(omgDesired, omg, omgErrs);
    
    // Сохраняем ошибки для цифрового ПИД  
    vectorstore(yawRateErr, omgErrs[YAW]);
    vectorstore(pitchRateErr, omgErrs[PITCH]);
    vectorstore(rollRateErr, omgErrs[ROLL]);
    
    // Выполнение алгоритмов ПИД регуляторов 
    PID_DIGITAL(qY, yawRateErr, uY);
    PID_DIGITAL(qP, pitchRateErr, uP);
    PID_DIGITAL(qR, rollRateErr, uR);
    
    // Насыщение выходных сигналов 
    uRotation[YAW]   = SATURATION(uY[0], uyaw_max, -uyaw_max);        // рыскание
    uRotation[PITCH] = SATURATION(uP[0], upitch_max, -upitch_max);    // тангаж
    uRotation[ROLL]  = SATURATION(uR[0], uroll_max, -uroll_max);      // крен
    
    // Базовое значение тяги (управление высотой)
    float uh = 0.0;
    
    // Расчет требуемых сил тяги для всех моторов 
    THRST1 = 0.25 * uh + 0.25 * uRotation[PITCH] + 0.25 * uRotation[ROLL] - 0.25 * uRotation[YAW];
    THRST2 = 0.25 * uh + 0.25 * uRotation[PITCH] - 0.25 * uRotation[ROLL] + 0.25 * uRotation[YAW];
    THRST3 = 0.25 * uh - 0.25 * uRotation[PITCH] - 0.25 * uRotation[ROLL] - 0.25 * uRotation[YAW];
    THRST4 = 0.25 * uh - 0.25 * uRotation[PITCH] + 0.25 * uRotation[ROLL] + 0.25 * uRotation[YAW];
    
    // Расчет ШИМ сигналов для моторов 
    if (!flag && motorsArmed) {
      PWM1 = T2PWM(THRST1, THROTTLE0, THROTTLE_MIN, THROTTLE_MAX, pMotor);
      PWM2 = T2PWM(THRST2, THROTTLE0, THROTTLE_MIN, THROTTLE_MAX, pMotor);
      PWM3 = T2PWM(THRST3, THROTTLE0, THROTTLE_MIN, THROTTLE_MAX, pMotor);
      PWM4 = T2PWM(THRST4, THROTTLE0, THROTTLE_MIN, THROTTLE_MAX, pMotor);
    } else {
      PWM1 = PWM2 = PWM3 = PWM4 = THROTTLE_MIN;
    }
    
    // Вывод углов в Serial с дополнительной информацией
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 100) {
      printAngles();
      lastPrint = millis();
    }
  }
  
  // Контроль безопасности 
  uint32_t TM = millis() - startMillis;
  if ( abs(theta[PITCH]) >= 50*PI/180 || abs(theta[ROLL]) >= 50*PI/180 || abs(theta[YAW]) >= 80*PI/180) {
    flag = true;
    Serial.println("Аварийная остановка: превышен предельный угол!");
  }
  
  // Если выключено, плавно снижаем тягу
  if (flag) {
    static float fT = float(THROTTLE0);
    if (THROTTLE0 > 1000) {
      fT = fT - 4.0 / 100;
      THROTTLE0 = int(fT);
    } else {
      THROTTLE0 = 1000;
    }
    StopDrone(THROTTLE0);
    motorsArmed = false;  //  сбрасываем флаг взвода
  }
  
  // Запись ШИМ в моторы (всегда)
  motor1.writeMicroseconds(PWM1);
  motor2.writeMicroseconds(PWM2);
  motor3.writeMicroseconds(PWM3);
  motor4.writeMicroseconds(PWM4);
  
  // Отправка телеметрии по UDP в Python-программу
  if (streamActive && (millis() - lastSend >= SEND_INTERVAL_MS)) {
    float yaw_deg = theta[YAW] * 180.0 / PI;
    float pitch_deg = theta[PITCH] * 180.0 / PI;
    float roll_deg = theta[ROLL] * 180.0 / PI;
    
    char msg[80];
    snprintf(msg, sizeof(msg), "%.2f,%.2f,%.2f", yaw_deg, pitch_deg, roll_deg);
    udp.beginPacket(pcIP, pcPort);
    udp.print(msg);
    udp.endPacket();
    lastSend = millis();
  }
  
  delayMicroseconds(100);
}

// ----------------- ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ----------------------

// Ограничение сигнала 
void SignalLMT(float arr[], float threshold[]) {
    for (int i = 0; i < 3; i++) {
        arr[i] = LMT(arr[i], threshold[i]);
    }
}

// Функция для вывода углов в нужном формате
void printAngles() {
  float yaw_deg = theta[YAW] * 180.0 / PI;
  float pitch_deg = theta[PITCH] * 180.0 / PI;
  float roll_deg = theta[ROLL] * 180.0 / PI;
  
  Serial.print("Yaw:");
  Serial.print(yaw_deg, 2);
  Serial.print(", Pitch:");
  Serial.print(pitch_deg, 2);
  Serial.print(", Roll:");
  Serial.print(roll_deg, 2);
  
  if(motorsArmed){
      Serial.print(", THROTTLE0:");
      Serial.print(THROTTLE0);
      Serial.print(", PWM:");
      Serial.print(PWM1);
      Serial.print(",");
      Serial.print(PWM2);
      Serial.print(",");
      Serial.print(PWM3);
      Serial.print(",");
      Serial.println(PWM4);
  } else {
      Serial.println();  // просто перенос строки
  }
}

// Вычисление коэффициентов цифровых ПИД регуляторов 
void DIGPIDCOEFF(float q[], float kPa, float kIa, float kDa, float TAU) {  
  float kP, kI, kD, d;
  int N = 10;
  
  kP = kPa;
  kI = TAU * kIa;
  kD = N * kDa / (kDa + N * TAU);
  d = kD / N;  
  
  q[0] = kP + kI + kD;
  q[1] = -kP * (1 + d) - kI * d - 2 * kD;
  q[2] = kP * d + kD;
  q[3] = 1 + d;
  q[4] = -d;
}

// Квадратный корневой регулятор 
float PREGULATOR(float Err, float k, float dxmax) {
    float xd = 0.0;
    float theta0 = dxmax / k / k;
    float theta1 = 0.5 * theta0;
    
    if (fabs(Err) > theta0)
        xd = pow(2 * dxmax * (fabs(Err) - theta1), 0.5) * fabs(Err) / Err;
    else
        xd = k * Err;    
    
    return xd;
}

// Формирование вектора требуемых угловых скоростей 
void FindAngleRateDesired(float angleErr[], float angleRated[], float kY, float kP, float kR) {
    float rollrate, pitchrate, yawrate;
    float dwmax = 360.0 * PI / 180;  // максимальное ускорение в rad/s²
    
    yawrate = PREGULATOR(angleErr[YAW], kY, dwmax);
    pitchrate = PREGULATOR(angleErr[PITCH], kP, dwmax);
    rollrate = PREGULATOR(angleErr[ROLL], kR, dwmax);

    // учет ограничения 
    angleRated[YAW] = LMT(yawrate, 90 * PI / 180);
    angleRated[PITCH] = LMT(pitchrate, 180 * PI / 180);  
    angleRated[ROLL] = LMT(rollrate, 180 * PI / 180);    
}

// Реализация алгоритмов цифровых ПИД регуляторов (ИСПРАВЛЕНО ИМЯ)
void PID_DIGITAL(float q[], float err[], float u[]) {           
    float unew;
    unew = q[3] * u[0] + q[4] * u[1] + q[0] * err[0] + q[1] * err[1] + q[2] * err[2];   
    
    u[2] = u[1];
    u[1] = u[0];
    u[0] = unew;
}

// Расчет ШИМ сигнала по требуемому значению тяги (ИСПРАВЛЕНО)
  int T2PWM(float T, int THROTTLE0, int THROTTLE_MIN, int THROTTLE_MAX, float p[]) {
    float T0, TSUM, PWM;
    
    // сила тяги при THROTTLE0
    T0 = p[0] * (THROTTLE0 - 1000) * (THROTTLE0 - 1000) + 
         p[1] * (THROTTLE0 - 1000) + 
         p[2];
    
    // суммарная сила тяги
    TSUM = T + T0;
    
    if (TSUM < 0) {
        TSUM = 0;
    }
    
    // PWM соответствующий суммарной тяге
    PWM = (-p[1] + sqrt(p[1]*p[1] - 4*p[0]*(p[2] - TSUM))) * 0.5/p[0] + 1000; 
    
    // Ограничение (saturation)
    if (PWM < THROTTLE_MIN) {
        PWM = THROTTLE_MIN;
    }
    if (PWM > THROTTLE_MAX) {
        PWM = THROTTLE_MAX;
    }
    
    return (int)PWM;
}

// Фильтрация угловой скорости 
void ANGLERATEFILTER(int16_t GYR[], float omg[], float DT, float w0) {
    static float gyrox[2] = {0.0}, gyroy[2] = {0.0}, gyroz[2] = {0.0};
    float omgx, omgy, omgz;

    // расчет угловой скорости 
    gyrox[0] = (float)GYR[0] / 32768 * 2000 / 180 * PI;   // roll
    gyroy[0] = -(float)GYR[1] / 32768 * 2000 / 180 * PI;  // pitch
    gyroz[0] = -(float)GYR[2] / 32768 * 2000 / 180 * PI;  // yaw
    
    // фильтрация показаний гироскопов 
    omgx = DigLowPassFil(omg[0], gyrox[0], gyrox[1], DT, w0);
    omgy = DigLowPassFil(omg[1], gyroy[0], gyroy[1], DT, w0);
    omgz = DigLowPassFil(omg[2], gyroz[0], gyroz[1], DT, w0);

    
    gyrox[1] = gyrox[0];
    gyroy[1] = gyroy[0];
    gyroz[1] = gyroz[0];

    // вывод
    omg[ROLL] = omgx;
    omg[PITCH] = omgy;
    omg[YAW] = omgz;
}

// Цифровой фильтр низких частот 
float DigLowPassFil(float x_prev, float u, float u_prev, float DT, float w0) {
    float a0 = DT * w0 + 2;
    float a1 = DT * w0 - 2;
    float b = DT * w0;
    return (-a1 * x_prev + b * (u + u_prev)) / a0;
}

// Ограничение сигнала 
float LMT(float x, float threshold) {
    if (fabs(x) > threshold) {
        return threshold * fabs(x) / x;
    }
    return x;
}

// Насыщение
float SATURATION(float x, float upper, float lower) {
    if (x > upper) return upper;
    if (x < lower) return lower;
    return x;
}

// Зона нечувствительности
float DEADZONE(float x, float threshold) {
    if (fabs(x) < threshold) return 0.0;
    if (x > 0) return x - threshold;
    return x + threshold;
}

void SignalDEADZONE(float x[], float threshold[]) {
    x[0] = DEADZONE(x[0], threshold[0]);    
    x[1] = DEADZONE(x[1], threshold[1]);
    x[2] = DEADZONE(x[2], threshold[2]); 
}

// Сохранение значения в циклический буфер
void vectorstore(float arr[], float val) {
    arr[2] = arr[1];
    arr[1] = arr[0];
    arr[0] = val;
}

// Вычисление ошибок управления
void GetErrs(float xd[], float x[], float dx[]) {
    dx[0] = xd[0] - x[0];
    dx[1] = xd[1] - x[1];
    dx[2] = xd[2] - x[2];
}

// Взвод моторов 
void armMotors() {
    if (flag) {
        Serial.println("ОШИБКА: Аварийная остановка активна!");
        return;
    }
    
    if (motorsArmed) {
        Serial.println("Моторы уже взведены!");
        return;
    }
    
    Serial.println("=== ПРОЦЕСС ВЗВОДА МОТОРОВ ===");
    Serial.println("Установка минимального газа...");
    motor1.writeMicroseconds(THROTTLE_MIN);
    motor2.writeMicroseconds(THROTTLE_MIN);
    motor3.writeMicroseconds(THROTTLE_MIN);
    motor4.writeMicroseconds(THROTTLE_MIN);
    delay(5000);  // 5 секунд на инициализацию ESC
    
    Serial.println("Моторы готовы к работе!");
    motorsArmed = true;
    THROTTLE0 = 1000;  // начинаем с минимального газа
    Serial.println("=== МОТОРЫ ВЗВЕДЕНЫ ===");
}

// Выключить все моторы 
void StopDrone(int PWM0) {
    PWM1 = PWM0;
    PWM2 = PWM0;
    PWM3 = PWM0;
    PWM4 = PWM0;
}

// Обработка UDP команд от Python-программы
void processUdpCommand(const String &cmd) {
    Serial.print("UDP получено: ");
    Serial.println(cmd);
    
    if (cmd.equalsIgnoreCase("START")) {
        streamActive = true;
        Serial.println("СТРИМИНГ ТЕЛЕМЕТРИИ: ВКЛ");
        return;
    }
    
    if (cmd.equalsIgnoreCase("STOP")) {
        streamActive = false;
        Serial.println("СТРИМИНГ ТЕЛЕМЕТРИИ: ВЫКЛ");
        return;
    }
    
    if (cmd.equalsIgnoreCase("ARM")) {
        flag = false;  // сброс аварийного флага
        armMotors();
        return;
    }
    
    if (cmd.equalsIgnoreCase("DISARM")) {
        flag = true;  // установка флага выключения
        motorsArmed = false;
        Serial.println("МОТОРЫ ОТКЛЮЧЕНЫ");
        return;
    }
    
    if (cmd.equalsIgnoreCase("RESET")) {
        flag = false;
        Serial.println("Аварийная остановка сброшена");
        return;
    }

    // Команда SET от Python-программы: SET yaw pitch roll throttle
    if (cmd.startsWith("SET")) {
        if (!motorsArmed) {
            Serial.println("ОШИБКА: Моторы не взведены! Отправьте команду ARM сначала.");
            return;
        }
        
        float y = 0, p = 0, r = 0;
        int t = THROTTLE0;
        
        int parsed = sscanf(cmd.c_str(), "SET %f %f %f %d", &y, &p, &r, &t);
        if (parsed >= 3) {
            yawDesired = y;
            pitchDesired = p;
            rollDesired = r;
            
            if (parsed == 4) {
                if (t < THROTTLE_MIN) t = THROTTLE_MIN;
                if (t > THROTTLE_MAX) t = THROTTLE_MAX;
                THROTTLE0 = t;
            }
            
            Serial.printf("SET -> рыскание=%.2f° тангаж=%.2f° крен=%.2f° газ=%d\n", 
                         yawDesired, pitchDesired, rollDesired, THROTTLE0);
        } else {
            Serial.println("Ошибка разбора SET. Используйте: SET рыскание тангаж крен газ");
        }
        return;
    }

    Serial.println("Неизвестная команда");
}