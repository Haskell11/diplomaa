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
const char* pcIP = "172.20.10.2";  // IP  компьютера 
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


#define YAW 0
#define PITCH 1
#define ROLL 2

// Глобальные переменные 
float theta[3] = {0.0};       // [yaw, pitch, roll] в радианах
float omg[3] = {0.0};         // угловая скорость [yaw, pitch, roll] rad/s
float omg_prev[3] = {0.0};    // предыдущая угловая скорость

// ----------------- Моторы / Сервоприводы ----------------
Servo motor1, motor2, motor3, motor4;
const int MOTOR_PIN_1 = 25;
const int MOTOR_PIN_2 = 26;
const int MOTOR_PIN_3 = 32;
const int MOTOR_PIN_4 = 33;

// Границы ШИМ для ESC
const int THROTTLE_MIN = 1000;
const int THROTTLE_MAX = 2000;
int throttleSet = 1100; // программная тяга
int THROTTLE0 = 1100;   // аналогично THROLLTE0 

// Флаги управления моторами
bool emergencyStop = false;
bool motorsArmed = false;
bool motorsLocked = true;

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
float kROLL = 3.14;    // П-коэффициент по крену

// Коэффициенты цифрового ПИД регулятора  - будут вычислены
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
uint32_t tmr, tmr_motion;
const uint32_t PID_PERIOD_US = 8000; // 8 мс (TPID)
float TAU = 0.01; // 10 мс в секундах

// Прототипы функций
void armMotors();
void stopAllMotors();
void calibrateMotors();
void processUdpCommand(const String &cmd);
void digitalPID(float q[], float err[], float u[]);
void vectorstore(float arr[], float val);
float SATURATION(float x, float upper, float lower);
float LMT(float x, float threshold);
float DEADZONE(float x, float threshold);
int T2PWM(float T, int T0, int Tmin, int Tmax, float p[]);
void GetErrs(float xd[], float x[], float dx[]);
void FindAngleRateDesired(float thetaErr[], float omgDesired[], float kY, float kP, float kR);
void ANGLERATEFILTER(int16_t GYR[], float omg[], float dt, float cutoff);
void DIGPIDCOEFF(float q[], float kPa, float kIa, float kDa, float TAU);
float DigLowPassFil(float y_prev, float x0, float x1, float T, float w0);
float PREGULATOR(float thetaErr, float k);
void printAngles();  // Функция для вывода углов

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

  // Автоматическая калибровка
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
  tmr_motion = millis();

  // ------ Инициализация параметров ПИД регуляторов  ------
  // вычисление коэффициентов цифровых ПИД регуляторов
  DIGPIDCOEFF(qP, 0.195, 1.01, 0.017*1.5, TAU);   // Pitch  
  DIGPIDCOEFF(qR, 0.195, 1.01, 0.017*1.5, TAU);   // Roll       
  DIGPIDCOEFF(qY, 0.195, 1.01, 0.017*2, TAU);     // Yaw

  Serial.println("\n=== НАСТРОЙКА ЗАВЕРШЕНА ===");
  Serial.println("Система готова к приему команд из Python.");
  Serial.println("\n=== ДОСТУПНЫЕ КОМАНДЫ UDP ===");
  Serial.println("  START/STOP  - стриминг углов на ПК");
  Serial.println("  ARM         - ВЗВЕСТИ моторы (разблокировать)");
  Serial.println("  DISARM      - ЗАГЛУШИТЬ моторы (заблокировать)");
  Serial.println("  RESET       - сброс аварийной остановки");
  Serial.println("  SET y p r t - установка углов и газа");
  Serial.println("\n=== ПРИМЕР ИСПОЛЬЗОВАНИЯ ===");
  Serial.println("1. Отправьте ARM из Python для разблокировки");
  Serial.println("2. Отправьте START для телеметрии");
  Serial.println("3. Отправьте SET 0 5 0 1150 для наклона вперед");
  Serial.println("4. Отправьте DISARM для остановки");
  Serial.println("=============================\n");
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
    
    // Преобразуем желаемые углы из градусов в радианы
    float thetaDesiredRad[3] = {
      yawDesired * PI / 180.0,    // yaw в радианы
      pitchDesired * PI / 180.0,  // pitch в радианы
      rollDesired * PI / 180.0    // roll в радианы
    };
    
    // Расчет отклонений - 
    GetErrs(thetaDesiredRad, theta, thetaErrs);
    
    // Вычисление заданной угловой скорости через П-регулятор
    FindAngleRateDesired(thetaErrs, omgDesired, kYAW, kPITCH, kROLL);
    
    // Ошибка по угловой скорости
    GetErrs(omgDesired, omg, omgErrs);
    
    // Сохраняем ошибки для цифрового ПИД  
    vectorstore(yawRateErr, omgErrs[YAW]);
    vectorstore(pitchRateErr, omgErrs[PITCH]);
    vectorstore(rollRateErr, omgErrs[ROLL]);
    
    // Выполнение алгоритмов ПИД регуляторов 
    digitalPID(qY, yawRateErr, uY);
    digitalPID(qP, pitchRateErr, uP);
    digitalPID(qR, rollRateErr, uR);
    
    // Насыщение выходных сигналов - ИДЕНТИЧНЫЕ ЗНАЧЕНИЯ
    uRotation[YAW]   = SATURATION(uY[0], 2.0/3.0, -2.0/3.0);   // рыскание
    uRotation[PITCH] = SATURATION(uP[0], 2.0, -2.0);           // тангаж
    uRotation[ROLL]  = SATURATION(uR[0], 2.0, -2.0);           // крен
    
    // Базовое значение тяги 
    float uh = 0.0;
    
    // Расчет требуемых сил тяги для всех моторов 
   
    THRST1 = 0.25 * uh + 0.25 * uRotation[PITCH] + 0.25 * uRotation[ROLL] - 0.25 * uRotation[YAW];
    THRST2 = 0.25 * uh + 0.25 * uRotation[PITCH] - 0.25 * uRotation[ROLL] + 0.25 * uRotation[YAW];
    THRST3 = 0.25 * uh - 0.25 * uRotation[PITCH] - 0.25 * uRotation[ROLL] - 0.25 * uRotation[YAW];
    THRST4 = 0.25 * uh - 0.25 * uRotation[PITCH] + 0.25 * uRotation[ROLL] + 0.25 * uRotation[YAW];
    
    // Расчет ШИМ сигналов для моторов 
    PWM1 = T2PWM(THRST1, THROTTLE0, THROTTLE_MIN, THROTTLE_MAX, pMotor);
    PWM2 = T2PWM(THRST2, THROTTLE0, THROTTLE_MIN, THROTTLE_MAX, pMotor);
    PWM3 = T2PWM(THRST3, THROTTLE0, THROTTLE_MIN, THROTTLE_MAX, pMotor);
    PWM4 = T2PWM(THRST4, THROTTLE0, THROTTLE_MIN, THROTTLE_MAX, pMotor);
    
    // Вывод углов в Serial в формате "Yaw:X.XX, Pitch:X.XX, Roll:X.XX"
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 100) {
      printAngles();
      lastPrint = millis();
    }
  }
  
  // Контроль безопасности
  uint32_t TM = millis() - startMillis;
  if ((TM > 60000) && (abs(theta[PITCH]) >= 60 * PI / 180 || 
                       abs(theta[ROLL]) >= 60 * PI / 180 || 
                       abs(theta[YAW]) >= 100 * PI / 180)) {
    emergencyStop = true;
    Serial.println("Аварийная остановка: превышен предельный угол!");
  }
  
  // Аварийная остановка
  if (emergencyStop) {
    static float fT = float(THROTTLE0);
    if (THROTTLE0 > 1200) {
      fT = fT - 4.0 / 100;
      THROTTLE0 = int(fT);
    } else {
      THROTTLE0 = 1000;
    }
    PWM1 = PWM2 = PWM3 = PWM4 = THROTTLE0;
  }
  
  // Запись ШИМ в моторы
  if (!emergencyStop && motorsArmed) {
    motor1.writeMicroseconds(PWM1);
    motor2.writeMicroseconds(PWM2);
    motor3.writeMicroseconds(PWM3);
    motor4.writeMicroseconds(PWM4);
  } else {
    // Если моторы заблокированы или аварийная остановка
    motor1.writeMicroseconds(THROTTLE_MIN);
    motor2.writeMicroseconds(THROTTLE_MIN);
    motor3.writeMicroseconds(THROTTLE_MIN);
    motor4.writeMicroseconds(THROTTLE_MIN);
  }
  
  // Отправка телеметрии по UDP в Python-программу
  //теперь отправляем в формате "Yaw,Pitch,Roll"
  if (streamActive && (millis() - lastSend >= SEND_INTERVAL_MS)) {
    float yaw_deg = theta[YAW] * 180.0 / PI;
    float pitch_deg = theta[PITCH] * 180.0 / PI;
    float roll_deg = theta[ROLL] * 180.0 / PI;
    
    //  формат "Yaw,Pitch,Roll" вместо "Roll,Pitch,Yaw"
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

// Функция для вывода углов в нужном формате: "Yaw:X.XX, Pitch:X.XX, Roll:X.XX"
void printAngles() {
  float yaw_deg = theta[YAW] * 180.0 / PI;
  float pitch_deg = theta[PITCH] * 180.0 / PI;
  float roll_deg = theta[ROLL] * 180.0 / PI;
  
  // Формат: "Yaw:X.XX, Pitch:X.XX, Roll:X.XX"
  Serial.print("Yaw:");
  Serial.print(yaw_deg, 2);
  Serial.print(", Pitch:");
  Serial.print(pitch_deg, 2);
  Serial.print(", Roll:");
  Serial.println(roll_deg, 2);
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

// квадратный корневой регулятор 
float PREGULATOR(float thetaErr, float k) {
    float wd = 0.0, wmax = 180.0 * PI / 180, amax = 360.0 * PI / 180;
    float theta0 = amax / k / k, theta1 = 0.5 * theta0;
    
    if (fabs(thetaErr) > theta0)
        wd = pow(2 * amax * (fabs(thetaErr) - theta1), 0.5) * fabs(thetaErr) / thetaErr;
    else
        wd = k * thetaErr;    
    
    return wd;
}

// формирование вектора требуемых угл. скоростей 
void FindAngleRateDesired(float angleErr[], float angleRated[], float kY, float kP, float kR) {
    float rollrate, pitchrate, yawrate;

    
    yawrate = PREGULATOR(angleErr[YAW], kY);
    pitchrate = PREGULATOR(angleErr[PITCH], kP);
    rollrate = PREGULATOR(angleErr[ROLL], kR);

    // учет ограничения
    angleRated[YAW] = LMT(yawrate, 90 * PI / 180);
    angleRated[PITCH] = LMT(pitchrate, 360 * PI / 180);
    angleRated[ROLL] = LMT(rollrate, 360 * PI / 180);
}

// Реализация алгоритмов цифровых ПИД регуляторов 
void digitalPID(float q[], float err[], float u[]) {           
    float unew;
    // расчет нового значения корректирующего сигнала управления
    unew = q[3] * u[0] + q[4] * u[1] + q[0] * err[0] + q[1] * err[1] + q[2] * err[2];    
    
    // хранение для следующего вызова
    u[2] = u[1];
    u[1] = u[0];
    u[0] = unew;
}

// Расчет ШИМ сигнала по требуемому значению тяги
int T2PWM(float T, int PWM0, int THROTTLE_MIN, int THROTTLE_MAX, float p[]) {
    float T0, TSUM, PWM;
    
    // сила тяги при THROTTLE0
    T0 = p[0] * (PWM0 - 1000) * (PWM0 - 1000) + p[1] * (PWM0 - 1000) + p[2];
    
    // суммарная сила тяги
    TSUM = T + T0;
    if (TSUM < 0) {
      TSUM = 0;
    }
    
    // PWM соответствующий суммарной тяге
    PWM = (-p[1] + sqrt(p[1] * p[1] - 4 * p[0] * (p[2] - TSUM))) * 0.5 / p[0] + 1000; 
    
    // ограничение (saturation)
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
    gyrox[0] = (float)GYR[0] / 32768 * 2000 / 180 * PI;   // в рад/с - roll
    gyroy[0] = -(float)GYR[1] / 32768 * 2000 / 180 * PI;  // pitch
    gyroz[0] = -(float)GYR[2] / 32768 * 2000 / 180 * PI;  // yaw
    
    // фильтрация показаний гироскопов
    omgx = DigLowPassFil(omg[0], gyrox[0], gyrox[1], DT, w0);
    omgy = DigLowPassFil(omg[1], gyroy[0], gyroy[1], DT, w0);
    omgz = DigLowPassFil(omg[2], gyroz[0], gyroz[1], DT, w0);

    // запомнить
    gyrox[1] = gyrox[0];
    gyroy[1] = gyroy[0];
    gyroz[1] = gyroz[0];

    // вывод
    omg[ROLL] = omgx;
    omg[PITCH] = omgy;
    omg[YAW] = omgz;
}

// Цифровой фильтр низких частот
float DigLowPassFil(float y_prev, float x0, float x1, float T, float w0) {
    float RC = 1.0 / (2.0 * PI * w0);
    float alpha = T / (RC + T);
    return alpha * x0 + (1 - alpha) * y_prev;
}

// Ограничение сигнала 
float LMT(float x, float threshold) {
    if (x > threshold) return threshold;
    if (x < -threshold) return -threshold;
    return x;
}

// Мертвая зона
float DEADZONE(float x, float threshold) {
    if (abs(x) < threshold) return 0.0;
    return x;
}

// Насыщение
float SATURATION(float x, float upper, float lower) {
    if (x > upper) return upper;
    if (x < lower) return lower;
    return x;
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
    if (emergencyStop) {
        Serial.println("ОШИБКА: Аварийная остановка активна!");
        return;
    }
    
    if (motorsArmed) {
        Serial.println("Моторы уже взведены!");
        return;
    }
    
    Serial.println("=== ПРОЦЕСС ВЗВОДА МОТОРОВ ===");
    Serial.println("1. Установка минимального газа...");
    motor1.writeMicroseconds(THROTTLE_MIN);
    motor2.writeMicroseconds(THROTTLE_MIN);
    motor3.writeMicroseconds(THROTTLE_MIN);
    motor4.writeMicroseconds(THROTTLE_MIN);
    delay(3000);
    
    Serial.println("2. Ожидание инициализации ESC...");
    delay(2000);
    
    Serial.println("3. Моторы готовы к работе!");
    motorsArmed = true;
    motorsLocked = false;
    Serial.println("=== МОТОРЫ ВЗВЕДЕНЫ ===");
}

// Остановка моторов
void stopAllMotors() {
    Serial.println("Остановка всех моторов...");
    motor1.writeMicroseconds(THROTTLE_MIN);
    motor2.writeMicroseconds(THROTTLE_MIN);
    motor3.writeMicroseconds(THROTTLE_MIN);
    motor4.writeMicroseconds(THROTTLE_MIN);
    PWM1 = PWM2 = PWM3 = PWM4 = THROTTLE_MIN;
    motorsArmed = false;
    motorsLocked = true;
    Serial.println("Все моторы остановлены и заблокированы");
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
    
    if (cmd.equalsIgnoreCase("RESET")) {
        emergencyStop = false;
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
        int t = throttleSet;
        
        int parsed = sscanf(cmd.c_str(), "SET %f %f %f %d", &y, &p, &r, &t);
        if (parsed >= 3) {
            yawDesired = y;
            pitchDesired = p;
            rollDesired = r;
            
            if (parsed == 4) {
                if (t < THROTTLE_MIN) t = THROTTLE_MIN;
                if (t > THROTTLE_MAX) t = THROTTLE_MAX;
                throttleSet = t;
                THROTTLE0 = t;
            }
            
            Serial.printf("SET -> рыскание=%.2f° тангаж=%.2f° крен=%.2f° газ=%d\n", 
                         yawDesired, pitchDesired, rollDesired, throttleSet);
        } else {
            Serial.println("Ошибка разбора SET. Используйте: SET рыскание тангаж крен газ");
        }
        return;
    }

    Serial.println("Неизвестная команда");
}
