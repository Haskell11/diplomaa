#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <helper_3dmath.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ======== Wi-Fi настройки ========
const char* ssid = "MTS_GPON_2A89";
const char* password = "3g6y2ZaiLu";

// ======== UDP настройки ========
WiFiUDP udp;
const char* pcIP = "192.168.1.104";  // IP твоего ПК (приём данных)
const int pcPort = 3333;
const int localPort = 3333;

// ======== MPU6050 ========
MPU6050 mpu(0x68);
bool dmpReady = false;
uint16_t packetSize;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

// ======== Управление ========
bool active = false; // включается при команде START
unsigned long lastSend = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(400000);

  Serial.println(F("🔧 Инициализация MPU6050..."));
  mpu.initialize();

  Wire.beginTransmission(0x68);
  if (Wire.endTransmission() != 0) {
    Serial.println(F("❌ MPU6050 не отвечает!"));
    while (1);
  } else {
    Serial.println(F("✅ MPU6050 найден."));
  }

  // Инициализация DMP
  uint8_t devStatus = mpu.dmpInitialize();

  // Калибровка
  Serial.println(F("⚙️ Калибровка... Не двигай датчик!"));
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.PrintActiveOffsets();

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
    Serial.println(F("✅ DMP запущен!"));
  } else {
    Serial.print(F("❌ Ошибка DMP (код ")); Serial.print(devStatus); Serial.println(F(")"));
    while (1);
  }

  // Подключение Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Подключение к WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi подключен!");
  Serial.print("ESP32 IP: "); Serial.println(WiFi.localIP());
  udp.begin(localPort);
  Serial.println("📡 UDP готов");

  Serial.println("\n--- ГОТОВО ---");
}

void loop() {
  // ======== Приём команд START / STOP ========
  int packetSizeUdp = udp.parsePacket();
  if (packetSizeUdp) {
    char buf[50];
    int len = udp.read(buf, sizeof(buf) - 1);
    if (len > 0) buf[len] = 0;
    String cmd = String(buf);
    cmd.trim();

    if (cmd == "START") {
      active = true;
      Serial.println("▶️ START streaming");
    } else if (cmd == "STOP") {
      active = false;
      Serial.println("⏹ STOP streaming");
    }
  }

  // ======== Считывание с DMP ========
  if (!dmpReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Радианы → градусы
    float yaw = ypr[0] * 180.0 / M_PI;
    float pitch = ypr[1] * 180.0 / M_PI;
    float roll = ypr[2] * 180.0 / M_PI;

    Serial.print("Yaw:"); Serial.print(yaw, 2);
    Serial.print(",Pitch:"); Serial.print(pitch, 2);
    Serial.print(",Roll:"); Serial.println(roll, 2);

    // === Отправка по UDP (если активировано) ===
    if (active && millis() - lastSend >= 50) { // ~20 Гц
      char msg[64];
      snprintf(msg, sizeof(msg), "%.2f,%.2f,%.2f", yaw, pitch, roll);
      udp.beginPacket(pcIP, pcPort);
      udp.print(msg);
      udp.endPacket();
      lastSend = millis();
    }
  }
  delay(100);
}
