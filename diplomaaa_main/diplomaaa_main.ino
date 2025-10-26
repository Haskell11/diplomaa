#include <WiFi.h>
#include <WiFiUdp.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// ======== Wi-Fi настройки ========
const char* ssid = "MTS_GPON_2A89";
const char* password = "3g6y2ZaiLu";

// ======== UDP настройки ========
WiFiUDP udp;
const char* pcIP = "192.168.1.104";
const char* pcIP2 = "192.168.1.38";
const int pcPort = 3333;
const int localPort = 3333;

// ======== MPU6050 ========
MPU6050 mpu(0x68);  // Используем адрес 0x70
bool dmpReady = false;
uint8_t mpuIntStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

bool active = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Подключаемся к Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi connected!");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(localPort);
  Serial.println("📡 UDP ready");

  // Инициализация MPU6050
  Serial.println("🔧 Initializing MPU6050...");
  mpu.initialize();

  // Проверка соединения
  if (!mpu.testConnection()) {
    Serial.println("❌ MPU6050 connection failed!");
    while (true);
  }
  Serial.println("✅ MPU6050 connected");

  // Инициализация DMP
  uint8_t devStatus = mpu.dmpInitialize();

  // Задаём смещения (при необходимости калибровки — можно поменять)
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println("✅ DMP ready!");
  } else {
    Serial.print("❌ DMP init failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
  }
}

void loop() {
  // Приём команд START / STOP
  int packetSizeUdp = udp.parsePacket();
  if (packetSizeUdp) {
    char buf[255];
    int len = udp.read(buf, 255);
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

  if (!dmpReady || !active) return;

  // Читаем данные из FIFO
  fifoCount = mpu.getFIFOCount();
  if (fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println("⚠️ FIFO overflow!");
    return;
  }

  if (fifoCount < packetSize) return;

  mpu.getFIFOBytes(fifoBuffer, packetSize);
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  // Преобразуем радианы в градусы
  float yaw = ypr[0] * 180.0 / M_PI;
  float pitch = ypr[1] * 180.0 / M_PI;
  float roll = ypr[2] * 180.0 / M_PI;

  // Формируем строку
  String msg = "Yaw:" + String(yaw, 2) + " Pitch:" + String(pitch, 2) + " Roll:" + String(roll, 2);
  Serial.println(msg);

  // Отправляем по Wi-Fi
  udp.beginPacket(pcIP, pcPort);
  udp.write((const uint8_t*)msg.c_str(), msg.length());
  udp.endPacket();
   udp.beginPacket(pcIP2, pcPort);
  udp.write((const uint8_t*)msg.c_str(), msg.length());
  udp.endPacket();

  delay(500);  // частота отправки ~10 Гц
}
