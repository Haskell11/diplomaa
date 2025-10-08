//основной код - пока еще проверяется на других скетчах(например, diploma1)
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include "MPU6050.h"

const char* ssid     = "MTS_GPON5_2A89";     // <-- Введи сюда Wi-Fi
const char* password = "3g6y2ZaiLu";   // <-- Введи сюда пароль

const char* host = "192.168.1.104";  // <-- IP твоего компьютера
const int udpPort = 3333;

WiFiUDP udp;
MPU6050 mpu;

void setup() {
  Serial.begin(115200);

  // Wi-Fi подключение
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  // UDP
  udp.begin(udpPort);

  // MPU6050
  Wire.begin(21, 22);  // SDA, SCL
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
  } else {
    Serial.println("MPU6050 ready!");
  }
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  char packet[128];
  snprintf(packet, sizeof(packet), "%d,%d,%d,%d,%d,%d",
           ax, ay, az, gx, gy, gz);

  udp.beginPacket(host, udpPort);
  udp.write((uint8_t*)packet, strlen(packet));
  udp.endPacket();

  delay(10); // ~100 Hz
}
