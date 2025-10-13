#include <WiFi.h>
#include <WiFiUdp.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

const char* ssid = "MTS_GPON_2A89";
const char* password = "3g6y2ZaiLu";

WiFiUDP udp;
const int localPort = 3333;
const char* pcIP = "192.168.1.104";  // IP твоего ПК
const int pcPort = 3333;

MPU6050 mpu(Wire);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  WiFi.begin(ssid, password);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(localPort);
  Serial.println("UDP ready");

  mpu.begin();
  mpu.calcGyroOffsets(true);
}

void loop() {
  mpu.update();

  String msg = "X:" + String(mpu.getAngleX()) +
               " Y:" + String(mpu.getAngleY()) +
               " Z:" + String(mpu.getAngleZ());

  int result = udp.beginPacket(pcIP, pcPort);
  if (result == 1) {
    udp.write((const uint8_t*)msg.c_str(), msg.length());
    udp.endPacket();
    Serial.print("Sent to ");
    Serial.print(pcIP);
    Serial.print(": ");
    Serial.println(msg);
  } else {
    Serial.println("❌ beginPacket failed");
  }

  // приём от Python
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char buf[255];
    int len = udp.read(buf, 255);
    if (len > 0) buf[len] = 0;
    Serial.print("Received from ");
    Serial.print(udp.remoteIP());
    Serial.print(": ");
    Serial.println(buf);
  }

  delay(1000);
}
