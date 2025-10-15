#include <WiFi.h>
#include <WiFiUdp.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

const char* ssid = "MTS_GPON_2A89";
const char* password = "3g6y2ZaiLu";

WiFiUDP udp;
const int localPort = 3333;
const char* pcIP = "192.168.1.104";
const int pcPort = 3333;

MPU6050 mpu(Wire);
bool active = false; // передача выключена по умолчанию

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
  // приём команд от Python
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char buf[255];
    int len = udp.read(buf, 255);
    if (len > 0) buf[len] = 0;

    String cmd = String(buf);
    cmd.trim();
    Serial.print("Received command: ");
    Serial.println(cmd);

    if (cmd == "START") {
      active = true;
      Serial.println("✅ Data stream started");
    } else if (cmd == "STOP") {
      active = false;
      Serial.println("🛑 Data stream stopped");
    }
  }

  // если активен режим передачи — шлём данные
  if (active) {
    mpu.update();
    String msg = "X:" + String(mpu.getAngleX()) +
                 " Y:" + String(mpu.getAngleY()) +
                 " Z:" + String(mpu.getAngleZ());

    udp.beginPacket(pcIP, pcPort);
    udp.write((const uint8_t*)msg.c_str(), msg.length());
    udp.endPacket();

    Serial.println("Sent: " + msg);
    delay(500);
  }
}
