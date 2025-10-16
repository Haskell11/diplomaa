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
bool active = false;

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

  if (active) {
    mpu.update();

    
    String msg = "AX:" + String(mpu.getAccX(), 2) + //линейные ускорения вдоль осей 
                 " AY:" + String(mpu.getAccY(), 2) +
                 " AZ:" + String(mpu.getAccZ(), 2) +
                 " GX:" + String(mpu.getGyroX(), 2) + // угловые скорости(насколько быстро вращается MPU вокруг осей)
                 " GY:" + String(mpu.getGyroY(), 2) +
                 " GZ:" + String(mpu.getGyroZ(), 2) +
                 " AngleX:" + String(mpu.getAngleX(), 2) + // вычисленные углы наклона
                 " AngleY:" + String(mpu.getAngleY(), 2) +
                 " AngleZ:" + String(mpu.getAngleZ(), 2) +
                 

    udp.beginPacket(pcIP, pcPort);
    udp.write((const uint8_t*)msg.c_str(), msg.length());
    udp.endPacket();

    Serial.println(msg);
    delay(500);
  }
}
