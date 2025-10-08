#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid     = "MTS_GPON_2A89"; // Ваш Wi-Fi
const char* password = "3g6y2ZaiLu";

WiFiUDP udp;
const int localPort = 3333;  // Порт для приёма
const char* pcIP = "192.168.1.104";  // IP ПК
const int pcPort = 3333;  // Порт ПК

void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(localPort);
  Serial.println("UDP server started");
}

void loop() {
  // --- Отправка данных на ПК ---
  const char* msg = "Hello PC!"; // здесь можно отправлять углы с MPU
  udp.beginPacket(pcIP, pcPort);
  udp.write((uint8_t*)msg, strlen(msg));
  udp.endPacket();
  Serial.println("Packet sent successfully!");

  // --- Приём команд от ПК ---
  char buffer[255];
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(buffer, 255);
    if (len > 0) buffer[len] = 0;
    Serial.print("Received from PC: ");
    Serial.println(buffer);

    // ответ обратно ПК
     const char* response = "Command received!";
  udp.beginPacket(udp.remoteIP(), udp.remotePort());
  udp.write((uint8_t*)response, strlen(response)); //
  udp.endPacket();
  }

  delay(10000); // раз в секунду
}
