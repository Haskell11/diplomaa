#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid     = "ТВОЙ_WIFI";        // имя Wi-Fi
const char* password = "ТВОЙ_ПАРОЛЬ";      // пароль Wi-Fi
const char* host     = "192.168.1.104";    // IP твоего ПК
const int udpPort    = 3333;               // порт для связи

WiFiUDP udp;

void setup() {
  Serial.begin(115200);

  // Подключение к Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(udpPort);
}

void loop() {
  const char* msg = "Hello PC!";
  udp.beginPacket(host, udpPort);
  udp.write((uint8_t*)msg, strlen(msg));
  udp.endPacket();

  Serial.println("Sent: Hello PC!");
  delay(1000);  // раз в секунду
}
