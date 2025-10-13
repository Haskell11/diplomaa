
//нужно заменить библиотеку 
#include <Wire.h>

#define SDA_PIN 21
#define SCL_PIN 22

void setup() {
  Serial.begin(115200);
  delay(200); // небольшая пауза после старта

  // Включаем внутренние подтяжки ПЕРЕД инициализацией Wire
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);

  // Инициализируем I2C с пониженной скоростью
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // 100 kHz — стандартная скорость I2C

  Serial.println("Reading WHO_AM_I register from MPU6050...");

  // Читаем регистр WHO_AM_I (адрес 0x75)
  Wire.beginTransmission(0x68);
  Wire.write(0x75); // адрес регистра WHO_AM_I
  uint8_t error = Wire.endTransmission(false); // false = не отправлять STOP (для restart)

  if (error != 0) {
    Serial.printf("❌ I2C error during write: %d\n", error);
    return;
  }

  // Запрашиваем 1 байт
  Wire.requestFrom(0x68, 1);
  if (Wire.available()) {
    uint8_t whoami = Wire.read();
    Serial.printf("WHO_AM_I = 0x%02X\n", whoami);

    if (whoami == 0x68) {
      Serial.println("✅ MPU6050 detected correctly!");
    } else {
      Serial.println("❌ Unexpected WHO_AM_I value. Possible I2C instability or wrong device.");
    }
  } else {
    Serial.println("❌ No data received from MPU6050.");
  }
}

void loop() {
  // Ничего не делаем
}