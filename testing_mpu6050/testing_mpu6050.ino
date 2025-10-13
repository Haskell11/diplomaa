#include <Wire.h>
#include <MPU6050_tockn.h>

MPU6050 mpu(Wire);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println("Инициализация MPU6050...");
  mpu.begin();

  // Подождём немного для стабилизации
  delay(1000);
  mpu.calcGyroOffsets(true); // Калибровка гироскопа

  Serial.println("MPU6050 готов!");
}

void loop() {
  mpu.update();

  Serial.print("AccX: "); Serial.print(mpu.getAccX());
  Serial.print(" | AccY: "); Serial.print(mpu.getAccY());
  Serial.print(" | AccZ: "); Serial.print(mpu.getAccZ());
  Serial.print(" | GyX: "); Serial.print(mpu.getGyroX());
  Serial.print(" | GyY: "); Serial.print(mpu.getGyroY());
  Serial.print(" | GyZ: "); Serial.print(mpu.getGyroZ());
  Serial.print(" | AngleX: "); Serial.print(mpu.getAngleX());
  Serial.print(" | AngleY: "); Serial.print(mpu.getAngleY());
  Serial.print(" | AngleZ: "); Serial.println(mpu.getAngleZ());

  delay(500);
}
