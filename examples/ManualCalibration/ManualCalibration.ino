#include <mpu6050_FastAngles.h>

mpu6050_FastAngles mpu;

// Valores de offset conhecidos
int offsetX = -50;
int offsetY = 30;
int offsetZ = 10;

void setup() {
  Serial.begin(9600);

  // Inicializa o sensor
  mpu.begin();

  // Define os offsets de calibração manualmente
  mpu.manualCalibration(offsetX, offsetY, offsetZ);

  // Exibe as configurações atuais
  mpu.printSettings();
}

void loop() {
  // Lê os ângulos nos eixos X, Y e Z
  float angleX = mpu.getAngle('X');
  float angleY = mpu.getAngle('Y');
  float angleZ = mpu.getAngle('Z');

  Serial.print("Ângulo X: "); Serial.print(angleX);
  Serial.print("\tÂngulo Y: "); Serial.print(angleY);
  Serial.print("\tÂngulo Z: "); Serial.println(angleZ);

  delay(100);
}
