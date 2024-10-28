#include <mpu6050_FastAngles.h>

mpu6050_FastAngles mpu;

void setup() {
  Serial.begin(9600);

  // Inicializa o sensor com a escala ±1000°/s
  mpu.begin(MPU_MODE_1000);

  // Ajusta o fator de correção do filtro complementar
  mpu.setComplementaryFactor(0.90); // Valor entre 0.0 e 1.0

  // Exibe as configurações atuais
  mpu.printSettings();
}

void loop() {
  // Lê os ângulos nos eixos X, Y e Z usando o filtro complementar
  float angleX = mpu.getAngle('X', COMPLEMENTARY);
  float angleY = mpu.getAngle('Y', COMPLEMENTARY);
  float angleZ = mpu.getAngle('Z');

  Serial.print("Ângulo X: "); Serial.print(angleX);
  Serial.print("\tÂngulo Y: "); Serial.print(angleY);
  Serial.print("\tÂngulo Z: "); Serial.println(angleZ);

  delay(100);
}
