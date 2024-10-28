#include <mpu6050_FastAngles.h>

mpu6050_FastAngles mpu;

void setup() {
  Serial.begin(9600);

  // Inicializa o sensor com a escala padrão (MPU_MODE_250)
  mpu.begin();

  // Exibe as configurações atuais
  mpu.printSettings();
}

void loop() {
  // Lê os ângulos nos eixos X, Y e Z usando o filtro complementar
  float angleX = mpu.getAngle('X');
  float angleY = mpu.getAngle('Y');
  float angleZ = mpu.getAngle('Z');

  Serial.print("Ângulo X: "); Serial.print(angleX);
  Serial.print("\tÂngulo Y: "); Serial.print(angleY);
  Serial.print("\tÂngulo Z: "); Serial.println(angleZ);

  delay(100);
}
