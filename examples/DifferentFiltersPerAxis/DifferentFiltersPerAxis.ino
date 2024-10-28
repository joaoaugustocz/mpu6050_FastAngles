#include <mpu6050_FastAngles.h>

mpu6050_FastAngles mpu;

void setup() {
  Serial.begin(9600);

  // Inicializa o sensor
  mpu.begin();

  // Exibe as configurações atuais
  mpu.printSettings();
}

void loop() {
  // Lê o ângulo no eixo X usando o filtro de Kalman
  float angleX = mpu.getAngle('X', KALMAN);

  // Lê o ângulo no eixo Y usando o filtro complementar
  float angleY = mpu.getAngle('Y', COMPLEMENTARY);

  // Lê o ângulo no eixo Z (apenas integração do giroscópio)
  float angleZ = mpu.getAngle('Z');

  Serial.print("Ângulo X (Kalman): "); Serial.print(angleX);
  Serial.print("\tÂngulo Y (Complementar): "); Serial.print(angleY);
  Serial.print("\tÂngulo Z: "); Serial.println(angleZ);

  delay(100);
}
