#include <mpu6050_FastAngles.h>

mpu6050_FastAngles mpu;

void setup() {
  Serial.begin(9600);

  // Inicializa o sensor
  mpu.begin();

  // Inverte os eixos X e Y
  mpu.invert('X');
  mpu.invert('Y');

  // Zera os ângulos acumulados
  mpu.zeroAngles();

  // Exibe as configurações atuais
  mpu.printSettings();
}

void loop() {
  // Lê os ângulos nos eixos X, Y e Z
  float angleX = mpu.getAngle('X');
  float angleY = mpu.getAngle('Y');
  float angleZ = mpu.getAngle('Z');

  Serial.print("Ângulo X (invertido): "); Serial.print(angleX);
  Serial.print("\tÂngulo Y (invertido): "); Serial.print(angleY);
  Serial.print("\tÂngulo Z: "); Serial.println(angleZ);

  delay(100);
}
