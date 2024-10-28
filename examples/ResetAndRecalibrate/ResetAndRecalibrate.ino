#include <mpu6050_FastAngles.h>

mpu6050_FastAngles mpu;
bool recalibrate = false;

void setup() {
  Serial.begin(9600);

  // Inicializa o sensor
  mpu.begin();

  // Exibe as configurações atuais
  mpu.printSettings();
}

void loop() {
  // Se um botão ou condição específica for atendida, resetar e recalibrar
  if (recalibrate) {
    Serial.println("Resetando o sensor e recalibrando...");
    mpu.resetSensor();
    mpu.calibrateGyro();
    mpu.saveCalibration();
    recalibrate = false; // Resetar a condição
  }

  // Lê os ângulos nos eixos X, Y e Z
  float angleX = mpu.getAngle('X');
  float angleY = mpu.getAngle('Y');
  float angleZ = mpu.getAngle('Z');

  Serial.print("Ângulo X: "); Serial.print(angleX);
  Serial.print("\tÂngulo Y: "); Serial.print(angleY);
  Serial.print("\tÂngulo Z: "); Serial.println(angleZ);

  delay(100);
}
