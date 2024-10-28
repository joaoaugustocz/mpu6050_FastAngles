#include <mpu6050_FastAngles.h>

mpu6050_FastAngles mpu;

void setup() {
  Serial.begin(9600);

  // Inicializa o sensor
  mpu.begin();

  // Verifica se os dados de calibração estão disponíveis na EEPROM
  if (!mpu.loadCalibration()) {
    Serial.println("Dados de calibração não encontrados. Calibrando o giroscópio...");
    mpu.calibrateGyro();
    mpu.saveCalibration();
    Serial.println("Calibração salva na EEPROM.");
  } else {
    Serial.println("Dados de calibração carregados da EEPROM.");
  }

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
