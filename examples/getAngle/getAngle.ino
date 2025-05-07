#include <mpu6050_FastAngles.h>

mpu6050_FastAngles mpu;

void setup() {
  Serial.begin(9600);

  // Inicializa o sensor com uma escala específica do giroscópio
  mpu.begin(MPU_MODE_250); // Opções: MPU_MODE_250, MPU_MODE_500, MPU_MODE_1000, MPU_MODE_2000

  //Opicional, ajustar o fator de correção do filtro complementar
  mpu.setComplementaryFactor(0.98); // Valor entre 0.0 e 1.0, o padrão é de 0.98

  //Opicional, ajustar os parâmetros do filtro de Kalman
  mpu.setKalmanQangle(0.001);
  mpu.setKalmanQbias(0.003);
  mpu.setKalmanRmeasure(0.03);

  //Opicional, exibir as configurações atuais
  mpu.printSettings();

  mpu.calibrateGyro(1000, 2000); // Calibrações e Delay de Inicialização
}

void loop() {
  // Obter os ângulos, o filtro padrão é o complementar
  float angleX = mpu.getAngle('X', KALMAN);
  float angleY = mpu.getAngle('Y', KALMAN);
  float angleZ = mpu.getAngle('Z'); // Não tem filtro de Kalman para o eixo Z

  Serial.print("Angle X: "); Serial.print(angleX);
  Serial.print("\tAngle Y: "); Serial.print(angleY);
  Serial.print("\tAngle Z: "); Serial.println(angleZ);

  delay(100);
}
