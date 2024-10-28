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
  // Exemplo de ajuste dinâmico (simulando uma condição)
  if (someCondition()) {
    // Ajusta os parâmetros do filtro de Kalman para maior responsividade
    mpu.setKalmanQangle(0.005);
    mpu.setKalmanQbias(0.005);
    mpu.setKalmanRmeasure(0.02);
    Serial.println("Parâmetros do filtro de Kalman ajustados para maior responsividade.");
  } else {
    // Ajusta os parâmetros para maior estabilidade
    mpu.setKalmanQangle(0.001);
    mpu.setKalmanQbias(0.003);
    mpu.setKalmanRmeasure(0.03);
    Serial.println("Parâmetros do filtro de Kalman ajustados para maior estabilidade.");
  }

  // Lê os ângulos nos eixos X e Y usando o filtro de Kalman
  float angleX = mpu.getAngle('X', KALMAN);
  float angleY = mpu.getAngle('Y', KALMAN);

  Serial.print("Ângulo X: "); Serial.print(angleX);
  Serial.print("\tÂngulo Y: "); Serial.println(angleY);

  delay(100);
}

bool someCondition() {
  // Função fictícia para simular uma condição
  // Pode ser substituída por uma leitura de botão, sensor, etc.
  return millis() % 5000 < 2500; // Alterna a cada 2,5 segundos
}
