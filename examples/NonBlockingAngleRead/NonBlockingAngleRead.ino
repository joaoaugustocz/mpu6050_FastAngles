#include <mpu6050_FastAngles.h>

mpu6050_FastAngles mpu;

unsigned long lastReadTime = 0;
const unsigned long readInterval = 50; // Intervalo de leitura em milissegundos

void setup() {
  Serial.begin(9600);

  // Inicializa o sensor
  mpu.begin();

  // Exibe as configurações atuais
  mpu.printSettings();
}

void loop() {
  // Verifica se é hora de ler os ângulos
  if (millis() - lastReadTime >= readInterval) {
    lastReadTime = millis();

    // Lê os ângulos nos eixos X, Y e Z
    float angleX = mpu.getAngle('X');
    float angleY = mpu.getAngle('Y');
    float angleZ = mpu.getAngle('Z');

    Serial.print("Ângulo X: "); Serial.print(angleX);
    Serial.print("\tÂngulo Y: "); Serial.print(angleY);
    Serial.print("\tÂngulo Z: "); Serial.println(angleZ);
  }

  // Outras tarefas podem ser executadas aqui sem serem bloqueadas
  doOtherTasks();
}

void doOtherTasks() {
  // Função que representa outras tarefas no loop principal
  // Pode incluir leituras de sensores, controle de atuadores, etc.
}
