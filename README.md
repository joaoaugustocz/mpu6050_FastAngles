# mpu6050_FastAngles Library Documentation

## Overview

mpu6050_FastAngles is an Arduino library designed to read fast and accurate angles from the MPU6050 sensor using complementary and Kalman filters. This library simplifies the integration and control of the MPU6050 sensor in Arduino projects, offering flexible configuration options and easy data acquisition.

## Installation

To use the mpu6050_FastAngles library in your Arduino projects:

1. Download the mpu6050_FastAngles library from the [GitHub repository](https://github.com/joaoaugustocz/mpu6050_FastAngles) as a ZIP file.
2. Install the library in the Arduino IDE by navigating to **Sketch** -> **Include Library** -> **Add .ZIP Library...** and selecting the downloaded ZIP file.
3. Open the mpu6050_FastAngles example sketches from **File** -> **Examples** -> **mpu6050_FastAngles** to get started with using the library.

## Class mpu6050_FastAngles

### Constructor

```cpp
mpu6050_FastAngles();
```

### Methods
`void begin(GyroScale scale = MPU_MODE_250);`
Initializes the sensor and configurations with the selected gyroscope scale.

`void calibrateGyro(int calibrations = 1000);`
Calibrates the gyroscope with the specified number of samples.

`void saveCalibration();`
Saves the calibration offsets to EEPROM.

`bool loadCalibration();`
Loads the calibration offsets from EEPROM.

`void manualCalibration(int x, int y, int z);`  
  Sets the gyroscope calibration offsets manually for the X, Y, and Z axes. This can be useful if you already know the offsets and want to avoid recalibrating each time.

  **Parameters:**
    - `x`: Offset for the X-axis
    - `y`: Offset for the Y-axis
    - `z`: Offset for the Z-axis

`void resetSensor();`
Resets the MPU6050 sensor.

`void zeroAngles();`
Resets the accumulated angles to zero.

`void invert(char axis);`
Inverts the specified axis ('X', 'Y', or 'Z').

`float getAngle(char axis, FilterType filter = COMPLEMENTARY);`
Returns the angle of the specified axis using the selected filter.

`void setComplementaryFactor(float factor);`
Sets the correction factor for the complementary filter (0.0 to 1.0).

`void setKalmanQangle(float Q_angle);`
Sets the process noise variance for the angle in the Kalman filter.

`void setKalmanQbias(float Q_bias);`
Sets the process noise variance for the bias in the Kalman filter.

`void setKalmanRmeasure(float R_measure);`
Sets the measurement noise variance in the Kalman filter.

`void printSettings();`
Prints the current configurations and calibration offsets.

Enumerations
`enum FilterType { COMPLEMENTARY, KALMAN };`
Defines the types of filters available.

`enum GyroScale { MPU_MODE_250, MPU_MODE_500, MPU_MODE_1000, MPU_MODE_2000 };`
Defines the gyroscope scale modes.


## Example Usage

```cpp
#include <mpu6050_FastAngles.h>

mpu6050_FastAngles mpu;

void setup() {
  Serial.begin(9600);

  // Initialize the sensor with a specific gyroscope scale
  mpu.begin(MPU_MODE_250); // Options: MPU_MODE_250, MPU_MODE_500, MPU_MODE_1000, MPU_MODE_2000

  // Optional, adjust the correction factor of the complementary filter
  mpu.setComplementaryFactor(0.95); // Value between 0.0 and 1.0, standard is 0.98

  // Optional, adjust the parameters of the Kalman filter
  mpu.setKalmanQangle(0.001);
  mpu.setKalmanQbias(0.003);
  mpu.setKalmanRmeasure(0.03);

  // Optional, display the current settings
  mpu.printSettings();
}

void loop() {
  // Get the angles, standard filter is COMPLEMENTARY
  float angleX = mpu.getAngle('X');
  float angleY = mpu.getAngle('Y');
  float angleZ = mpu.getAngle('Z', KALMAN);

  Serial.print("Angle X: "); Serial.print(angleX);
  Serial.print("\tAngle Y: "); Serial.print(angleY);
  Serial.print("\tAngle Z: "); Serial.println(angleZ);

  delay(100);
}
```

## Differences Between Gyroscope Scales
The MPU6050 gyroscope offers four selectable full-scale ranges:

- `MPU_MODE_250`: ±250°/s (highest sensitivity, best precision for slow rotations)
- `MPU_MODE_500`: ±500°/s
- `MPU_MODE_1000`: ±1000°/s
- `MPU_MODE_2000`: ±2000°/s (lowest sensitivity, suitable for fast rotations)
Selecting a lower scale (e.g., ±250°/s) provides higher sensitivity and better precision for slow movements but limits the maximum measurable angular velocity. Higher scales allow for measuring faster rotations but with reduced precision.

## License
This library is released under the MIT License. See the LICENSE file for more details.



# Documentação da Biblioteca mpu6050_FastAngles
## Visão Geral
mpu6050_FastAngles é uma biblioteca para Arduino projetada para ler ângulos de forma rápida e precisa do sensor MPU6050, utilizando filtros complementar e de Kalman. Esta biblioteca simplifica a integração e controle do sensor MPU6050 em projetos Arduino, oferecendo opções de configuração flexíveis e aquisição de dados simplificada.

## Instalação
Para utilizar a biblioteca mpu6050_FastAngles nos seus projetos Arduino:

Baixe a biblioteca mpu6050_FastAngles do repositório GitHub como um arquivo ZIP.
Instale a biblioteca no Arduino IDE navegando em Sketch -> Incluir Biblioteca -> Adicionar Biblioteca .ZIP... e selecione o arquivo ZIP baixado.
Abra os exemplos da mpu6050_FastAngles em Arquivo -> Exemplos -> mpu6050_FastAngles para começar a utilizar a biblioteca.
## Classe mpu6050_FastAngles
### Construtor
`mpu6050_FastAngles();`

### Métodos

`void begin(GyroScale scale = MPU_MODE_250);`
Inicializa o sensor e as configurações com a escala do giroscópio selecionada.

`void calibrateGyro(int calibrations = 1000);`
Calibra o giroscópio com o número de amostras especificado.

`void saveCalibration();`
Salva os offsets de calibração na EEPROM.

`bool loadCalibration();`
Carrega os offsets de calibração da EEPROM.

`void manualCalibration(int x, int y, int z);`  
Define manualmente os offsets de calibração do giroscópio para os eixos X, Y e Z. Isso pode ser útil se você já conhece os offsets e deseja evitar recalibrar a cada vez.

**Parâmetros:**

-`x`: Offset para o eixo X
-`y`: Offset para o eixo Y
-`z`: Offset para o eixo Z

`void resetSensor();`
Reinicia o sensor MPU6050.

`void zeroAngles();`
Reseta os ângulos acumulados para zero.

`void invert(char axis);`
Inverte o eixo especificado ('X', 'Y' ou 'Z').

`float getAngle(char axis, FilterType filter = COMPLEMENTARY);`
Retorna o ângulo do eixo especificado usando o filtro selecionado.

`void setComplementaryFactor(float factor);`
Define o fator de correção do filtro complementar (0.0 a 1.0).

`void setKalmanQangle(float Q_angle);`
Define a variância do ruído do processo para o ângulo no filtro de Kalman.

`void setKalmanQbias(float Q_bias);`
Define a variância do ruído do processo para o bias no filtro de Kalman.

`void setKalmanRmeasure(float R_measure);`
Define a variância do ruído da medição no filtro de Kalman.

`void printSettings();`
Imprime as configurações atuais e os offsets de calibração.

Enumerações
`enum FilterType { COMPLEMENTARY, KALMAN };`
Define os tipos de filtros disponíveis.

`enum GyroScale { MPU_MODE_250, MPU_MODE_500, MPU_MODE_1000, MPU_MODE_2000 };`
Define os modos de escala do giroscópio.

## Exemplo de Uso

``` cpp
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
}

void loop() {
  // Obter os ângulos, o filtro padrão é o complementar
  float angleX = mpu.getAngle('X', KALMAN);
  float angleY = mpu.getAngle('Y', KALMAN);
  float angleZ = mpu.getAngle('Z');

  Serial.print("Angle X: "); Serial.print(angleX);
  Serial.print("\tAngle Y: "); Serial.print(angleY);
  Serial.print("\tAngle Z: "); Serial.println(angleZ);

  delay(100);
}

```

## Diferenças Entre as Escalas do Giroscópio
O giroscópio do MPU6050 oferece quatro intervalos de escala completa selecionáveis:

- `MPU_MODE_250`: ±250°/s (maior sensibilidade, melhor precisão para rotações lentas)
- `MPU_MODE_500`: ±500°/s
- `MPU_MODE_1000`: ±1000°/s
- `MPU_MODE_2000`: ±2000°/s (menor sensibilidade, adequado para rotações rápidas)
Selecionar uma escala menor (por exemplo, ±250°/s) fornece maior sensibilidade e melhor precisão para movimentos lentos, mas limita a velocidade angular máxima mensurável. Escalas maiores permitem medir rotações mais rápidas, mas com precisão reduzida.

## Licença
Esta biblioteca é lançada sob a Licença MIT. Consulte o arquivo LICENSE para mais detalhes.
