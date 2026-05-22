<a href="https://opencyphal.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/opencyphal.svg" width="25%"></a>
RobotemRovne_Cyphal
==================
Code for the Robot for the Robotem Rovne Contest based on the Cyphal protocol

# Cyphal Settings

## Node IDs

| **Board**                                                                                    | **Node-ID** |
|:--------------------------------------------------------------------------------------------:|:-----------:|
| [CyphalRobotController07/CAN](https://github.com/generationmake/CyphalRobotController07-CAN) | 21          |
| [CyphalPicoBase/CAN](https://github.com/generationmake/CyphalPicoBase-CAN) as HMI            | 22          |
| [CyphalPicoBase/CAN](https://github.com/generationmake/CyphalPicoBase-CAN) as IMU and GNSS   | 23          |

## Subject IDs

| **Function**              | **Subject-ID** |
|:-------------------------:|:--------------:|
| Emergency Stop            | 100            |
| Motor 0 PWM               | 200            |
| Motor 1 PWM               | 201            |
| IMU Orientation X         | 300            |
| IMU Calibration           | 301            |
| GNSS Coordinates          | 302            |

# Pin Usage and connectors

## Raspberry Pi Pico

| **Pin** | **Pin Name** | **Signal**    | **Description**                  |
|:-------:|:------------:|:-------------:|:--------------------------------:|
| 1       | GP0          | UART0_TX      | reserved                         |
| 2       | GP1          | UART0_RX      | reserved                         |
| 3       | GND          | GND           |                                  |
| 4       | GP2          |               |                                  |
| 5       | GP3          |               |                                  |
| 6       | GP4          | I2C0_SDA      | for eeprom and BNO055            |
| 7       | GP5          | I2C0_SCL      | for eeprom and BNO055            |
| 8       | GND          | GND           |                                  |
| 9       | GP6          | TFT_CS        | display CS                       |
| 10      | GP7          | TFT_DC        | display DC                       |
| 11      | GP8          | reserved      |                                  |
| 12      | GP9          | TFT_RST       | display RST                      |
| 13      | GND          | GND           |                                  |
| 14      | GP10         | SPI1_CLK      | SPI for display                  |
| 15      | GP11         | SPI1_MOSI     | SPI for display                  |
| 16      | GP12         | SPI1_MISO     | SPI for display                  |
| 17      | GP13         | ROT_SW        | rotary encoder switch            |
| 18      | GND          | GND           |                                  |
| 19      | GP14         | SERVO0        | servo 0                          |
| 20      | GP15         | SERVO1        | servo 1                          |
| 21      | GP16         | SPI_MISO      | SPI for MCP2515                  |
| 22      | GP17         | MCP2515_CS    | SPI for MCP2515                  |
| 23      | GND          | GND           |                                  |
| 24      | GP18         | SPI_CLK       | SPI for MCP2515                  |
| 25      | GP19         | SPI_MOSI      | SPI for MCP2515                  |
| 26      | GP20         | MCP2515_INT   | interrupt for MCP2515            |
| 27      | GP21         | STATUS_LED2   | internal status LED 2            |
| 28      | GND          | GND           |                                  |
| 29      | GP22         | STATUS_LED3   | internal status LED 3            |
| 30      | RUN          | RESET         | Reset for Board                  |
| 31      | GP26         | INPUT_VOLTAGE | measure input voltage            |
| 32      | GP27         | ROT_A         | rotary encoder A                 |
| 33      | GND          | GND           |                                  |
| 34      | GP28         | ROT_B         | rotary encoder B                 |
| 35      | ADC_VREF     |               |                                  |
| 36      | 3V3 (OUT)    | 3V3-rail      | supply voltage for board         |
| 37      | 3V3_EN       |               |                                  |
| 38      | GND          | GND           |                                  |
| 39      | VSYS         |               |                                  |
| 40      | VBUS         | 5V-rail       | supply voltage for board         |
