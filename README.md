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
