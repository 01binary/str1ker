# Leg Robot Board

A board that controls the mobile platform with four mecanum wheels:

+ Leg Actuators
+ Leg Actuator Potentiometers
+ Leg Wheels
+ Leg Wheel Encoders

## Modules

The following components are placed onto the board as modules:

|Module|Function|
|-|-|
|[Teensy 4.0](https://www.sparkfun.com/teensy-4-0.html)|[ROS](https://www.ros.org/) Node|

## Devices

The following external components are connected to the board via `JST-XH` locking connectors:

|Device|Function|
|-|-|
|8x [IBT2 Motor Drivers](https://www.amazon.com/2-Piece-High-Power-Limiting-Function-Suitable/dp/B0DXKXKYRK)|Wheel Motor Drivers, Leg Actuator Drivers|
|4x [Feedback Rod Linear Actuator](https://www.firgelliauto.com/products/feedback-rod-actuator?variant=2632742851) Potentiometers|Leg Actuator Potentiometers|
|4x [Leg Encoder Board](../LegEncoderBoard/readme.md)|Wheel Encoders|

## Functionality

### PWM

There are four legs and each has two motors: a linear lift actuator and a brushed DC motor that runs the wheel.

[PCA9685PW,118](https://www.digikey.com/en/products/detail/nxp-usa-inc/pca9685pw-118/2034325) is used to run this large number of motors to avoid over-allocating Teensy pins.

### ADC

The linear lift actuators have built-in multi-turn potentiometers. These are routed to Teensy ADC pins.

## Interface

## Pins

| Pin            | Function                       |
| -------------- | ------------------------------ |
| `A0`           | Actuator 1 Potentiometer `SIG` |
| `A1`           | Actuator 2 Potentiometer `SIG` |
| `A2`           | Actuator 3 Potentiometer `SIG` |
| `A3`           | Actuator 4 Potentiometer `SIG` |
| `A5`           | I2C `SCL` (PCA9685)            |
| `A4`           | I2C `SDA` (PCA9685)            |
| `D0`           | Wheel 1 Encoder `A`            |
| `D1`           | Wheel 1 Encoder `B`            |
| `D2`           | Wheel 2 Encoder `A`            |
| `D3`           | Wheel 2 Encoder `B`            |
| `D4`           | Wheel 3 Encoder `A`            |
| `D5`           | Wheel 3 Encoder `B`            |
| `D7`           | Wheel 4 Encoder `A`            |
| `D8`           | Wheel 4 Encoder `B`            |
| `D6`           | Wheel 1 Driver `EN`            |
| `D9`           | Wheel 2 Driver `EN`            |
| `D10`          | Wheel 3 Driver `EN`            |
| `D11`          | Wheel 4 Driver `EN`            |
| `D12`          | Actuator 1 Driver `EN`         |
| `D13`          | Actuator 2 Driver `EN`         |
| `D20`          | Actuator 3 Driver `EN`         |
| `D21`          | Actuator 4 Driver `EN`         |
| `PCA9685 CH0`  | Wheel 1 Driver `LPWM`          |
| `PCA9685 CH1`  | Wheel 1 Driver `RPWM`          |
| `PCA9685 CH2`  | Wheel 2 Driver `LPWM`          |
| `PCA9685 CH3`  | Wheel 2 Driver `RPWM`          |
| `PCA9685 CH4`  | Wheel 3 Driver `LPWM`          |
| `PCA9685 CH5`  | Wheel 3 Driver `RPWM`          |
| `PCA9685 CH6`  | Wheel 4 Driver `LPWM`          |
| `PCA9685 CH7`  | Wheel 4 Driver `RPWM`          |
| `PCA9685 CH8`  | Actuator 1 Driver `LPWM`       |
| `PCA9685 CH9`  | Actuator 1 Driver `RPWM`       |
| `PCA9685 CH10` | Actuator 2 Driver `LPWM`       |
| `PCA9685 CH11` | Actuator 2 Driver `RPWM`       |
| `PCA9685 CH12` | Actuator 3 Driver `LPWM`       |
| `PCA9685 CH13` | Actuator 3 Driver `RPWM`       |
| `PCA9685 CH14` | Actuator 4 Driver `LPWM`       |
| `PCA9685 CH15` | Actuator 4 Driver `RPWM`       |

## Bill of Materials

|Component|Description|
|-|-|
|[PCA9685PW,118](https://www.digikey.com/en/products/detail/nxp-usa-inc/pca9685pw-118/2034325)|PWM Multiplexer|
|[CRCW060310K0FKEA](https://www.digikey.com/en/products/detail/vishay-dale/crcw060310k0fkea/1174782)|`10K` Pull-Down Resistor (PWM channels)|
||`47R` Series Resistor (PWM channels)|
||`2.2K` Series Resistor (ADC channels)|
||`47nF` Capacitor (ADC channels)|
|[JST_XH_B3B-XH-A](https://www.digikey.com/en/products/detail/jst-sales-america-inc/b3b-xh-a/1651046)|Knee Potentiometers (`3V3`, `SIG`, `GND`)|
|[JST_XH_B4B-XH-A](https://www.digikey.com/en/products/detail/jst-sales-america-inc/b4b-xh-a/1651047)|Leg Actuator Drivers, Leg Wheel Drivers (`LPWM`, `RPWM`, `L_EN`, `R_EN`)|
