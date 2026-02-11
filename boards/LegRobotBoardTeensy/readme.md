# Leg Robot Board

A board that controls robot legs:

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

## Pins

| Pin   | Function                                          |
| ----- | ------------------------------------------------- |
| `A0`  | Actuator 1 Potentiometer `SIG`                        |
| `A1`  | Actuator 2 Potentiometer `SIG`                        |
| `A2`  | Actuator 3 Potentiometer `SIG`                        |
| `A3`  | Actuator 4 Potentiometer `SIG`                        |
| `D0`  | Wheel 1 Encoder `A`                               |
| `D1`  | Wheel 1 Encoder `B`                               |
| `D2`  | Wheel 2 Encoder `A`                               |
| `D3`  | Wheel 2 Encoder `B`                               |
| `D4`  | Wheel 3 Encoder `A`                               |
| `D5`  | Wheel 3 Encoder `B`                               |
| `D7`  | Wheel 4 Encoder `A`                               |
| `D8`  | Wheel 4 Encoder `B`                               |
| `D9`  | Wheel 1 Driver `LPWM`                             |
| `D10` | Wheel 1 Driver `RPWM`                             |
| `D11` | Wheel 2 Driver `LPWM`                             |
| `D12` | Wheel 2 Driver `RPWM`                             |
| `D20` | Wheel 3 Driver `LPWM`                             |
| `D21` | Wheel 3 Driver `RPWM`                             |
| `D22` | Wheel 4 Driver `LPWM`                             |
| `D23` | Wheel 4 Driver `RPWM`                             |
| `D6`  | Leg Actuator Drivers `LPWM` (fan-out to all 4)    |
| `D13` | Leg Actuator Drivers `RPWM` (fan-out to all 4)    |
| `D18` | Wheels `EN` (shared to all wheel drivers)         |
| `D19` | Actuators `EN` (shared to all actuator drivers)   |

## Bill of Materials

|Component|Description|
|-|-|
|[CRCW060310K0FKEA](https://www.digikey.com/en/products/detail/vishay-dale/crcw060310k0fkea/1174782)|`10K` Pull-Down Resistor (PWM channels)|
||`47R` Series Resistor (PWM channels)|
||`2.2K` Series Resistor (ADC channels)|
||`47nF` Capacitor (ADC channels)|
|[JST_XH_B3B-XH-A](https://www.digikey.com/en/products/detail/jst-sales-america-inc/b3b-xh-a/1651046)|Knee Potentiometers (`3V3`, `SIG`, `GND`)|
|[JST_XH_B4B-XH-A](https://www.digikey.com/en/products/detail/jst-sales-america-inc/b4b-xh-a/1651047)|Leg Actuator Drivers, Leg Wheel Drivers (`LPWM`, `RPWM`, `L_EN`, `R_EN`)|
|
