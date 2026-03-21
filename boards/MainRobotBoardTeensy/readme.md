# Main Robot Board

A board that controls robot head and body.

+ Head Pan/Tilt
+ Torso Pan/Tilt
+ Mouth Movement
+ Voltage and Current Sensing

## Modules

The following components are placed onto the board as modules:

|Module|Function|
|-|-|
|[Teensy 4.0](https://www.sparkfun.com/teensy-4-0.html)|[ROS](https://www.ros.org/) Node|
|[INA260 Voltage Sensor](https://learn.adafruit.com/adafruit-ina260-current-voltage-power-sensor-breakout)|Bus Voltage Sensing|
|[ACS37220 Current Sensor](https://www.pololu.com/product/5295)|Bus Current Sensing|

## Devices

The following external components are connected to the board via JST-XH locking connectors:

|Device|Function|
|-|-|
|[TB6600HG Stepper Motor Driver](https://www.amazon.com/dp/B01N6AIEQT)|Head Pan Motor|
|4x [Mini IBT Motor Driver](https://www.aliexpress.us/item/2251832603816751.html)|2x Head Tilt Motor Driver, 2x Torso Tilt Motor Driver|
|[Lamprey 2 Absolute Encoder](https://andymark.com/products/lamprey2-absolute-encoder)|Head Pan Encoder|
|[Lamprey 2 Absolute Encoder 4 inch](https://andymark.com/products/lamprey2-4-inch-absolute-encoder)|Torso Pan Encoder|
|3x Potentiometers|1x Head Tilt Encoder, 2x Torso Tilt Encoders|
|[PerfectPass 56Kg Servo](https://www.amazon.com/dp/B09Y4NZJBJ)|Mouth Movement|
|[CPM-MCVC-3441S-RLN](https://teknic.com/model-info/CPM-MCVC-3441S-RLN/?model_voltage=75VDC)|Torso Pan Motor Driver

## Buses

| Bus   | Devices                       |
| ----- | ----------------------------- |
| `I2C` | INA260 voltage/current sensor |
| `SPI` | Lamprey encoders              |

## Pins

| Pin   | Function                                  |
| ----- | ----------------------------------------- |
| `D19` | I2C `SCL` (INA260)                        |
| `D18` | I2C `SDA` (INA260)                        |
| `A0`  | `ACS37220` Current Sensor `VOUT`          |
| `D20` | `CPM-MCVC-3441S-RLN` Torso Motor `A`      |
| `D21` | `CPM-MCVC-3441S-RLN` Torso Motor `B`      |
| `D7`  | `CPM-MCVC-3441S-RLN` Torso Motor `Enable` |
| `D8`  | `CPM-MCVC-3441S-RLN` Torso Motor `Status` |
| `D6`  | `TB6600` Head Pan Stepper Driver `EN`     |
| `D24` | `TB6600` Head Pan Stepper Driver `PUL`    |
| `D25` | `TB6600` Head Pan Stepper Driver `DIR`    |
| `D13` | SPI `SCK` (shared: Lamprey neck & torso)  |
| `D12` | SPI `MISO` (shared: Lamprey neck & torso) |
| `D0`  | Lamprey (neck) `CS`                       |
| `D1`  | Lamprey (torso) `CS`                      |
| `A1`  | Head Tilt Potentiometer `SIG`             |
| `A2`  | Torso Tilt Potentiometer 1 `SIG`          |
| `A3`  | Torso Tilt Potentiometer 2 `SIG`          |
| `D2`  | Head Pan Motor Driver 1 `LPWM`            |
| `D3`  | Head Pan Motor Driver 1 `RPWM`            |
| `D4`  | Head Pan Motor Driver 2 `LPWM`            |
| `D5`  | Head Pan Motor Driver 2 `RPWM`            |
| `D9`  | Torso Tilt Motor Driver 1 `LPWM`          |
| `D10` | Torso Tilt Motor Driver 1 `RPWM`          |
| `D22` | Torso Tilt Motor Driver 2 `LPWM`          |
| `D23` | Torso Tilt Motor Driver 2 `RPWM`          |
| `D26` | PerfectPass Servo `SIG`                   |

## Bill of Materials

|Component|Description|
|-|-|
|[CRCW060310K0FKEA](https://www.digikey.com/en/products/detail/vishay-dale/crcw060310k0fkea/1174782)|`10K` Pull-Down Resistor (PWM channels)|
||`47R` Series Resistor (PWM channels)|
||`2.2K` Series Resistor (ADC channels)|
||`470R` Series Resistor (Pink LEDs)|
||`47nF` Capacitor (ADC channels)|
|[JST_XH_B3B-XH-A](https://www.digikey.com/en/products/detail/jst-sales-america-inc/b3b-xh-a/1651046)|Current Sensor, Head Tilt Potentiometer, Torso Tilt Potentiometer, Knee Potentiometers (`3V3`, `SIG`, `GND`)|
|[JST_XH_B4B-XH-A](https://www.digikey.com/en/products/detail/jst-sales-america-inc/b4b-xh-a/1651047)|IMU, INA260 (`3V3`, `SDA`, `SCL`, `GND`)|
|[JST_XH_B3B-XH-A](https://www.digikey.com/en/products/detail/jst-sales-america-inc/b3b-xh-a/1651046)|Head Tilt Driver, Torso Tilt Drivers (`LPWM`, `RPWM`, `EN`)|
|[JST_XH_B4B-XH-A](https://www.digikey.com/en/products/detail/jst-sales-america-inc/b4b-xh-a/1651047)|Leg Actuator Drivers, Leg Wheel Drivers (`LPWM`, `RPWM`, `L_EN`, `R_EN`)|
|[JST_XH_B5B-XH-A](https://www.digikey.com/en/products/detail/jst-sales-america-inc/b5b-xh-a/1530483)|AS5045 Encoder, Lamprey Encoder (`3V3`, `CS`, `SCK`, `MISO`, `GND`)|
|[JST_XH_B8B-XH-A](https://www.digikey.com/en/products/detail/jst-sales-america-inc/b8b-xh-a/1651049)|CPM-MCVC-3441S-RLN Driver (`EN+`, `EN-`, `A+`, `A-`, `B+`, `B-`, `ST+`, `ST-`)|
|[JST_XH_B4B-XH-A](https://www.digikey.com/en/products/detail/jst-sales-america-inc/b4b-xh-a/1651047)|TB6600 Driver (`3V3`, `PUL`, `DIR`, `ENA`)|
