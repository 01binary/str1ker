# Main Robot Board

A board that controls robot head and body.

+ Head Pan/Tilt
+ Mouth Movement
+ Torso Pan/Tilt
+ Voltage and Current Sensing
+ 9-DOF IMU

## Modules

The following components are placed onto the board as modules:

|Module|Function|
|-|-|
|[Teensy 4.0](https://www.sparkfun.com/teensy-4-0.html)|[ROS](https://www.ros.org/) Node|
|[Adafruit 9-DOF IMU](https://www.adafruit.com/product/2472)|[TF2](https://wiki.ros.org/tf2) Transform|
|[INA260 Voltage Sensor](https://learn.adafruit.com/adafruit-ina260-current-voltage-power-sensor-breakout)|Bus Voltage Sensing|
|[ACS37220 Current Sensor](https://www.pololu.com/product/5295)|Bus Current Sensing|

## Devices

The following external components are connected to the board via JST-XH locking connectors:

|Device|Function|
|-|-|
|[TB6600HG Stepper Motor Driver](https://www.amazon.com/dp/B01N6AIEQT)|Head Pan|
|4x [Mini IBT Motor Driver](https://www.aliexpress.us/item/2251832603816751.html)|Head Tilt (2), Torso Tilt (2)|
|[Lamprey 2 Absolute Encoder 4 inch](https://andymark.com/products/lamprey2-4-inch-absolute-encoder)|Torso Pan Encoder|
|2x [NP24HS Hollow Shaft Potentiometer](https://p3america.com/np24hs-series/)|Head Tilt Encoder, Torso Tilt Encoder|
|[AS5045 Hall Effect Encoder](https://www.digikey.com/en/products/detail/ams-osram-usa-inc/as5045-adapterboard/2339623)|Head Pan Encoder|
|[PerfectPass 56Kg Servo](https://www.amazon.com/dp/B09Y4NZJBJ)|Mouth Movement|
|[CPM-MCVC-3441S-RLN](https://teknic.com/model-info/CPM-MCVC-3441S-RLN/?model_voltage=75VDC)|Torso Pan Motor Driver

## Buses

| Bus   | Devices              |
| ----- | -------------------- |
| `I2C` | IMU, INA260          |
| `SPI` | AS5045, Lamprey      |

## Pins

| Pin   | Function                                                   |
| ----- | ---------------------------------------------------------- |
| `D19` | I2C `SCL` (9-DOF IMU, INA260)                              |
| `D18` | I2C `SDA` (9-DOF IMU, INA260)                              |
| `A0`  | `ACS37220` Current Sensor `VOUT`                           |
| `D2`  | `CPM-MCVC-3441S-RLN` Torso Motor `A`                       |
| `D3`  | `CPM-MCVC-3441S-RLN` Torso Motor `B`                       |
| `D4`  | `CPM-MCVC-3441S-RLN` Torso Motor `Enable`                  |
| `D5`  | `CPM-MCVC-3441S-RLN` Torso Motor `Status`                  |
| `D7`  | `TB6600` Head Pan Stepper Driver `PUL`                     |
| `D8`  | `TB6600` Head Pan Stepper Driver `DIR`                     |
| `D13` | SPI `SCK` (shared: Lamprey, AS5045)                        |
| `D12` | SPI `MISO` (shared: Lamprey, AS5045)                       |
| `D0`  | Lamprey `CS`                                               |
| `D1`  | AS5045 `CS`                                                |
| `A1`  | Head Tilt Potentiometer `SIG`                              |
| `A2`  | Torso Tilt Potentiometer `SIG`                             |
| `D9`  | Head Tilt Drivers `LPWM`                                   |
| `D10` | Head Tilt Drivers `RPWM`                                   |
| `D20` | Torso Tilt Drivers `LPWM`                                  |
| `D21` | Torso Tilt Drivers `RPWM`                                  |
| `D6`  | PerfectPass Servo `SIG`                                    |

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
