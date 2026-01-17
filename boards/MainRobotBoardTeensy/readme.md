# Main Robot Board

A board that controls robot head, body, and legs (everything but the arms).

+ Head Pan/Tilt
+ Mouth Movement
+ Torso Pan/Tilt
+ Leg Raise/Lower
+ Leg Drive
+ Voltage and Current Sensing
+ 9-DOF IMU for Odometry

## Modules

The following components are placed onto the board as modules:

|Module|Function|
|-|-|
|[Teensy 4.0](https://www.sparkfun.com/teensy-4-0.html)|ROS Node|
|[Adafruit 9-DOF IMU](https://www.adafruit.com/product/2472)|Odometry|
|[PCA9685 Adafruit 16-Channel PWM Driver](https://www.adafruit.com/product/815)|Motor and Servo Control|
|[INA260 Voltage Sensor](https://learn.adafruit.com/adafruit-ina260-current-voltage-power-sensor-breakout)|Bus Voltage Sensing|
|[ACS37220 Current Sensor](https://www.pololu.com/product/5295)|Bus Current Sensing|

## Devices

The following external components are connected to the board via JST-XH locking connectors:

|Device|Function|
|-|-|
|[1x TB6600HG Stepper Motor Driver](https://www.amazon.com/dp/B01N6AIEQT)|Head Pan|
|[1x Mini IBT Motor Driver](https://www.aliexpress.us/item/2251832603816751.html)|Head Tilt|
|[2x Mini IBT Motor Driver](https://www.aliexpress.us/item/2251832603816751.html)|Torso Tilt|
|[4x IBT2 Motor Drivers](https://www.amazon.com/2-Piece-High-Power-Limiting-Function-Suitable/dp/B0DXKXKYRK)|Wheel Motor Drivers|
|[4x IBT2 Motor Drivers](https://www.amazon.com/2-Piece-High-Power-Limiting-Function-Suitable/dp/B0DXKXKYRK)|Leg Actuator Drivers|
|[Lamprey 2 Absolute Encoder 4 inch](https://andymark.com/products/lamprey2-4-inch-absolute-encoder)|Torso Pan Encoder|
|[2x NP24HS Hollow Shaft Potentiometer](https://p3america.com/np24hs-series/)|Head Tilt Encoder, Torso Tilt Encoder|
|[AS5045 Hall Effect Encoder](https://www.digikey.com/en/products/detail/ams-osram-usa-inc/as5045-adapterboard/2339623)|Head Pan Encoder|
|[4x NP32HS Hollow Shaft Potentiometer](https://p3america.com/np32hs-series/)|Knee Encoder|
|[PerfectPass 56Kg Servo](https://www.amazon.com/dp/B09Y4NZJBJ)|Mouth Movement|
|[CPM-MCVC-3441S-RLN](https://teknic.com/model-info/CPM-MCVC-3441S-RLN/?model_voltage=75VDC)|Torso Pan Motor Driver

## Buses

| Bus   | Devices              |
| ----- | -------------------- |
| `I2C` | IMU, INA260, PCA9685 |
| `SPI` | AS5045, Lamprey      |

## Pins

| Pin   | Function                             |
| ----- | ------------------------------------ |
| `A5`  | I2C SCL (9-DOF IMU, INA260, PCA9685) |
| `A4`  | I2C SDA (9-DOF IMU, INA260, PCA9685) |
| `A0`  | ACS37220 Current Sensor VOUT         |
| `D16` | Legs Enable                          |
| `D17` | Head Enable                          |
| `D18` | Torso Enable                         |
| `D4`  | CPM-MCVC-3441S-RLN Enable            |
| `D2`  | CPM-MCVC-3441S-RLN A                 |
| `D3`  | CPM-MCVC-3441S-RLN B                 |
| `D5`  | CPM-MCVC-3441S-RLN Status            |
| `D7`  | TB6600 Head Pan PUL                  |
| `D8`  | TB6600 Head Pan DIR                  |
| `D14` | Lamprey CS                           |
| `D13` | Lamprey SCK                          |
| `D12` | Lamprey MISO                         |
| `D15` | AS5045 CS                            |
| `D13` | AS5045 SCK                           |
| `D12` | AS5045 MISO                          |
| `A1`  | Head Tilt Potentiometer SIG          |
| `A2`  | Torso Tilt Potentiometer SIG         |
| `A3`  | Knee 1 Potentiometer SIG             |
| `A6`  | Knee 2 Potentiometer SIG             |
| `A7`  | Knee 3 Potentiometer SIG             |
| `A8`  | Knee 4 Potentiometer SIG             |

## PWM

| PCA9685 Channel | Function                                                    |
| --------------- | ----------------------------------------------------------- |
| `CH0`           | Head Tilt Actuator Driver LPWM                              |
| `CH1`           | Head Tilt Actuator Driver RPWM                              |
| `CH2`           | Leg Actuator Drivers LPWM (shared, fanned out to 4 drivers) |
| `CH3`           | Leg Actuator Drivers RPWM (shared, fanned out to 4 drivers) |
| `CH4`           | Leg 1 Wheel Driver LPWM                                     |
| `CH5`           | Leg 1 Wheel Driver RPWM                                     |
| `CH6`           | Leg 2 Wheel Driver LPWM                                     |
| `CH7`           | Leg 2 Wheel Driver RPWM                                     |
| `CH8`           | Leg 3 Wheel Driver LPWM                                     |
| `CH9`           | Leg 3 Wheel Driver RPWM                                     |
| `CH10`          | Leg 4 Wheel Driver LPWM                                     |
| `CH11`          | Leg 4 Wheel Driver RPWM                                     |
| `CH12`          | PerfectPass SIG (Servo PWM)                                 |
| `CH13`          | Torso Tilt Driver LPWM                                      |
| `CH14`          | Torso Tilt Driver RPWM                                      |

## Bill of Materials

|Component|Description|
|-|-|
|[CRCW060310K0FKEA](https://www.digikey.com/en/products/detail/vishay-dale/crcw060310k0fkea/1174782)|`10K` Pull-Down Resistor (PWM channels)|
||`47R` Series Resistor (PWM channels)|
||`2.2K` Series Resistor (ADC channels)|
||`150R` Series Resistor (Blue LEDs)|
||`470R` Series Resistor (Pink LEDs)|
||`47nF` Capacitor (ADC channels)|
|[JST_XH_B3B-XH-A](https://www.digikey.com/en/products/detail/jst-sales-america-inc/b3b-xh-a/1651046)|Current Sensor, Head Tilt Potentiometer, Torso Tilt Potentiometer, Knee Potentiometers (`3V3`, `SIG`, `GND`)|
|[JST_XH_B4B-XH-A](https://www.digikey.com/en/products/detail/jst-sales-america-inc/b4b-xh-a/1651047)|IMU, INA260, PCA9685 (`3V3`, `SDA`, `SCL`, `GND`)|
|[JST_XH_B3B-XH-A](https://www.digikey.com/en/products/detail/jst-sales-america-inc/b3b-xh-a/1651046)|Head Tilt Driver, Torso Tilt Drivers (`LPWM`, `RPWM`, `EN`)|
|[JST_XH_B4B-XH-A](https://www.digikey.com/en/products/detail/jst-sales-america-inc/b4b-xh-a/1651047)|Leg Actuator Drivers, Leg Wheel Drivers (`LPWM`, `RPWM`, `L_EN`, `R_EN`)|
|[JST_XH_B5B-XH-A](https://www.digikey.com/en/products/detail/jst-sales-america-inc/b5b-xh-a/1530483)|AS5045 Encoder, Lamprey Encoder (`3V3`, `CS`, `SCK`, `MISO`, `GND`)|
|[JST_XH_B8B-XH-A](https://www.digikey.com/en/products/detail/jst-sales-america-inc/b8b-xh-a/1651049)|CPM-MCVC-3441S-RLN Driver (`EN+`, `EN-`, `A+`, `A-`, `B+`, `B-`, `ST+`, `ST-`)|
|[JST_XH_B4B-XH-A](https://www.digikey.com/en/products/detail/jst-sales-america-inc/b4b-xh-a/1651047)|TB6600 Driver (`3V3`, `PUL`, `DIR`, `ENA`)|
