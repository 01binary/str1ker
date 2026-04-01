# Main Robot Board

A board that controls robot head & body and measures battery charge.

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
|3x Hollow Shaft Potentiometers|1x Head Tilt Encoder, 2x Torso Tilt Encoders|
|[PerfectPass 56Kg Servo](https://www.amazon.com/dp/B09Y4NZJBJ)|Mouth Movement|
|[CPM-MCVC-3441S-RLN](https://teknic.com/model-info/CPM-MCVC-3441S-RLN/?model_voltage=75VDC)|Torso Pan Motor Driver

## Buses

| Bus   | Devices                       |
| ----- | ----------------------------- |
| `I2C` | INA260 voltage/current sensor |
| `SPI` | Lamprey encoders              |

## Pins

| Pin   | Function                                   |
| ----- | ------------------------------------------ |
| `D19` | I2C `SCL` (INA260)                         |
| `D18` | I2C `SDA` (INA260)                         |
| `A0`  | `ACS37220` Current Sensor `VOUT`           |
| `D20` | `CPM-MCVC-3441S-RLN` Torso Motor `DIR` (A) |
| `D21` | `CPM-MCVC-3441S-RLN` Torso Motor `PWM` (B) |
| `D7`  | `CPM-MCVC-3441S-RLN` Torso Motor `Enable`  |
| `D8`  | `CPM-MCVC-3441S-RLN` Torso Motor `Status`  |
| `D6`  | `TB6600` Head Pan Stepper Driver `EN`      |
| `D24` | `TB6600` Head Pan Stepper Driver `PUL`     |
| `D25` | `TB6600` Head Pan Stepper Driver `DIR`     |
| `D13` | SPI `SCK` (shared: Lamprey neck & torso)   |
| `D12` | SPI `MISO` (shared: Lamprey neck & torso)  |
| `D0`  | Lamprey (head) `CS`                        |
| `D1`  | Lamprey (torso) `CS`                       |
| `A1`  | Head Tilt Potentiometer `SIG`              |
| `A2`  | Torso Tilt Potentiometer 1 `SIG`           |
| `A3`  | Torso Tilt Potentiometer 2 `SIG`           |
| `D27` | Head Tilt Motor Driver 1 `EN`              |
| `D2`  | Head Tilt Motor Driver 1 `LPWM`            |
| `D3`  | Head Tilt Motor Driver 1 `RPWM`            |
| `D28` | Head Tilt Motor Driver 2 `EN`              |
| `D4`  | Head Tilt Motor Driver 2 `LPWM`            |
| `D5`  | Head Tilt Motor Driver 2 `RPWM`            |
| `D29` | Torso Tilt Motor Driver 1 `EN`             |
| `D9`  | Torso Tilt Motor Driver 1 `LPWM`           |
| `D10` | Torso Tilt Motor Driver 1 `RPWM`           |
| `D30` | Torso Tilt Motor Driver 2 `EN`             |
| `D22` | Torso Tilt Motor Driver 2 `LPWM`           |
| `D23` | Torso Tilt Motor Driver 2 `RPWM`           |
| `D26` | PerfectPass Servo `SIG`                    |

## Bill of Materials

|Component|Description|
|-|-|
|[150080BS75000](https://www.digikey.com/en/products/detail/w%C3%BCrth-elektronik/)|Blue LED for `LPWM`/`RPWM` signals|
|[NCD0603R1](https://www.lcsc.com/product-detail/C84263.html?s_z=s_C84263)|Red LED for `EN` signals|
|[RC0603FR-0747RL](https://www.digikey.com/en/products/detail/yageo/rc0603fr-0747rl/727252)|`47R` Series Resistor (PWM channels)|
|[RC0603FR-07150RL](https://www.digikey.com/en/products/detail/yageo/RC0603FR-07150RL/726958)|`150R` Blue LED Resistor|
|[RC0603FR-07470RL](https://www.digikey.com/en/products/detail/yageo/RC0603FR-07470RL/727256)|`470R` Red LED Resistor|
|[RCG06031K00FKEA](https://www.digikey.com/en/products/detail/vishay-dale/rcg06031k00fkea/4172389)|`1K` Red LED Resistor|
|[RC0603FR-072K2L](https://www.digikey.com/en/products/detail/yageo/RC0603FR-072K2L/727016)|`2.2K` Series Resistor (ADC channels)|
|[CRCW060310K0FKEA](https://www.digikey.com/en/products/detail/vishay-dale/crcw060310k0fkea/1174782)|`10K` Pull-Down Resistor (PWM channels)|
|[CL10B473KB8NNNC](https://www.digikey.com/en/products/detail/samsung-electro-mechanics/cl10b473kb8nnnc/3886721)|`47nF` Capacitor (ADC channels)|
|[SN74HC08DR](https://www.digikey.com/en/products/detail/texas-instruments/sn74hc08dr/276834)|IC Gate|
|[MMBT3904LT1G](https://www.digikey.com/en/products/detail/onsemi/MMBT3904LT1G/919601)|LED Transistor|
|[TLC555CDR](https://www.digikey.com/en/products/detail/texas-instruments/tlc555cdr/276979)|555 Timer|
|[RC0603FR-07360KL](https://www.digikey.com/en/products/detail/yageo/rc0603fr-07360kl/727183)|555 Timer Resistor `360K`|
|[CC0603JRX7R7BB105](https://www.digikey.com/en/products/detail/yageo/CC0603JRX7R7BB105/7164369)|555 Timer Capacitor `1uF`|
|[C0402C103J4RACTU](https://www.digikey.com/en/products/detail/kemet/C0402C103J4RACTU/411041)|555 Timer Capacitor `0.01uF`|
|[JST_XH_B3B-XH-A](https://www.digikey.com/en/products/detail/jst-sales-america-inc/b3b-xh-a/1651046)|Current Sensor (`3V3`, `VOUT`, `GND`), Head Tilt Potentiometer, Torso Tilt Potentiometer, Head Tilt Driver, Torso Tilt Drivers (`LPWM`, `RPWM`, `EN`)|
|[JST_XH_B4B-XH-A](https://www.digikey.com/en/products/detail/jst-sales-america-inc/b4b-xh-a/1651047)|INA260 (`3V3`, `SDA`, `SCL`, `GND`), TB6600 Driver (`3V3`, `PUL`, `DIR`, `ENA`)|
|[JST_XH_B5B-XH-A](https://www.digikey.com/en/products/detail/jst-sales-america-inc/b5b-xh-a/1530483)|AS5045 Encoder, Lamprey Encoder (`3V3`, `CS`, `SCK`, `MISO`, `GND`)|
|2x4 IDC connector|CPM-MCVC-3441S-RLN Driver (`EN+`, `EN-`, `A+`, `A-`, `B+`, `B-`, `ST+`, `ST-`)|