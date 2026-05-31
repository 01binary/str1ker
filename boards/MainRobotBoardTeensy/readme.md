# Main Robot Board

A board that controls robot head & body and measures voltage/current/battery charge.

+ Head Pan/Tilt
+ Torso Pan/Tilt
+ Mouth Movement
+ Voltage and Current Sensing

## Modules

The following components are placed onto the board as modules:

|Module|Function|
|-|-|
|[Teensy 4.0](https://www.sparkfun.com/teensy-4-0.html)|[ROS](https://www.ros.org/) Node|
|[ACS37220 Current Sensor](https://www.pololu.com/product/5295)|Bus Current Sensing|
|[SSD1306 OLED Display](https://www.amazon.com/dp/B00O2LKEW2)|Monochrome 0.96" 128x64 Voltage/Current/Power display|

## Devices

The following external components are connected to the board via JST-XH locking connectors:

|Device|Function|
|-|-|
|[TB6600HG Stepper Motor Driver](https://www.amazon.com/dp/B01N6AIEQT)|Head Pan Motor|
|4x [Mini IBT Motor Driver](https://www.aliexpress.us/item/2251832603816751.html)|2x Head Tilt Motor Driver, 2x Torso Tilt Motor Driver|
|[Lamprey 2 Absolute Encoder](https://andymark.com/products/lamprey2-absolute-encoder)|Head Pan Encoder|
|[Lamprey 2 Absolute Encoder 4 inch](https://andymark.com/products/lamprey2-4-inch-absolute-encoder)|Torso Pan Encoder|
|3x Hollow Shaft Potentiometers|1x Head Tilt Encoder, 2x Torso Tilt Encoders|
|[PerfectPass 56Kg Servo](https://www.amazon.com/dp/B09Y4NZJBJ)|Mouth Expressions|
|[CPM-MCVC-3441S-RLN](https://teknic.com/model-info/CPM-MCVC-3441S-RLN/?model_voltage=75VDC)|Torso Pan Motor Driver

The two encoders can also be connected through USB.

## Buses

|Bus|Devices
|-|-|
| `I2C` | SSD1306 voltage/current display
| `SPI` | Lamprey encoders
| `SPI` | Shift Registers for battery level LED meter

## Pins

|Pin|Function
|-|-|
| `D19` | I2C `SCL` (SSD1306)
| `D18` | I2C `SDA` (SSD1306)
| `A0`  | `ACS37220` Current Sensor `VOUT`
| `D20` | `CPM-MCVC-3441S-RLN` Torso Motor `DIR` (A)
| `D21` | `CPM-MCVC-3441S-RLN` Torso Motor `PWM` (B)
| `D7`  | `CPM-MCVC-3441S-RLN` Torso Motor `Enable`
| `D8`  | `CPM-MCVC-3441S-RLN` Torso Motor `Status`
| `D6`  | `TB6600` Head Pan Stepper Driver `EN`
| `D24` | `TB6600` Head Pan Stepper Driver `PUL`
| `D25` | `TB6600` Head Pan Stepper Driver `DIR`
| `D13` | SPI `SCK` (Lamprey Encoders, Shift Registers)
| `D11` | SPI `MOSI` (Shift Registers)
| `D12` | SPI `MISO` (Lamprey Encoders)
| `D31` | Shift Register `RCLK` (latch)
| `D0`  | Lamprey head encoder `CS`
| `D1`  | Lamprey torso encoder `CS`
| `A1`  | Head Tilt Potentiometer `SIG`
| `A2`  | Torso Tilt Potentiometers (Average) `SIG`
| `A3`  | Voltage Sense `SIG`
| `D27` | Head Tilt Motor Driver 1 `EN`
| `D2`  | Head Tilt Motor Driver 1 `LPWM`
| `D3`  | Head Tilt Motor Driver 1 `RPWM`
| `D28` | Head Tilt Motor Driver 2 `EN`
| `D4`  | Head Tilt Motor Driver 2 `LPWM`
| `D5`  | Head Tilt Motor Driver 2 `RPWM`
| `D29` | Torso Tilt Motor Driver 1 `EN`
| `D9`  | Torso Tilt Motor Driver 1 `LPWM`
| `D10` | Torso Tilt Motor Driver 1 `RPWM`
| `D30` | Torso Tilt Motor Driver 2 `EN`
| `D22` | Torso Tilt Motor Driver 2 `LPWM`
| `D23` | Torso Tilt Motor Driver 2 `RPWM`
| `D26` | PerfectPass Servo `SIG`

## Shift Registers

Two shift registers are used to expand Teensy I/O capabilities, adding a total of 16 digital pins:

| Shift Register | Pin | Function
|-|-|-|
| `U8` | `QA` | Battery Level Meter `LED1` (Lowest Charge)
| `U8` | `QB` | Battery Level Meter `LED2`
| `U8` | `QC` | Battery Level Meter `LED3`
| `U8` | `QD` | Battery Level Meter `LED4`
| `U8` | `QE` | Battery Level Meter `LED5`
| `U8` | `QF` | Battery Level Meter `LED6`
| `U8` | `QG` | Battery Level Meter `LED7`
| `U8` | `QH` | Battery Level Meter `LED8`
| `U9` | `QA` | Battery Level Meter `LED9`
| `U9` | `QB` | Battery Level Meter `LED10` (Highest Charge)
| `U9` | `QC` | Torso Encoder Status `TORSOENCODERST`
| `U9` | `QD` | Head Encoder Status `HEADENCODERST`

## Encoders

The Lamprey 2 Absolute Encoders accept a [JST Molex PicoBlade](https://www.amazon.com/dp/B07S18D3RN) 5x2 connector with a 10-wire ribbon cable:

```
SCK   TX    RX    A     RST
GND   MOSI  PWM   MISO  VSS
```

|Pin|Function|
|-|-|
|1|`RESET`
|2|`VSS`
|3|`ANALOG OUT`
|4|`MISO`
|5|`RX`
|6|`PWM OUT`
|7|`TX`
|8|`MOSI`
|9|`SCK`
|10|`GND`

The Lamprey 2 Absolute Encoder (4 Inch) also supports a USB interface. When the device is assigned a Linux port (e.g. `ttyACM0`) the following commands can be sent:

|Command|Description|
|-|-|
|`d`|Output in degrees (default)
|`r`|Output in radians
|`a`|Debug output
|`p`|PWM output (`0`- `4095`)
|`5`|5V analog output
|`3`|3.3V analog output
|`0`|Set zero point
|`f`|Enable [Finite Impulse Response](https://wirelesspi.com/finite-impulse-response-fir-filters/) filter

> Both encoders require calibration, see [Lamprey 4 inch instructions](https://andymark.com/products/lamprey2-4-inch-absolute-encoder) and [Lamprey instructions](https://s3.amazonaws.com/docusync-files/8e336754b99eaa57f816a158bf15fce1721f511b38099a9a28875963cc3a0f22/am-4179a%20Lamprey%20Encoder%20Calibration%20Instructions.pdf). 

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
|[0603WAF2202T5E](https://jlcpcb.com/partdetail/32812-0603WAF2202T5E/C31850)|`22K` Voltage Sense Divider Resistor|
|[CL10B473KB8NNNC](https://www.digikey.com/en/products/detail/samsung-electro-mechanics/cl10b473kb8nnnc/3886721)|`47nF` Capacitor (ADC channels)|
|[SN74HC08DR](https://www.digikey.com/en/products/detail/texas-instruments/sn74hc08dr/276834)|IC Gate|
|[SN74HC595DR](https://www.digikey.com/en/products/detail/texas-instruments/sn74hc595dr/562919)|IC Shift Register|
|[KWL-R1025BB](https://www.adafruit.com/product/1815)|LED Light Bar
|[MMBT3904LT1G](https://www.digikey.com/en/products/detail/onsemi/MMBT3904LT1G/919601)|LED Transistor|
|[TLC555CDR](https://www.digikey.com/en/products/detail/texas-instruments/tlc555cdr/276979)|555 Timer|
|[RC0603FR-07360KL](https://www.digikey.com/en/products/detail/yageo/rc0603fr-07360kl/727183)|555 Timer Resistor `360K`|
|[CC0603JRX7R7BB105](https://www.digikey.com/en/products/detail/yageo/CC0603JRX7R7BB105/7164369)|555 Timer Capacitor `1uF`|
|[C0402C103J4RACTU](https://www.digikey.com/en/products/detail/kemet/C0402C103J4RACTU/411041)|555 Timer Capacitor `0.01uF`|
|[JST_XH_B3B-XH-A](https://www.digikey.com/en/products/detail/jst-sales-america-inc/b3b-xh-a/1651046)|Current Sensor (`3V3`, `VOUT`, `GND`), Head Tilt Potentiometer, Torso Tilt Potentiometer, Head Tilt Driver, Torso Tilt Drivers (`LPWM`, `RPWM`, `EN`)|
|[JST_XH_B4B-XH-A](https://www.digikey.com/en/products/detail/jst-sales-america-inc/b4b-xh-a/1651047)|INA260 (`3V3`, `SDA`, `SCL`, `GND`), TB6600 Driver (`3V3`, `PUL`, `DIR`, `ENA`)|
|[JST_XH_B5B-XH-A](https://www.digikey.com/en/products/detail/jst-sales-america-inc/b5b-xh-a/1530483)|AS5045 Encoder, Lamprey Encoder (`3V3`, `CS`, `SCK`, `MISO`, `GND`)|
|2x4 IDC connector|CPM-MCVC-3441S-RLN Driver (`EN+`, `EN-`, `A+`, `A-`, `B+`, `B-`, `ST+`, `ST-`)|
