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
|[Teensy 4.1](https://www.sparkfun.com/teensy-4-1.html)|[ROS](https://www.ros.org/) Node|
|[ACS37220 Current Sensor 100A](https://www.pololu.com/product/5295)|Bus Current Sensing|
|[SSD1306 OLED Displays](https://www.amazon.com/dp/B00O2LKEW2)|Monochrome 0.96" 128x64 Voltage/Current/Power displays|

## Devices

The following external components are connected to the board via JST-XH locking connectors:

|Device|Function|
|-|-|
|[TB6600HG Stepper Motor Driver](https://www.amazon.com/dp/B01N6AIEQT)|Head Pan Motor|
|`4x` [Mini IBT Motor Driver](https://www.aliexpress.us/item/2251832603816751.html)|`2x` Head Tilt Motor Driver, `2x` Torso Tilt Motor Driver|
|`2x` Hollow Shaft Potentiometers|Head Tilt Encoder, Torso Tilt Encoder|
|[PerfectPass 56Kg Servo](https://www.amazon.com/dp/B09Y4NZJBJ)|Mouth Movement|
|[CPM-MCVC-3441S-RLN](https://teknic.com/model-info/CPM-MCVC-3441S-RLN/?model_voltage=75VDC)|Torso Pan Integrated Motor Driver

## Buses

|Bus|Device|Address
|-|-|-|
|`I2C`|SSD1306 voltage, current, and power meter displays|`0x3C`, `0x3D`|
|`I2C`|PCA9685 PWM driver|`0x40`|

## Functionality

### PWM

The board runs several PWM-controlled motor drivers and a servo.

[PCA9685PW,118](https://www.digikey.com/en/products/detail/nxp-usa-inc/pca9685pw-118/2034325) PWM driver is used to run these motor drivers over `I2C` bus to avoid over-allocating Teensy pins.

### ADC

The potentiometers that measure head tilt and torso tilt are routed to Teensy `ADC` pins.

Head and torso pan are measured by [Lamprey 2 Absolute Encoder](https://andymark.com/products/lamprey2-absolute-encoder) and [Lamprey 2 Absolute Encoder 4 inch](https://andymark.com/products/lamprey2-4-inch-absolute-encoder) which are connected to USB, and thus do not appear on this board.

Two more ADC pins are used as analog inputs for voltage and current sensing. Voltage sensing is done with a simple voltage divider and current sending by [ACS37220](https://www.pololu.com/product/5295) from Pololu.

## Pins

|Pin|Function
|-|-|
| `A0`  | `ACS37220` Current Sensor `VOUT`
| `A1`  | Head Tilt Potentiometer `SIG`
| `A2`  | Torso Tilt Potentiometers (Average) `SIG`
| `A3`  | Voltage Sense `SIG`
| `D19` | I2C `SCL` (SSD1306 x2)
| `D18` | I2C `SDA` (SSD1306 x2)
| `D0` | `TB6600` Head Pan Stepper Driver `EN`
| `D1` | `TB6600` Head Pan Stepper Driver `DIR`
| `D2` | `CPM-MCVC-3441S-RLN` Torso Motor `EN`
| `D3` | `CPM-MCVC-3441S-RLN` Torso Motor `DIR` (A)
| `D4` | `CPM-MCVC-3441S-RLN` Torso Motor `Status`
| `D5` | Head Tilt Motor Driver 1 `EN`
| `D6` | Head Tilt Motor Driver 2 `EN`
| `D7` | Torso Tilt Motor Driver 1 `EN`
| `D8` | Torso Tilt Motor Driver 2 `EN`
| `D9` | Battery Level Meter `LED1` (Lowest Charge)
| `D10` | Battery Level Meter `LED2`
| `D20` | Battery Level Meter `LED3`
| `D21` | Battery Level Meter `LED4`
| `D22` | Battery Level Meter `LED5`
| `D23` | Battery Level Meter `LED6`
| `D24` | Battery Level Meter `LED7`
| `D25` | Battery Level Meter `LED8`
| `D26` | Battery Level Meter `LED9`
| `D27` | Battery Level Meter `LED10` (Highest Charge)
| `PCA9685 CH1`  | `CPM-MCVC-3441S-RLN` Torso Motor `PWM` (B)
| `PCA9685 CH2`  | `TB6600` Head Pan Stepper Driver `PUL`
| `PCA9685 CH3`  | Head Tilt Motor Driver 1 `LPWM`
| `PCA9685 CH4`  | Head Tilt Motor Driver 1 `RPWM`
| `PCA9685 CH5`  | Head Tilt Motor Driver 2 `LPWM`
| `PCA9685 CH6`  | Head Tilt Motor Driver 2 `RPWM`
| `PCA9685 CH7`  | Torso Tilt Motor Driver 1 `LPWM`
| `PCA9685 CH8`  | Torso Tilt Motor Driver 1 `RPWM`
| `PCA9685 CH9`  | Torso Tilt Motor Driver 2 `LPWM`
| `PCA9685 CH10` | Torso Tilt Motor Driver 2 `RPWM`
| `PCA9685 CH11` | PerfectPass Servo `SIG`

## Bill of Materials

|Component|Description|
|-|-|
|[150080BS75000](https://www.digikey.com/en/products/detail/w%C3%BCrth-elektronik/)|Blue LED for `LPWM`/`RPWM` signals|
|[XL-1608SURC-06](https://www.lcsc.com/product-detail/C965799.html)|Red LED for `EN` signals|
|[RC0603FR-0747RL](https://www.digikey.com/en/products/detail/yageo/rc0603fr-0747rl/727252)|`47R` Series Resistor (PWM channels)|
|[RC0603FR-07150RL](https://www.digikey.com/en/products/detail/yageo/RC0603FR-07150RL/726958)|`150R` Blue LED Resistor|
|[RC0603FR-07470RL](https://www.digikey.com/en/products/detail/yageo/RC0603FR-07470RL/727256)|`470R` Red LED Resistor|
|[RCG06031K00FKEA](https://www.digikey.com/en/products/detail/vishay-dale/rcg06031k00fkea/4172389)|`1K` Red LED Resistor|
|[RC0603FR-072K2L](https://www.digikey.com/en/products/detail/yageo/RC0603FR-072K2L/727016)|`2.2K` Series Resistor (ADC channels)|
|[CRCW060310K0FKEA](https://www.digikey.com/en/products/detail/vishay-dale/crcw060310k0fkea/1174782)|`10K` Pull-Down Resistor (PWM channels)|
|[0603WAF2202T5E](https://jlcpcb.com/partdetail/32812-0603WAF2202T5E/C31850)|`22K` Voltage Sense Divider Resistor|
|[CL10B473KB8NNNC](https://www.digikey.com/en/products/detail/samsung-electro-mechanics/cl10b473kb8nnnc/3886721)|`47nF` Capacitor (ADC channels)|
|[SN74HC08DR](https://www.digikey.com/en/products/detail/texas-instruments/sn74hc08dr/276834)|IC Gate|
|[B10 1B4PG3Y2R](https://www.amazon.com/dp/B0C9QGMGFB)|LED Light Bar (Red/Blue/Green) with `DIP-20` Socket (Battery Charge Meter)|
|[MMBT3904LT1G](https://www.digikey.com/en/products/detail/onsemi/MMBT3904LT1G/919601)|LED Transistor|
|[TLC555CDR](https://www.digikey.com/en/products/detail/texas-instruments/tlc555cdr/276979)|555 Timer|
|[RC0603FR-07360KL](https://www.digikey.com/en/products/detail/yageo/rc0603fr-07360kl/727183)|555 Timer Resistor `360K`|
|[CC0603JRX7R7BB105](https://www.digikey.com/en/products/detail/yageo/CC0603JRX7R7BB105/7164369)|555 Timer Capacitor `1uF`|
|[C0402C103J4RACTU](https://www.digikey.com/en/products/detail/kemet/C0402C103J4RACTU/411041)|555 Timer Capacitor `0.01uF`|
|[JST_XH_B3B-XH-A](https://www.digikey.com/en/products/detail/jst-sales-america-inc/b3b-xh-a/1651046)|Head Tilt Potentiometer, Torso Tilt Potentiometer, Head Tilt Driver, Torso Tilt Drivers (`LPWM`, `RPWM`, `EN`)|
|[JST_XH_B4B-XH-A](https://www.digikey.com/en/products/detail/jst-sales-america-inc/b4b-xh-a/1651047)|4-pin male receptacle for `TB6600` Driver (`3V3`, `PUL`, `DIR`, `ENA`)|
|[Molex 0702460802](https://www.digikey.com/en/products/detail/molex/0702460802/760180)|2-row 4-pin male receptacle for `CPM-MCVC-3441S-RLN` Driver (`EN+`, `EN-`, `A+`, `A-`, `B+`, `B-`, `ST+`, `ST-`)|
|[Teensy 4.1](https://www.sparkfun.com/teensy-4-1.html)|Teensy 4.1 without Headers|
|[Needle Headers](https://www.amazon.com/dp/B09WH8N3HX)|Removable Socket Headers|
|[ACS37220](https://www.pololu.com/product/5295)|Current Sensor `100A`, `3.3V` Logic|
|2x [SSD1306](https://www.amazon.com/dp/B00O2LKEW2)|Monochrome I2C OLED Displays `3.3V`|