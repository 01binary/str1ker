# Arm Encoder Board

A custom board that simplifies mounting `AS5045` hall-effect on-axis encoder with 12-bit resolution onto robot joints.

> Ready-made alternatives like [AS5045 Adapter Board](https://www.digikey.com/en/products/detail/ams-osram-usa-inc/as5045-adapterboard/2339623) do exist, but have extra pins not used in this project, and a footprint that makes them hard to mount.

## Power

For `3.3V` operation `VDD5V` and `VDD3V3` are bridged and routed to `VIN` (3.3V). A single `100nF` decoupling capacitor is placed at the bridge (close to the IC) to `GND`.

## Interface

The following is documented in [AS5045 specifications](./doc/AS5045Encoder.pdf):

+ SSI / synchronous serial output (no MOSI)
+ Compatible with SPI mode 1 (`CPOL` = `0`, `CPHA` = `1`)
+ Transfer begins when `CSn` is `LOW`
+ MSB-first, clocked data output

The encoder continuously shifts out a 16-bit data frame containing the absolute position and flags in response to clock input without having to request this information - that's what makes its interface *SSI* instead of *SPI*.

## Data Format

The data frame contains a 12-bit position value, status flags, and parity.

|Offset|Size|Data|
|-|-|-|
|0|12|Position/Angle (MSB-first)|
|12|1|`OCF` - Offset Compensation Finished|
|13|1|`COF` - Cordic Overflow/Out of Range|
|14|1|`LIN` - Linearity Alarm|
|15|1|`MacINC` - Magnet too far|
|16|1|`MagDEC` - Magnet too close|
|17|1|Parity for error-checking|

## Indicators

LEDs are attached to quadrature and index outputs as well as magnetic field status outputs.

+ The `A` LED blinks when receiving quadrature input (robot joint is moving)
+ The `I` LED lights when the encoder is at zero position (robot joint at zero)
+ The `MagINC` and `MagDEC` LEDs indicate the quality of readings:

|MagINC|MagDEC|Description
|-|-|-
|`0`|`0`|Distance to magnet is not changing, magnetic field OK
|`0`|`1`|Distance to magnet is increasing
|`1`|`0`|Distance to magnet is decreasing
|`1`|`1`|Magnetic field too low (magnet too far from sensor)

The following components are used to enable this:

+ `SN74LVC2G17DBVR` two-channel Schmitt buffer
+ `BAT54WS` Schottky diodes
+ `470K` resistors (transistor base bleed)
+ `47K` resistors (transistor base resistor)
+ `100K` resistors (envelope discharge)
+ `1uF` capacitors (envelope storage)
+ `100nF` capacitor (buffer decoupling)
+ `150R` and `1K` resistors (LED current limit)
+ `MMBT3904` transistors (LED sink drivers)

The `A` quadrature signal from the encoder indicates activity and is connected in the following network:

|Signal|Pin|
|-|-|
|`A`|AS5045 `A` pin going to Schmitt buffer input
|`A_BUF`|Schmitt buffer output
|`A_EDGE`|Charging capacitor input
|`A_PULSE`|Time-stretched signal
|`A_ENV`|Envelope capacitor
|`A_BLEED`|Bleed resistor
|`A_BASE`|Transistor base
|`BLUE_CATHODE`|Blue LED cathode
|`BLUE_ANODE`|Blue LED anode

The `I` index signal from the encoder indicates zero position and is connected in the following network:

|Signal|Pin|
|-|-|
|`I`|AS5045 `I` pin going to Schmitt buffer input
|`I_BUF`|Schmitt buffer output
|`I_ENV`|Envelope capacitor
|`I_BLEED`|Bleed resistor
|`I_BASE`|Transistor base
|`BLUE_CATHODE`|Blue LED cathode
|`BLUE_ANODE`|Blue LED anode

### Schmitt Buffer

The buffer clamps the analog input to either `HIGH` or `LOW` on the output.

+ AS5045 output (`RAW`) connects to buffer input (`1A`/`2A`)
+ Schmitt buffer output (`1Y`/`2Y`) connects to buffer output (`BUF`)
+ A `100nF` decoupling capacitor from `VCC` to `GND` is placed next to the buffer IC

### Envelope Generators

The envelope generator modifies the transient characteristics of the incoming signal by making it rise quickly (fast attack) but decay slowly (slow release).

Without adding sustain, the LEDs would blink too quickly to be noticeable, and simply end up looking "weakly on".

+ `BAT54` diode: anode connects to `BUF`, cathode to `ENV`
+ `1uF` capacitor between `ENV` and `GND`
+ `100K` resistor between `ENV` and `GND`

### Transistors

Transistors are used to drive LEDs from the power source, using AS5045's A/I pins as on/off switches so they don't drive LEDs directly.

+ Emitter connects to `GND`
+ Collector connects to LED cathode
+ Base connects to `ENV` through `47K` base resistor
+ Base connects to `GND` through `470K` pull-down resistor
+ LED anode connects to `3.3V` through LED resistor (depending on LED color)

### Open-Drain Outputs

The `MagINC` and `MagDEC` pins on AS5045 are open-drain, active-low outputs:

+ When the signals are off, `10K` resistors are used to pull the lines up to `VIN` bus voltage, making the voltage potential across the corresponding LEDs zero and turning them off (both cathode and anode are bridged to `VIN`).
+ When the signals are on (`LOW`), the LED cathodes get bridged to `GND` through these AS5045 pins (and `1K` current-limiting resistors), while LED anodes remain bridged to `VIN` as before, generating a positive voltage potential and turning the LEDs on.

## Connector

The `JST-XH` connector is designed to accomodate a 5-pin `SSI` interface (encoder continuously streams 16-bit words containing the current angle in response to clock input):

|Position|1|2|3|4|5
|-|-|-|-|-|-|
|Signal|`3V3`|`CS`|`SCK`|`MISO`|`GND`|

## Network

The following components appear on the board other than the encoder and the connector:

+ `100nF` (`0.1uF`) to `GND` on `VDD`: decoupling capacitor
+ `VDD` and `VDD3V3` bridged on `VIN`: for 3.3V operation
+ `4.7K` to `GND` on `SCK`: clock pull-down
+ `4.7K` to `VIN` on `CS`: chip select pull-up

## Bill of Materials

|Component|Description|
|-|-|
|[AS5045B-ASST SSOP16 LF T&RDP](https://www.digikey.com/en/products/detail/ams-osram-usa-inc/as5045b-asst-ssop16-lf-t-rdp/4896030)|Hall Effect Encoder|
|[CC0603KRX7R9BB104](https://www.digikey.com/en/products/detail/yageo/cc0603krx7r9bb104/2103082)|`100nF` Decoupling Capacitor|
|[CC0603JRX7R7BB105](https://www.digikey.com/en/products/detail/yageo/cc0603jrx7r7bb105/7164369)|`1uF` Envelope Capacitor|
|[RCG06031K00FKEA](https://www.digikey.com/en/products/detail/vishay-dale/rcg06031k00fkea/4172389)|`1K` Red LED Resistor|
|[CR0603-FX-4701ELF](https://www.digikey.com/en/products/detail/bourns-inc/cr0603-fx-4701elf/3740916)|`4.7K` Pull-Down/Pull-Up Resistor|
|[RC0603FR-0747KL](https://www.digikey.com/en/products/detail/yageo/rc0603fr-0747kl/727253)|`47K` Transistor Base Resistor|
|[CRCW0603100KFKEA](https://www.digikey.com/en/products/detail/vishay-dale/crcw0603100kfkea/1174896)|`100K` Envelope Pull-Down Resistor|
|[RC0603FR-07150RL](https://www.digikey.com/en/products/detail/yageo/rc0603fr-07150rl/726958)|`150R` Blue LED Resistor|
|[RC0603FR-07470KL](https://www.digikey.com/en/products/detail/yageo/rc0603fr-07470kl/727257)|`470K` Envelope Bleed Resistor|
|[CRCW060310K0FKEA](https://www.digikey.com/en/products/detail/vishay-dale/crcw060310k0fkea/1174782)|`10K` Open-Drain Pull-Up Resistors|
|[SN74LVC2G17DBVR](https://www.digikey.com/en/products/detail/umw/sn74lvc2g17dbvr/24890103)|Schmitt Buffer (2-channel)|
|[MMBT3904LT1G](https://www.digikey.com/en/products/detail/onsemi/MMBT3904LT1G/919601)|Transistor|
|[BAT54WS-7-F](https://www.digikey.com/en/products/detail/diodes-incorporated/bat54ws-7-f/804865)|Envelope Diode|
|[NCD0603R1](https://www.lcsc.com/product-detail/C84263.html?s_z=n_NCD0603R1)|Red LED `0603`|
|[150080BS75000](https://www.digikey.com/en/products/detail/w%C3%BCrth-elektronik/150080bs75000/4489912)|Blue LED `0805`|
|[B5B-XH-A](https://www.digikey.com/en/products/detail/jst-sales-america-inc/b5b-xh-a/1530483)|5-pin `JST-XH` connector (2.5mm pitch)|