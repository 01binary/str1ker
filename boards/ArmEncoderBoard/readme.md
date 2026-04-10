# Arm Encoder Board

A custom board that simplifies mounting `AS5045` hall-effect on-axis encoder with 12-bit resolution onto robot joints.

> Ready-made alternatives like [AS5045 Adapter Board](https://www.digikey.com/en/products/detail/ams-osram-usa-inc/as5045-adapterboard/2339623) do exist, but have extra pins not used in this project, and a footprint that makes them hard to mount.

## Power

For `3.3V` operation `VDD5V` and `VDD3V3` are bridged and routed to `VIN` (3.3V). A single `100nF` decoupling capacitor is placed at the bridge (close to the IC) to `GND`.

## Signal

The following is documented in [AS5045 specifications](./doc/AS5045Encoder.pdf):

+ Synchronous Serial Interface ([SSI](https://knowledge.ni.com/KnowledgeArticleDetails?id=kA00Z0000019MgLSAU)) with one-way communication, `MOSI` line not used
+ Compatible with SPI mode 1 (`CPOL` = `0`, `CPHA` = `1`)
+ Transfer begins when `CSn` is `LOW`
+ MSB-first, clocked data output

> The encoder continuously shifts out an 18-bit data frame containing the absolute position and flags in response to clock input without having to request this information - that's what makes its interface *SSI* instead of *SPI*.

In addition to the digital communication interface, AS5045 also exposes these pins:

+ Quadrature `A`, `B` which can take on different values (according to [grey code](https://cool-emerald.blogspot.com/2014/03/reading-rotary-encoder-using.html) specification) as the magnet angle is changing
+ Index `I`, which is signaled when the magnet passes the zero position (typically used to count complete revolutions)
+ Magnet `MagINC`, `MagDEC` pins that indicate magnetic field strength and changing of distance between sensor and magnet (can be used to troubleshoot bad readings or "wobbly" mounting where the magnet changes distance to sensor as it rotates)

## Data Format

The data frame contains 18 bits: 12 data bits, 5 status bits, and 1 parity bit.

| Bit  | Size | Name     | Description
| ---- | ---- | -------- | -------------------------------
| 0–11 | 12   | Position | Angle value (MSB first)
| 12   | 1    | OCF      | Offset Compensation Finished
| 13   | 1    | COF      | CORDIC overflow (invalid angle)
| 14   | 1    | LIN      | Linearity alarm
| 15   | 1    | MagINC   | Magnet too far
| 16   | 1    | MagDEC   | Magnet too close
| 17   | 1    | Parity   | Even parity over previous bits

In practice, it's enough to simply read the first `16` bits using [SPI.transfer16](https://docs.arduino.cc/language-reference/en/functions/communication/SPI/transfer/) function commonly available on Arduino-like microcontrollers such as Teensy.

## Indicators

LEDs are attached to quadrature and index pins as well as magnetic field status pins:

+ The `A` LED blinks when receiving quadrature input (magnet is moving)
+ The `I` LED lights when the encoder is at zero position (magnet is near zero position)
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
|`A_EDGE`|Schmitt buffer output
|`A_PULSE`|RC circuit that turns edges into pulses
|`A_ENV`|Envelope circuit that makes pulses last longer
|`A_BLEED`|Envelope bleed
|`A_BASE`|Transistor base input
|`BLUE_CATHODE`|Blue LED cathode
|`BLUE_ANODE`|Blue LED anode

The `I` index signal from the encoder indicates zero position and is connected in the following network:

|Signal|Pin|
|-|-|
|`I`|AS5045 `I` pin going to Schmitt buffer input
|`I_BUF`|Schmitt buffer output
|`I_ENV`|Envelope circuit that makes pulses last longer
|`I_BLEED`|Envelope bleed
|`I_BASE`|Transistor base input
|`BLUE_CATHODE`|Blue LED cathode
|`BLUE_ANODE`|Blue LED anode

### Schmitt Buffer

The Schmitt Trigger Buffer clamps quadrature signals to either `HIGH` or `LOW`, which *cleans the edges* of these signals, making them a *sharp* square wave instead of a *noisy*, approximate, square wave with *soft* edges.

![schmitt buffer diagram](./doc/schmitt-buffer.png)

This conditioning is needed because the later stages (a differentiator and an integrator) amplify, and therefore are very sensitive to, noise.

### RC Filter (Differentiator)

The differentiator, implemented by using an RC (*Resistor and Capacitor*) circuit, reacts to *changes* in voltage level.

If the input voltage changes from `HIGH` to `LOW`, the output from the differentiator is `HIGH`, same from `LOW` back to `HIGH`. The differentiator stays `LOW` when the voltage is not changing.

![rc integrator](./doc/envelope-integrator.png)

This makes the `A` LED light only when the quadrature signal changes. It's the only way to make an *activity* LED for this kind of signal because a quadrature channel may be left either `HIGH` or `LOW` depending on when the thing being measured stops moving (there is a truth table called the [grey code](https://cool-emerald.blogspot.com/2014/03/reading-rotary-encoder-using.html) that describes the possible states).

![grey code](./doc/grey-code.png)

### Envelope Generator (Integrator)

Envelope generators modify the transient characteristics of the incoming signal. The one used here acts like an integrator (it accumulates a value over time) by making the signal rise quickly (fast attack) but decay slowly (slow release).

> Fast attack/slow release combination creates "sustain" which enables the LED indicators to stay on long enough to be seen as a "blink" by the human eye after the differentiator detects a change (movement).

![envelope generator](./doc/envelope-integrator.png)

Without this part of the circuit, the LEDs would strobe so fast that to a human they would look dimly lit.

### Transistors

Transistors are used to drive LEDs from the power source, using AS5045's A/I pins as on/off switches so they don't drive LEDs directly.

+ Emitter connects to `GND`
+ Collector connects to LED cathode
+ Base connects to `ENV` through `47K` base resistor
+ Base connects to `GND` through `470K` pull-down resistor
+ LED anode connects to `3.3V` through LED resistor (depending on LED color)

### Open-Drain Outputs

The `MagINC` and `MagDEC` pins on AS5045 are open-drain, active-low outputs:

+ When the signals are off (`High-Z`), `10K` resistors are used to pull the lines up to `VIN` bus voltage, making the voltage potential across the corresponding LEDs zero and turning them off (both cathode and anode are bridged to `VIN`).
+ When the signals are on (`LOW`), the LED cathodes get bridged to `GND` through these AS5045 pins (and `1K` current-limiting resistors), while LED anodes remain bridged to `VIN` as before, generating a positive voltage potential and turning the LEDs on.

## Connector

The `JST-XH` connector is designed to accomodate a 5-pin `SSI` interface:

|Position|1|2|3|4|5
|-|-|-|-|-|-|
|Signal|`3V3`|`CS`|`SCK`|`MISO`|`GND`|

This is designed for a simple 5-terminal ribbon with a JST-XH locking connector crimped on.

## Miscellaneous

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