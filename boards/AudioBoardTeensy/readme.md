# Audio Board

Displays audio waveform and level meter when the robot talks.

## Components

The following components are used:

|Component|Function|
|-|-|
|[Teensy 4.0](https://www.sparkfun.com/teensy-4-0.html)|MCU
|[SSD1306 OLED Display](?)|Audio waveform display
|[Adafruit Seguins](?)|Audio level metering

## Buses

|Bus|Devices
|-|-|
|`SPI`|SSD1306

## Pins

|Pin|Function|
|-|-|
|`A0` | Audio input from Voltage Divider
|`D13`| OLED Display `SCK`
|`D12`| OLED Display `MISO` (?)
|`D11`| OLED Display `MOSI` (?)
|`D0` | OLED Display `CS`
|`D1` | OLED Display Reset
|?|Seguins

## Bill of Materials

|Component|Description|
|-|-|
|[]()|Voltage Divider Resistor `100K`
|[]()|Voltage Bias Capacitor `10uF`
|[]()|Pull-Down Resistor `2K`
|[]()|Current-Limiting Resistor `1K`
|[]()|Audio Jack 3.5mm Stereo
