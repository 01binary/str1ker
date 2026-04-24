# Audio Board

Displays audio waveform and level meter when the robot talks.

## Components

The following components are used:

|Component|Function|
|-|-|
|[Teensy 4.0](https://www.sparkfun.com/teensy-4-0.html)|MCU
|[SSD1309 OLED Display](https://www.dfrobot.com/product-2521.html)|Audio waveform display
|[Adafruit Sequins](https://www.adafruit.com/product/1792)|Audio level metering

## Buses

|Bus|Devices
|-|-|
|`SPI`|SSD1309

## Pins

|Pin|Function|
|-|-|
|`A0` | Audio input from Voltage Divider
|`D13`| OLED Display `SCK`
|`D12`| OLED Display `DC` (`MISO`)
|`D11`| OLED Display `MOSI`
|`D0` | OLED Display `CS`
|`D1` | OLED Display Reset
|?|Seguins

## Bill of Materials

|Component|Description|
|-|-|
|[CRCW0603100KFKEA](https://www.digikey.com/en/products/detail/vishay-dale/crcw0603100kfkea/1174896)|Voltage Divider Resistor `100K`
|[CL10A106KP8NNNC](https://www.digikey.com/en/products/detail/samsung-electro-mechanics/cl10a106kp8nnnc/3886850)|Voltage Bias Capacitor `10uF`
|[0603WAF2001T5E](https://www.lcsc.com/product-detail/C22975.html)|Pull-Down Resistor `2K`
|[RCG06031K00FKEA](https://www.digikey.com/en/products/detail/vishay-dale/rcg06031k00fkea/4172389)|Current-Limiting Resistor `1K`
|[]()|Audio Jack 3.5mm Stereo
