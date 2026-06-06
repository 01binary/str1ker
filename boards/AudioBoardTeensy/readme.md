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
|`D2` | LED 1
|`D3` | LED 2
|`D4` | LED 3
|`D5` | LED 4
|`D6` | LED 5

Only a single pin from the TRRS audio jack (stereo audio + mono mic) is used as the audio signal for this board:

|Jack Pin|Description|Function
|-|-|-|
|`4`|Tip|Left audio channel output to Voltage Divider
|`3`|Ring 1|Right audio channel (unconnected)
|`2`|Ring 2|`GND`
|`1`|Sleeve|Mic (unconnected)

> This audio jack was used because it's more commonly in stock by various manufacturers than mono audio jacks or stereo audio jacks with no mic.

## Bill of Materials

|Component|Description|
|-|-|
|[CRCW0603100KFKEA](https://www.digikey.com/en/products/detail/vishay-dale/crcw0603100kfkea/1174896)|Voltage Divider Resistor `100K`
|[CL10A106KP8NNNC](https://www.digikey.com/en/products/detail/samsung-electro-mechanics/cl10a106kp8nnnc/3886850)|Voltage Bias Capacitor `10uF`
|[0603WAF2001T5E](https://www.lcsc.com/product-detail/C22975.html)|Pull-Down Resistor `2K`
|[RCG06031K00FKEA](https://www.digikey.com/en/products/detail/vishay-dale/rcg06031k00fkea/4172389)|Current-Limiting Resistor `1K`
|[54-00174](https://www.digikey.com/en/products/detail/tensility-international-corp/54-00174/12140150)|Audio Jack 3.5mm TRRS (2 Stereo channels and Microphone mono channel which is unused)
|[1792](https://www.digikey.com/en/products/detail/adafruit-industries-llc/1792/5774409)|Adafruit LED Sequins Pink
|[Teensy 4.0](https://www.sparkfun.com/teensy-4-0-headers.html)|Teensy 4.0 with Headers
|[Millmax 0531-0-15-15-31-27-10-0](https://www.digikey.com/en/products/detail/mill-max-manufacturing-corp/0531-0-15-15-31-27-10-0/4879975)|Pin receptacle connector for side Teensy pins|