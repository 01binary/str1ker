# Encoder Board

A custom board that simplifies mounting `AS5045` hall-effect on-axis encoder with 12-bit resolution onto DC and stepper motors.

> Ready-made alternatives like [AS5045 Adapter Board](https://www.digikey.com/en/products/detail/ams-osram-usa-inc/as5045-adapterboard/2339623) do exist, but have extra pins not used in this project, and a footprint that makes them hard to mount.

## Power

For `3.3V` operation `VDD5V` and `VDD3V3` are bridged and routed to `VIN` (3.3V). A single `100nF` decoupling capacitor is placed at the bridge (close to the IC) to `GND`.

## Interface

The following is documented in AS5045 specifications:

+ SSI / synchronous serial output (no MOSI)
+ Compatible with SPI mode 1 (`CPOL` = `0`, `CPHA` = `1`)
+ Transfer begins when `CSn` is `LOW`
+ MSB-first, clocked data output

The encoder continuously shifts out a 16-bit data frame in response to clock input.

## Data Format

The 16-bit frame contains a 12-bit position value and status flags:

```c++
union AS5045Reading
{
  struct
  {
    unsigned int position : 12;
    unsigned int status   : 4;
  };

  uint16_t data;

  static const double MAX = 0b111111111111;
};
```

## Connector

The `JST-XH` connector is designed to accomodate a 5-pin `SSI` interface (encoder continuously streams 16-bit words containing the current angle in response to clock input):

|Position|1|2|3|4|5
|-|-|-|-|-|-|
|Signal|`3V3`|`CS`|`SCK`|`MISO`|`GND`|

## Example

The following can be used to initialize the encoder on [Teensy 4.0](https://www.sparkfun.com/teensy-4-0.html):

```c++
#include <Arduino.h>
#include <SPI.h>

const int CS   = 10;
const int SCK  = 13;
const int MISO = 12;

void setup()
{
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);

  SPI.begin();

  Serial.begin(115200);
  while (!Serial && millis() < 2000);
}
```

The following can be used to read the encoder value:

```c++
const long MHz = 1000000;

SPISettings settings(
  2 * MHz, // 2 MHz conservative
  MSBFIRST,
  SPI_MODE1
);

AS5045Reading reading;

void loop()
{
  SPI.beginTransaction(settings);

  digitalWrite(CS, LOW);
  reading.data = SPI.transfer16(0x0000);
  digitalWrite(CS, HIGH);

  SPI.endTransaction();

  delay(10);

  double value = double(reading.position) / AS5045Reading::MAX;
  Serial.println(value, 6);
}
```

## Network

The following components appear on the board other than the encoder and the connector:

+ `100nF` to `GND` on `VDD`: decoupling capacitor
+ `VDD` and `VDD3V3` bridged on `VIN`: for 3.3V operation
+ `4.7K` to `GND` on `SCK`: clock pull-down
+ `4.7K` to `VIN` on `CS`: chip select pull-up

## BOM

|Component|Description|
|-|-|
|[AS5045-ATST](https://www.digikey.com/en/products/detail/ams-osram-usa-inc/as5045-asst/2334769)|Hall Effect Encoder|
|[885342208002](https://www.digikey.com/en/products/detail/w%C3%BCrth-elektronik/885342208002/9345893)|`100nF` Decoupling Capacitor|
|[RE0603FRE074K7L](https://www.digikey.com/en/products/detail/yageo/RE0603FRE074K7L/12708232)|`4.7K` Pull-Down/Pull-Up Resistor|
|[B5B-XH-A](https://www.digikey.com/en/products/detail/jst-sales-america-inc/b5b-xh-a/1530483)|5-pin `JST-XH` connector (2.5mm pitch)|