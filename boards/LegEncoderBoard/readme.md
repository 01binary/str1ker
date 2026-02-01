# Leg Encoder Board

A custom board that simplifies mounting `AS5047P` hall-effect on-axis encoder with 14-bit resolution directly on robot wheel hubs.

> Ready-made alternatives like [SideView Tech](https://www.amazon.com/AS5047P-Magnetic-Position-Breakout-Compatible/dp/B0DLJ6XDNM) and [AS5046P Adapter Board](https://www.digikey.com/en/products/detail/ams-osram-usa-inc/as5047p-adapterboard/5452344) do exist, but have extra pins not used in this project and a footprint that makes them hard to mount.

## Power

For 3.3V operation `VDD` and `VDD3V3` are bridged and decoupled to `GND` via `100nF` capacitor.

## Interface

The following is documented in [AS5047P specifications](./doc/AS5047P.pdf):

+ SPI mode 1 (`CPOL` = `0`, `CPHA` = `1`)
+ Transfer begins when `CSn` is `LOW` and `CLK` is `LOW`
+ MSB-first, data protected by parity

The encoder returns a 16-bit data frame:

```c++
struct AS5047Reading
{
  union
  {
    struct
    {
      union
      {
        struct
        {
          unsigned int position : 14;
          unsigned int error: 1;
        };
        struct
        {
          unsigned int value: 15;
        };
      };
      unsigned int parity: 1;
    };
    uint16_t data;
  };

  static const double MAX = 0b11111111111111;

  inline bool isValid()
  {
    this->error || __builtin_popcount(this->value) & 1 != this->parity;
  }
};
```

The commands are also sent in a 16-bit data frame:

```c++
enum AS5047Commands
{
  ANGLE = 0x3FFF;
  NOP = 0;
};

struct AS5047Command
{
  union
  {
    uint16_t data;
    struct
    {
      union
      {
        struct
        {
          unsigned int address: 14;
          unsigned int readWrite: 1;
        };
        unsigned int value: 15; 
      };
      uint16_t parity: 1;
    };
  };

  AS5047Command(uint16_t command)
  {
    this->address = (command & 0x3FFF);
    this->readWrite = 1;
    this->parity = 0;
    this->parity = __builtin_popcount(this->value) % 2;
  }
};
```

> The parity bit is calculated on the first 15 bits of the data frame. This counts how many bits are set in the value - if the number of `1`'s is *even* the parity bit will be `0`, otherwise `1`.

## Connector

The connector is designed to accomodate only absolute output (one row of 5 pins) or an IDE connector with two rows of 5 pins (absolute and incremental output with programming support).

```
|3V3 |CS |SCK |MISO|GND |
|MOSI|A  |B   |I   |GND |
```

## Example

The following can be used to initialize the encoder on [Teensy 4.0](https://www.sparkfun.com/teensy-4-0.html):

```c++
#include <Arduino.h>
#include <SPI.h>

const int CS = 10;
const int SCK = 13;
const int MISO = 12;

void setup()
{
  pinMode(CS, OUTPUT);
  pinMode(SCK, OUTPUT);
  pinMode(MISO, OUTPUT);

  digitalWrite(CS, HIGH);

  SPI.begin();

  Serial.begin(115200);
  while (!Serial && millis() < 2000);
}
```

The following can be used to read the encoder value:

```c++
const long MHz = 1000000;

AS5047Command angleCommand(ANGLE);
AS5047Command nopCommand(NOP);
AS5047Reading reading;

SPISettings settings(
  2 * MHz, // 2 MHz conservative, 4 MHz average, 10 mHz max
  MSBFIRST,
  SPI_MODE1
);

void loop()
{
  SPI.beginTransaction(settings);

  digitalWrite(CS, LOW);
  SPI.transfer16(angleCommand.data);
  reading.data = SPI.transfer16(nopCommand.data);
  digitalWrite(CS, HIGH);

  SPI.endTransaction();

  delay(10);

  if (!reading.isValid())
  {
    Serial.println("error");
    return;
  }

  double value = double(reading.position) / AS5047Reading::MAX;
  Serial.println(value, 6);
}
```

## Network

The following components appear on the board other than the encoder and the connector:

+ `100nF` to `GND` on `VDD`: decoupling capacitor
+ `10nF` to `GND` on `VDD`: bypass capacitor
+ `1uF` to `GDN` on `VDD3V3`: bulk capacitor
+ `VDD` and `VDD3V3` bridged on `VIN`: for 3.3V operation
+ `4.7K` to `GND` on `SCK`: clock pull-down
+ `4.7K` to `VIN` on `CS`: chip select pull-up

## BOM

|Component|Description|
|-|-|
|[AS5047P-ATST](https://www.digikey.com/en/products/detail/ams-osram-usa-inc/as5047p-atst-tssop14-lf-t-rdp/5288535)|Encoder|
|[885342208002](https://www.digikey.com/en/products/detail/w%C3%BCrth-elektronik/885342208002/9345893)|`100nF` Decoupling Capacitor|
|[885012206089](https://www.digikey.com/en/products/detail/w%C3%BCrth-elektronik/885012206089/5453862)|`10nF` Bypass Capacitor|
|[CC0603JRX7R7BB105](https://www.digikey.com/en/products/detail/yageo/CC0603JRX7R7BB105/7164369)|`1uF` Bulk Capacitor|
|[RE0603FRE074K7L](https://www.digikey.com/en/products/detail/yageo/RE0603FRE074K7L/12708232)|`4.7K` Pull-Down/Pull-Up Resistor|
|PinHeader_2x05_P2.54mm|Standard KiCad IDC Connector|