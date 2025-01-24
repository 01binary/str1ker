/*
    encoder.h
    Gets absolute encoder readings.
    Supports SSI/SPI and analog/PWM interfaces.
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <SPI.h>
#include <QuadratureEncoder.h>

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const double ADC_MAX = double(0b1111111111);
const double AS5045_MAX = double(0b111111111111);

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

class Encoder
{
public:
  virtual double read() = 0;
};

class Potentiometer: Encoder
{
public:
  // Analog pin the potentiometer or encoder is connected to
  int adcPin;
  double min;
  double max;
  bool invert;

public:
  Potentiometer():
    adcPin(0),
    min(0.0),
    max(1.0),
    invert(false)
  {
  }

  void initialize(
    int adc,
    double scaleMin = 0.0,
    double scaleMax = 1.0,
    bool invertReadings = false)
  {
    adcPin = adc;
    min = scaleMin;
    max = scaleMax;
    invert = invertReadings;

    pinMode(adcPin, INPUT_PULLUP);
  }

  double read()
  {
    // Convert
    int reading = analogRead(adcPin);

    // Normalize
    double norm = double(reading) / ADC_MAX;

    // Invert
    if (invert)
    {
      norm = 1.0 - norm;
    }

    // Scale
    return norm * (max - min) + min;
  }
};

// AS5045 (SPI)
// ------------
// GND (black)
// 5V (red)
// SS (green)
// SCK (yellow)
// MISO (white)
// MOSI (orange)

class AS5045Encoder: Encoder
{
public:
  int csPin;
  int clockPin;
  int misoPin;
  int mosiPin;
  double min;
  double max;
  bool invert;
  bool initialized;

public:
  AS5045Encoder():
    csPin(0),
    clockPin(0),
    misoPin(0),
    mosiPin(0),
    min(0.0),
    max(1.0),
    invert(false),
    initialized(false)
  {
  }

  ~AS5045Encoder()
  {
    SPI.end();
  }

public:
  void initialize(
    int cs,
    int clock,
    int miso,
    int mosi,
    double scaleMin = 0.0,
    double scaleMax = 1.0,
    bool invertReadings = false)
  {
    if (initialized)
    {
      SPI.end();
    }

    csPin = cs;
    clockPin = clock;
    misoPin = miso;
    mosiPin = mosi;
    min = scaleMin;
    max = scaleMax;
    invert = invertReadings;
    initialized = true;

    SPI.begin();

    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
  }

  double read()
  {
    // Select
    digitalWrite(csPin, LOW);
    delayMicroseconds(1);

    // Read
    SPI.beginTransaction(SPISettings(
      1e6,
      MSBFIRST,
      SPI_MODE0
    ));

    unsigned int raw = SPI.transfer16(0);

    SPI.endTransaction();

    // Deselect
    digitalWrite(csPin, HIGH);

    // Convert
    unsigned int reading = (raw >> 3) & 0x1FFF;

    // Normalize
    double norm = double(reading) / AS5045_MAX;

    // Scale
    return norm * (max - min) + min;
  }
};

class QuadratureEncoder
{
public:
  Encoders* pQuadrature;
  bool invert;

  int count;
  int lastCount;

public:
  QuadratureEncoder():
    pQuadrature(nullptr),
    invert(false),
    count(0),
    lastCount(0)
  {
  }

public:
  void initialize(int a, int b, bool invertCount = false)
  {
    delete pQuadrature;
    pQuadrature = new Encoders(a, b);

    invert = invertCount;
    count = 0;
    lastCount = 0;
  }

  int read()
  {
    count = pQuadrature
      ? pQuadrature->getEncoderCount()
      : 0;

    int diff = count - lastCount;
    lastCount = count;

    return diff;
  }
}
