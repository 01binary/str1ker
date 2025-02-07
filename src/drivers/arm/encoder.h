
/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 encoder.h
 Absolute and Quadrature encoder drivers
 Copyright (C) 2025 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <SPI.h>
#include <QuadratureEncoder.h>

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

class Encoder
{
public:
  virtual double read() = 0;
  virtual void readSettings() = 0;
  virtual void writeSettings() = 0;
};

class Potentiometer: Encoder
{
public:
  static const unsigned int MAX = 0b1111111111;

public:
  // Analog pin the potentiometer or encoder is connected to
  int adcPin;
  int normMin;
  int normMax;
  double scaleMin;
  double scaleMax;
  bool invert;

public:
  Potentiometer():
    adcPin(0),
    normMin(0),
    normMax(MAX),
    scaleMin(0.0),
    scaleMax(1.0),
    invert(false)
  {
  }

  void initialize(
    int adc,
    int normalizedMin = 0,
    int normalizedMax = MAX,
    double scaledMin = 0.0,
    double scaledMax = 1.0,
    bool invertReadings = false)
  {
    adcPin = adc;
    normMin = normalizedMin;
    normMax = normalizedMax;
    scaleMin = scaledMin;
    scaleMax = scaledMax;
    invert = invertReadings;

    pinMode(adcPin, INPUT_PULLUP);
  }

  void readSettings()
  {
    normMin = EEPROM.readInt(EEPROM.getAddress(sizeof(int)));
    normMax = EEPROM.readInt(EEPROM.getAddress(sizeof(int)));
    scaleMin = EEPROM.readDouble(EEPROM.getAddress(sizeof(double)));
    scaleMax = EEPROM.readDouble(EEPROM.getAddress(sizeof(double)));
    invert = EEPROM.readBool(EEPROM.getAddress(sizeof(double)));
  }

  void writeSettings()
  {
    EEPROM.writeInt(EEPROM.getAddress(sizeof(int)), normMin);
    EEPROM.writeInt(EEPROM.getAddress(sizeof(int)), normMax);
    EEPROM.writeDouble(EEPROM.getAddress(sizeof(double)), scaleMin);
    EEPROM.writeDouble(EEPROM.getAddress(sizeof(double)), scaleMax);
    EEPROM.writeBool(EEPROM.getAddress(sizeof(double)), invert);
  }

  double read()
  {
    // Convert
    int reading = analogRead(adcPin);

    // Normalize
    double norm = double(reading - normMin) / double(normMax - normMin);

    // Invert
    if (invert)
    {
      norm = 1.0 - norm;
    }

    // Scale
    return norm * (scaleMax - scaleMin) + scaleMin;
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
  static const unsigned int MAX = 0b111111111111;

public:
  int csPin;
  int normMin;
  int normMax;
  double scaleMin;
  double scaleMax;
  bool invert;
  bool initialized;

public:
  AS5045Encoder():
    csPin(0),
    normMin(0),
    normMax(MAX),
    scaleMin(0.0),
    scaleMax(1.0),
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
    int normalizedMin = 0,
    int normalizedMax = MAX,
    double scaledMin = 0.0,
    double scaledMax = 1.0,
    bool invertReadings = false)
  {
    if (initialized)
    {
      SPI.end();
    }

    csPin = cs;
    normMin = normalizedMin;
    normMax = normalizedMax;
    scaleMin = scaledMin;
    scaleMax = scaledMax;
    invert = invertReadings;
    initialized = true;

    SPI.begin();

    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
  }

  void readSettings()
  {
    normMin = EEPROM.readInt(EEPROM.getAddress(sizeof(int)));
    normMax = EEPROM.readInt(EEPROM.getAddress(sizeof(int)));
    scaleMin = EEPROM.readDouble(EEPROM.getAddress(sizeof(double)));
    scaleMax = EEPROM.readDouble(EEPROM.getAddress(sizeof(double)));
    invert = EEPROM.readBool(EEPROM.getAddress(sizeof(double)));
  }

  void writeSettings()
  {
    EEPROM.writeInt(EEPROM.getAddress(sizeof(int)), normMin);
    EEPROM.writeInt(EEPROM.getAddress(sizeof(int)), normMax);
    EEPROM.writeDouble(EEPROM.getAddress(sizeof(double)), scaleMin);
    EEPROM.writeDouble(EEPROM.getAddress(sizeof(double)), scaleMax);
    EEPROM.writeBool(EEPROM.getAddress(sizeof(double)), invert);
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
    double norm = constrain(
      double(reading - normMin) / double(normMax - normMin), 0.0, 1.0);

    // Invert
    if (invert)
    {
      norm = 1.0 - norm;
    }

    // Scale
    return norm * (scaleMax - scaleMin) + scaleMin;
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

  void readSettings()
  {
    invert = EEPROM.readBool(EEPROM.getAddress(sizeof(double)));
  }

  void writeSettings()
  {
    EEPROM.writeBool(EEPROM.getAddress(sizeof(double)), invert);
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

class FusionEncoder: public Encoder
{
public:
  AS5045Encoder absolute;
  QuadratureEncoder quadrature;

public:
  void initialize(
    int cs,
    int a,
    int b)
  {
    absolute.initialize(cs);
    quadrature.initialize(a, b);
  }

  double read()
  {
    // TODO: fuse measurements
    return absolute.read();
  }

  void readSettings()
  {
    absolute.readSettings();
    quadrature.readSettings();
  }

  void writeSettings()
  {
    absolute.writeSettings();
    quadrature.writeSettings();
  }
}
