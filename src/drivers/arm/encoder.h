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
#include <ros.h>

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

class Encoder
{
public:
  virtual float read() = 0;
  virtual void loadSettings(ros::NodeHandle& node, const char* group) = 0;
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
  float scaleMin;
  float scaleMax;
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
    float scaledMin = 0.0,
    float scaledMax = 1.0,
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

  void loadSettings(ros::NodeHandle& node, const char* group)
  {
    node.getParam((String("~") + group + "/normMin").c_str(), &normMin);
    node.getParam((String("~") + group + "/normMax").c_str(), &normMax);
    node.getParam((String("~") + group + "/scaleMin").c_str(), &scaleMin);
    node.getParam((String("~") + group + "/scaleMax").c_str(), &scaleMax);

    int invert_i = 0;
    node.getParam((String("~") + group + "/invert").c_str(), &invert_i);
    
  }

  float read()
  {
    // Convert
    int reading = analogRead(adcPin);

    // Normalize
    float norm = float(reading - normMin) / float(normMax - normMin);

    // Invert
    if (invert)
    {
      norm = 1.0 - norm;
    }

    // Scale
    return norm * (scaleMax - scaleMin) + scaleMin;
  }

  void debug(ros::NodeHandle& node, const char* group)
  {
    char buffer[256] = {0};
    char scaleMin_s[16], scaleMax_s[16];

    dtostrf(scaleMin, 0, 4, scaleMin_s);
    dtostrf(scaleMax, 0, 4, scaleMax_s);

    sprintf(
      buffer,
      "%s: normMin=%d normMax=%d scaleMin=%s scaleMax=%s%s",
      group, normMin, normMax, scaleMin_s, scaleMax_s, invert ? " invert" : ""
    );

    node.loginfo(buffer);
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
  float scaleMin;
  float scaleMax;
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
    float scaledMin = 0.0,
    float scaledMax = 1.0,
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

  void loadSettings(ros::NodeHandle& node, const char* group)
  {
    node.getParam((String("~") + group + "/normMin").c_str(), &normMin);
    node.getParam((String("~") + group + "/normMax").c_str(), &normMax);
    node.getParam((String("~") + group + "/scaleMin").c_str(), &scaleMin);
    node.getParam((String("~") + group + "/scaleMax").c_str(), &scaleMax);

    int invert_i = 0;
    node.getParam((String("~") + group + "/invert").c_str(), &invert_i);
    invert = invert_i;
  }

  float read()
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
    float norm = constrain(
      float(reading - normMin) / float(normMax - normMin), 0.0, 1.0);

    // Invert
    if (invert)
    {
      norm = 1.0 - norm;
    }

    // Scale
    return norm * (scaleMax - scaleMin) + scaleMin;
  }

  void debug(ros::NodeHandle& node, const char* group)
  {
    char buffer[256] = {0};
    char scaleMin_s[16], scaleMax_s[16];

    dtostrf(scaleMin, 0, 4, scaleMin_s);
    dtostrf(scaleMax, 0, 4, scaleMax_s);

    sprintf(
      buffer,
      "%s: normMin=%d normMax=%d scaleMin=%s scaleMax=%s%s",
      group, normMin, normMax, scaleMin_s, scaleMax_s, invert ? " invert" : ""
    );

    node.loginfo(buffer);
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

  void loadSettings(ros::NodeHandle& node, const char* group)
  {
    int invert_i = 0;
    node.getParam((String("~") + group + "/invert").c_str(), &invert_i);
    invert = invert_i;
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

  void debug(ros::NodeHandle& node, const char* group)
  {
    if (!invert) return;

    char buffer[100] = {0};

    sprintf(
      buffer,
      "%s: %s%s",
      group, invert ? " invert" : ""
    );

    node.loginfo(buffer);
  }
};

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

  float read()
  {
    // TODO: fuse measurements
    return absolute.read();
  }

  void loadSettings(ros::NodeHandle& node, const char* group)
  {
    absolute.loadSettings(node, group);
    quadrature.loadSettings(node, group);
  }

  void debug(ros::NodeHandle& node, const char* group)
  {
    absolute.debug(node, group);
    quadrature.debug(node, group);
  }
};
