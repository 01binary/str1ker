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
#include "params.h"

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

class Encoder
{
public:
  virtual float read() = 0;
  virtual void loadSettings(ros::NodeHandle& node, const char* group) = 0;
};

class Potentiometer: public Encoder
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
    loadParam(node, group, "normMin", normMin);
    loadParam(node, group, "normMax", normMax);
    loadParam(node, group, "scaleMin", scaleMin);
    loadParam(node, group, "scaleMax", scaleMax);
    loadBoolParam(node, group, "invert", invert);
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

union AS5045Reading
{
  struct
  {
    uint16_t status   : 4;
    uint16_t position : 12;
  };

  uint16_t data;
};

class AS5045Encoder: public Encoder
{
public:
  static const unsigned int MAX = 4096;

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
    loadParam(node, group, "normMin", normMin);
    loadParam(node, group, "normMax", normMax);
    loadParam(node, group, "scaleMin", scaleMin);
    loadParam(node, group, "scaleMax", scaleMax);
    loadBoolParam(node, group, "invert", invert);
  }

  float read()
  {
    // Select
    digitalWrite(csPin, LOW);

    // Read
    SPI.beginTransaction(SPISettings(
      1e6,
      MSBFIRST,
      SPI_MODE0
    ));

    AS5045Reading reading;
    reading.data = SPI.transfer16(0);

    SPI.endTransaction();

    // Deselect
    digitalWrite(csPin, HIGH);

    // Normalize
    float norm = constrain(
      float(reading.position - normMin) / float(normMax - normMin), 0.0, 1.0);

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
    loadBoolParam(node, group, "invert", invert);
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
