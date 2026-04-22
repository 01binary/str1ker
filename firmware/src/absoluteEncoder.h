/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 absoluteEncoder.h
 AS5045 Absolute Encoder: https://www.digikey.com/en/products/detail/ams-osram-usa-inc/AS5045-ASST/2334769
 Copyright (C) 2025 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <SPI.h>
#include "encoder.h"
#include <ros.h>
#include "params.h"

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

/*
  AS5045 data frame
  [15:4]=position
  [3]=ocf
  [2]=cof
  [1]=lin
  [0]=magInc
*/

class Reading
{
public:
  static const uint16_t MAG_INC_MASK = 0x0001;
  static const uint16_t LIN_MASK = 0x0002;
  static const uint16_t COF_MASK = 0x0004;
  static const uint16_t OCF_MASK = 0x0008;
  static const uint16_t POSITION_MASK = 0xFFF0;
  static const uint8_t POSITION_SHIFT = 4;

public:
  uint16_t data;

  bool magInc() const
  {
    return (data & MAG_INC_MASK) != 0;
  }

  bool lin() const
  {
    return (data & LIN_MASK) != 0;
  }

  bool cof() const
  {
    return (data & COF_MASK) != 0;
  }

  bool ocf() const
  {
    return (data & OCF_MASK) != 0;
  }

  uint16_t position() const
  {
    return (data & POSITION_MASK) >> POSITION_SHIFT;
  }

  bool valid() const
  {
    return ocf() && !cof();
  }
};

class AbsoluteEncoder: public BaseEncoder
{
public:
  static const unsigned int MAX = 4096;
  typedef void (*StatusWriter)(int id, bool value);

public:
  int statusPin;
  int csPin;
  int normMin;
  int normMax;
  float scaleMin;
  float scaleMax;
  bool invert;
  bool initialized;
  bool valid;
  StatusWriter statusWriter;

  SPISettings settings;
  Reading reading;

public:
  AbsoluteEncoder():
    statusPin(0),
    csPin(0),
    normMin(0),
    normMax(MAX),
    scaleMin(0.0),
    scaleMax(1.0),
    invert(false),
    initialized(false),
    valid(false),
    statusWriter(nullptr),
    settings(1e6, MSBFIRST, SPI_MODE0)
  {
  }

  ~AbsoluteEncoder()
  {
    SPI.end();
  }

public:
  void initialize(
    int status,
    int cs,
    int normalizedMin = 0,
    int normalizedMax = MAX,
    float scaledMin = 0.0,
    float scaledMax = 1.0,
    bool invertReadings = false,
    StatusWriter onStatusWrite = nullptr)
  {
    if (initialized)
    {
      SPI.end();
    }

    statusPin = status;
    csPin = cs;
    normMin = normalizedMin;
    normMax = normalizedMax;
    scaleMin = scaledMin;
    scaleMax = scaledMax;
    invert = invertReadings;
    statusWriter = onStatusWrite;
    initialized = true;
    reading.data = 0;

    SPI.begin();

    if (statusWriter == nullptr)
    {
      pinMode(statusPin, OUTPUT);
    }

    pinMode(csPin, OUTPUT);

    digitalWrite(csPin, HIGH);
    writeStatus(false);
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
    SPI.beginTransaction(settings);
    reading.data = SPI.transfer16(0);
    SPI.endTransaction();

    // Deselect
    digitalWrite(csPin, HIGH);

    // Validate
    if (!reading.valid())
    {
      valid = false;
      writeStatus(false);
      return invert ? scaleMax : scaleMin;
    }

    if (!valid)
    {
      valid = true;
      writeStatus(true);
    }

    // Normalize
    float norm = constrain(
      float(reading.position() - normMin) / float(normMax - normMin), 0.0, 1.0);

    // Invert
    if (invert)
    {
      norm = 1.0 - norm;
    }

    // Scale
    return norm * (scaleMax - scaleMin) + scaleMin;
  }

  int raw() const
  {
    return reading.position();
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

private:
  void writeStatus(bool enabled)
  {
    if (statusWriter != nullptr)
    {
      statusWriter(statusPin, enabled);
      return;
    }

    digitalWrite(statusPin, enabled ? HIGH : LOW);
  }
};
