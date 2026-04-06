/*
                                                                                     ███████
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████
 potentiometer.h
 Generic analog potentiometer helper
 Copyright (C) 2026 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <Arduino.h>

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

class Potentiometer
{
public:
  static const int RAW_MAX = 4095;

public:
  int adcPin;
  int rawMin;
  int rawMax;
  float scaleMin;
  float scaleMax;
  bool invert;
  int reading;

public:
  Potentiometer():
    adcPin(-1),
    rawMin(0),
    rawMax(RAW_MAX),
    scaleMin(0.0f),
    scaleMax(1.0f),
    invert(false),
    reading(0)
  {
  }

public:
  void initialize(
    int pin,
    int minimum = 0,
    int maximum = RAW_MAX,
    float scaledMin = 0.0f,
    float scaledMax = 1.0f,
    bool invertReadings = false)
  {
    adcPin = pin;
    rawMin = minimum;
    rawMax = maximum;
    scaleMin = scaledMin;
    scaleMax = scaledMax;
    invert = invertReadings;
    reading = 0;

    pinMode(adcPin, INPUT);
  }

  float read()
  {
    reading = analogRead(adcPin);

    int span = rawMax - rawMin;
    if (span == 0)
    {
      return scaleMin;
    }

    float normalized = constrain(
      float(reading - rawMin) / float(span), 0.0f, 1.0f);

    if (invert)
    {
      normalized = 1.0f - normalized;
    }

    return scaleMin + normalized * (scaleMax - scaleMin);
  }

  int raw() const
  {
    return reading;
  }
};
