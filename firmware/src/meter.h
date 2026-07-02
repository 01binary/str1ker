/*
                                                                                     ███████
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████
 meter.h
 LED Level Meter: https://www.adafruit.com/product/1815
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

class Meter
{
public:
  uint8_t levelCount;
  const int* levelPins;
  float level;

public:
  Meter(uint8_t levels, const int* pins):
    levelCount(levels),
    levelPins(pins),
    level(0.0f)
  {
    for (uint8_t i = 0; i < levelCount; i++)
    {
      pinMode(levelPins[i], OUTPUT);
      digitalWrite(levelPins[i], LOW);
    }
  }

public:
  void write(float normalizedLevel)
  {
    if (!levelCount || !levelPins)
    {
      return;
    }

    level = constrain(normalizedLevel, 0.0f, 1.0f);

    int activeLevels = int(level * float(levelCount));

    if (level >= 1.0f)
    {
      activeLevels = levelCount;
    }

    for (uint8_t i = 0; i < levelCount; i++)
    {
      digitalWrite(levelPins[i], int(i) < activeLevels ? HIGH : LOW);
    }
  }
};
