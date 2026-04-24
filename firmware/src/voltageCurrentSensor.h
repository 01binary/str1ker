/*
                                                                                     ███████
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████
 voltageCurrentSensor.h
 INA260 Digital Voltage/Current Sensor: https://learn.adafruit.com/adafruit-ina260-current-voltage-power-sensor-breakout
 Copyright (C) 2026 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <Adafruit_INA260.h>

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

class VoltageCurrentSensor
{
public:
  static constexpr float MA_TO_A = 0.001f;
  static constexpr float MV_TO_V = 0.001f;

public:
  Adafruit_INA260 sensor;
  bool initialized;

public:
  VoltageCurrentSensor():
    sensor(),
    initialized(false)
  {
  }

  bool initialize()
  {
    initialized = sensor.begin();
    return initialized;
  }

  float readVoltage()
  {
    if (!initialized)
    {
      return 0.0f;
    }

    return sensor.readBusVoltage() * MV_TO_V;
  }

  float readCurrent()
  {
    if (!initialized)
    {
      return 0.0f;
    }

    return sensor.readCurrent() * MA_TO_A;
  }
};
