/*
                                                                                     ███████
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████
 currentSensor.h
 Analog current sensor
 Copyright (C) 2026 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

#ifdef ROS
  #include <ros.h>
  #include "params.h"
#endif

const float ADC_REFERENCE_VOLTS = 3.3f;
const float ADC_MAX_COUNTS = 4095.0f;

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

class CurrentSensor
{
public:
  int adcPin;
  bool enabled;
  float zeroVoltage;
  float ampsPerVolt;
  bool absoluteValue;
  int reading;

public:
  CurrentSensor():
    adcPin(0),
    enabled(false),
    zeroVoltage(ADC_REFERENCE_VOLTS * 0.5f),
    ampsPerVolt(0.0f),
    absoluteValue(true),
    reading(0)
  {
  }

  void initialize(int adc)
  {
    adcPin = adc;
    pinMode(adcPin, INPUT_PULLUP);
  }

  #ifdef ROS
  void loadSettings(ros::NodeHandle& node, const char* group)
  {
    loadBoolParam(node, group, "enabled", enabled);
    loadParam(node, group, "zeroVoltage", zeroVoltage);
    loadParam(node, group, "ampsPerVolt", ampsPerVolt);
    loadBoolParam(node, group, "absoluteValue", absoluteValue);
  }
  #endif

  float read()
  {
    if (!enabled || ampsPerVolt == 0.0f)
    {
      reading = 0;
      return 0.0f;
    }

    reading = analogRead(adcPin);

    float voltage = float(reading) * ADC_REFERENCE_VOLTS / ADC_MAX_COUNTS;
    float current = (voltage - zeroVoltage) * ampsPerVolt;

    if (absoluteValue)
    {
      current = abs(current);
    }

    return current;
  }

  int raw() const
  {
    return reading;
  }
};
