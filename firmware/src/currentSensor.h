/*
                                                                                     ███████
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████
 currentSensor.h
 ACS37220LEZATR-100B3 Analog Current Sensor: https://www.pololu.com/product/5295
 Copyright (C) 2026 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

#include <Arduino.h>

const float ADC_REFERENCE_VOLTS = 3.3f;
const float ADC_MAX_COUNTS = 4095.0f;
const float ACS37220_ZERO_VOLTAGE = 1.65f;
const float ACS37220_SENSITIVITY_VOLTS_PER_AMP = 0.0132f;

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

class CurrentSensor
{
public:
  int adcPin;
  int reading;

public:
  CurrentSensor():
    adcPin(0),
    reading(0)
  {
  }

  void initialize(int adc)
  {
    adcPin = adc;
    pinMode(adcPin, INPUT);
  }

  float read()
  {
    reading = analogRead(adcPin);
    float voltage = (float(reading) / ADC_MAX_COUNTS) * ADC_REFERENCE_VOLTS;
    return (voltage - ACS37220_ZERO_VOLTAGE) / ACS37220_SENSITIVITY_VOLTS_PER_AMP;
  }

  int raw() const
  {
    return reading;
  }
};
