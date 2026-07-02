/*
                                                                                     ███████
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████
 voltageSensor.h
 Analog Voltage Sensor via Divider
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

class VoltageSensor
{
public:
  static constexpr float ADC_REFERENCE_VOLTS = 3.3f;
  static constexpr float ADC_MAX_COUNTS = 4095.0f;
  static constexpr float DIVIDER_R_HIGH_OHMS = 200000.0f;
  static constexpr float DIVIDER_R_LOW_OHMS = 22000.0f;
  static constexpr float DIVIDER_SCALE =
    (DIVIDER_R_HIGH_OHMS + DIVIDER_R_LOW_OHMS) / DIVIDER_R_LOW_OHMS;
  static constexpr float VOLTAGE_CALIBRATION_QUADRATIC = 0.0f;
  static constexpr float VOLTAGE_CALIBRATION_GAIN = 1.0f;
  static constexpr float VOLTAGE_CALIBRATION_OFFSET_VOLTS = 0.0f;

public:
  int adcPin;
  int reading;

public:
  VoltageSensor():
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
    const float sensedVoltage = (float(reading) / ADC_MAX_COUNTS) * ADC_REFERENCE_VOLTS;
    const float nominalVoltage = sensedVoltage * DIVIDER_SCALE;
    return correctVoltage(nominalVoltage);
  }

  int raw() const
  {
    return reading;
  }

private:
  float correctVoltage(float nominalVoltage) const
  {
    return (VOLTAGE_CALIBRATION_QUADRATIC * nominalVoltage * nominalVoltage) +
           (VOLTAGE_CALIBRATION_GAIN * nominalVoltage) +
           VOLTAGE_CALIBRATION_OFFSET_VOLTS;
  }
};
