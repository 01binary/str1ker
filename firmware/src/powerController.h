/*
                                                                                     ███████
 ████████████  ████████████   ███████████████       █  █████████████  █           █  ███       ███  ████████████
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████
 powerController.h
 Power Display and Battery Meter Controller
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

class PowerController
{
public:
  static constexpr unsigned long UPDATE_INTERVAL = 100;
  static constexpr float BATTERY_FULL_VOLTS = 29.2f;
  static constexpr float BATTERY_EMPTY_VOLTS = 20.0f;
  static constexpr float CHARGE_SMOOTHING_ALPHA = 0.15f;

public:
  CurrentSensor& currentSensor;
  VoltageCurrentSensor& voltageSensor;
  Meter& batteryMeter;
  PowerDisplay& display;
  unsigned long lastUpdateMs;
  float voltage;
  float current;
  float batteryCharge;

public:
  PowerController(
    CurrentSensor& busCurrent,
    VoltageCurrentSensor& busVoltage,
    Meter& batteryLevelMeter,
    PowerDisplay& powerLevelDisplay):
    currentSensor(busCurrent),
    voltageSensor(busVoltage),
    batteryMeter(batteryLevelMeter),
    display(powerLevelDisplay),
    lastUpdateMs(0),
    voltage(0.0f),
    current(0.0f),
    batteryCharge(0.0f)
  {
  }

  void update()
  {
    unsigned long now = millis();

    if (lastUpdateMs != 0 && now - lastUpdateMs < UPDATE_INTERVAL)
    {
      return;
    }

    lastUpdateMs = now;
    voltage = voltageSensor.readVoltage();
    current = currentSensor.readCurrent();

    float measuredCharge = estimateBatteryCharge(voltage);

    if (batteryCharge <= 0.0f)
    {
      batteryCharge = measuredCharge;
    }
    else
    {
      batteryCharge += (measuredCharge - batteryCharge) * CHARGE_SMOOTHING_ALPHA;
    }

    batteryMeter.write(batteryCharge);
    display.update(voltage, current);
  }

  float readBatteryCharge() const
  {
    return batteryCharge;
  }

private:
  float estimateBatteryCharge(float packVoltage) const
  {
    return constrain(
      (packVoltage - BATTERY_EMPTY_VOLTS) / (BATTERY_FULL_VOLTS - BATTERY_EMPTY_VOLTS),
      0.0f,
      1.0f
    );
  }
};
