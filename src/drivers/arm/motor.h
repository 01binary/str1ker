
/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 motor.h
 Motor PWM driver
 Copyright (C) 2025 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include "reconfigure.h"

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

class Motor
{
public:
  static const unsigned int CURRENT_MAX = 0b1111111111;
  static const unsigned int PWM_MAX = 0b11111111;

public:
  int lpwmPin;
  int rpwmPin;
  int isPin;

  double pwmMin;
  double pwmMax;
  double stallThreshold;
  bool invert;

  double velocity;

public:
  Motor():
    lpwmPin(0),
    rpwmPin(0),
    isPin(0),
    pwmMin(0.0),
    pwmMax(1.0),
    invert(false),
    stallThreshold(1.0),
    velocity(0.0)
  {
  }

public:
  void initialize(
    int lpwm,
    int rpwm,
    int is,
    double min = 0.0,
    double max = 1.0,
    bool invertCommand = false,
    double stallCurrentThreshold = 1.0)
  {
    lpwmPin = lpwm;
    rpwmPin = rpwm;
    isPin = is;
    pwmMin = min;
    pwmMax = max;
    invert = invertCommand;
    stallThreshold = stallCurrentThreshold;
    velocity = 0.0;

    pinMode(lpwmPin, OUTPUT);
    pinMode(rpwmPin, OUTPUT);
  }

  void readSettings(Group& group)
  {
    pwmMin = EEPROM.readDouble(EEPROM.getAddress(sizeof(double)));
    pwmMax = EEPROM.readDouble(EEPROM.getAddress(sizeof(double)));
    stallThreshold = EEPROM.readDouble(EEPROM.getAddress(sizeof(double)));
    invert = EEPROM.readInt(EEPROM.getAddress(sizeof(bool)));

    group
      .describe("pwmMin", &pwmMin, 0, PWM_MAX, "Max PWM pulse")
      .describe("pwmMax", &pwmMax, 0, PWM_MAX, "Max PWM pulse")
      .describe("stallThreshold", &stallThreshold, 0, 1000.0, "Stall current")
      .describe("invert", &invert, "Invert PWM pulse");
  }

  void writeSettings()
  {
    EEPROM.writeDouble(EEPROM.getAddress(sizeof(double)), pwmMin);
    EEPROM.writeDouble(EEPROM.getAddress(sizeof(double)), pwmMax);
    EEPROM.writeDouble(EEPROM.getAddress(sizeof(double)), stallThreshold);
    EEPROM.writeInt(EEPROM.getAddress(sizeof(bool)), invert);
  }

  double read()
  {
    return double(analogRead(isPin)) / double(CURRENT_MAX);
  }

  void write(double command)
  {
    double speed = abs(command);
    double direction = command >= 0 ? 1 : -1;

    if (invert)
    {
      direction *= -1;
    }

    if (speed != 0)
    {
      if (speed < pwmMin)
      {
        speed = pwmMin;
      }
      else if (speed > pwmMax)
      {
        speed = pwmMax;
      }
    }

    double nextCommand = direction * speed;

    if (nextCommand != velocity)
    {
      int lpwm = direction < 0 ? 0 : int(speed * double(PWM_MAX));
      int rpwm = direction > 0 ? 0 : int(speed * double(PWM_MAX));

      analogWrite(lpwmPin, lpwm);
      analogWrite(rpwmPin, rpwm);

      velocity = nextCommand;
    }
  }
};
