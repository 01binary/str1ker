
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

#include <ros.h>

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

  float pwmMin;
  float pwmMax;
  float stallThreshold;
  bool invert;

  float velocity;

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
    float min = 0.0,
    float max = 1.0,
    bool invertCommand = false,
    float stallCurrentThreshold = 1.0)
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

  void loadSettings(ros::NodeHandle& node, const char* group)
  {
    node.getParam((String("~") + group + "/pwmMin").c_str(), &pwmMin);
    node.getParam((String("~") + group + "/pwmMax").c_str(), &pwmMax);
    node.getParam((String("~") + group + "/stallThreshold").c_str(), &stallThreshold);
    node.getParam((String("~") + group + "/invert").c_str(), &invert);
  }

  float read()
  {
    return float(analogRead(isPin)) / float(CURRENT_MAX);
  }

  void write(float command)
  {
    float speed = abs(command);
    float direction = command >= 0 ? 1 : -1;

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

    float nextCommand = direction * speed;

    if (nextCommand != velocity)
    {
      int lpwm = direction < 0 ? 0 : int(speed * float(PWM_MAX));
      int rpwm = direction > 0 ? 0 : int(speed * float(PWM_MAX));

      analogWrite(lpwmPin, lpwm);
      analogWrite(rpwmPin, rpwm);

      velocity = nextCommand;
    }
  }
};
