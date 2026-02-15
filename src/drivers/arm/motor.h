
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

  int pwmMin;
  int pwmMax;
  float stallThreshold;
  bool invert;

  float velocity;
  float current;

public:
  Motor():
    lpwmPin(0),
    rpwmPin(0),
    isPin(0),
    pwmMin(0),
    pwmMax(PWM_MAX),
    invert(false),
    stallThreshold(1.0),
    velocity(0.0),
    current(0.0)
  {
  }

public:
  void initialize(
    int lpwm,
    int rpwm,
    int is,
    int min = 0,
    int max = PWM_MAX,
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
    current = 0.0;

    pinMode(lpwmPin, OUTPUT);
    pinMode(rpwmPin, OUTPUT);
  }

  void loadSettings(ros::NodeHandle& node, const char* group)
  {
    node.getParam((String("~") + group + "/pwmMin").c_str(), &pwmMin);
    node.getParam((String("~") + group + "/pwmMax").c_str(), &pwmMax);
    node.getParam((String("~") + group + "/stallThreshold").c_str(), &stallThreshold);

    if (pwmMin < 0 || pwmMin > int(PWM_MAX))
    {
      char buffer[100] = {0};
      snprintf(buffer, sizeof(buffer), "%s/pwmMin out of range [0..%u]: %d",
        group, PWM_MAX, pwmMin);
      node.logwarn(buffer);
    }

    if (pwmMax < 0 || pwmMax > int(PWM_MAX))
    {
      char buffer[100] = {0};
      snprintf(buffer, sizeof(buffer), "%s/pwmMax out of range [0..%u]: %d",
        group, PWM_MAX, pwmMax);
      node.logwarn(buffer);
    }

    if (pwmMin > pwmMax)
    {
      char buffer[100] = {0};
      snprintf(buffer, sizeof(buffer), "%s has pwmMin > pwmMax (%d > %d)",
        group, pwmMin, pwmMax);
      node.logwarn(buffer);
    }

    if (stallThreshold < 0.0f || stallThreshold > 1.0f)
    {
      char buffer[100] = {0};
      char threshold_s[16];
      dtostrf(stallThreshold, 0, 4, threshold_s);
      snprintf(buffer, sizeof(buffer), "%s/stallThreshold out of range [0..1]: %s",
        group, threshold_s);
      node.logwarn(buffer);
    }

    int invert_i = 0;
    node.getParam((String("~") + group + "/invert").c_str(), &invert_i);
    invert = invert_i;
  }

  float read()
  {
    current = float(analogRead(isPin)) / float(CURRENT_MAX);
    return current;
  }

  void write(float command)
  {
    float speed = constrain(abs(command), 0.0f, 1.0f);
    float direction = command >= 0 ? 1 : -1;

    if (invert)
    {
      direction *= -1;
    }

    int pwm = int(speed * float(PWM_MAX));

    if (pwm != 0)
    {
      if (pwm < pwmMin)
      {
        pwm = pwmMin;
      }
      else if (pwm > pwmMax)
      {
        pwm = pwmMax;
      }
    }

    float limitedSpeed = float(pwm) / float(PWM_MAX);
    float nextCommand = direction * limitedSpeed;

    if (nextCommand != velocity)
    {
      int lpwm = direction < 0 ? 0 : pwm;
      int rpwm = direction > 0 ? 0 : pwm;

      analogWrite(lpwmPin, lpwm);
      analogWrite(rpwmPin, rpwm);

      velocity = direction * limitedSpeed;
    }
  }

  void debug(ros::NodeHandle& node, const char* group)
  {
    char buffer[256] = {0};
    char thresh_s[16], current_s[16];

    dtostrf(stallThreshold, 0, 4, thresh_s);
    dtostrf(current, 0, 4, current_s);

    snprintf(buffer, sizeof(buffer),
      "%s: pwmMin=%d pwmMax=%d stallThreshold=%s current=%s%s",
      group, pwmMin, pwmMax, thresh_s, current_s, invert ? " invert" : "");

    node.loginfo(buffer);
  }
};
