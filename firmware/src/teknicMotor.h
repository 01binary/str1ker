/*
                                                                                     ███████
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████
 teknicMotor.h
 Teknic Motor Driver: https://teknic.com/model-info/CPM-MCVC-3441S-RLN
 Copyright (C) 2026 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros.h>
#include "params.h"

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

class TeknicMotor
{
public:
  static const unsigned int PWM_MAX = 255;
  static const unsigned int PWM_FREQ = 20000;

public:
  int enablePin;
  int directionPin;
  int torquePin;
  int statusPin;

  bool enabled;
  int pwmMin;
  int pwmMax;
  bool invert;

  float velocity;

public:
  TeknicMotor():
    enablePin(0),
    directionPin(0),
    torquePin(0),
    statusPin(-1),
    enabled(false),
    pwmMin(0),
    pwmMax(PWM_MAX),
    invert(false),
    velocity(0.0f)
  {
  }

  void initialize(
    int enable,
    int direction,
    int torque,
    int status,
    int min = 0,
    int max = PWM_MAX,
    bool invertCommand = false)
  {
    enablePin = enable;
    directionPin = direction;
    torquePin = torque;
    statusPin = status;
    pwmMin = min;
    pwmMax = max;
    invert = invertCommand;
    velocity = 0.0f;

    pinMode(enablePin, OUTPUT);
    pinMode(directionPin, OUTPUT);
    pinMode(torquePin, OUTPUT);
    pinMode(statusPin, INPUT);

    analogWriteFrequency(torquePin, PWM_FREQ);
    disable();
  }

  void loadSettings(ros::NodeHandle& node, const char* group)
  {
    loadBoolParam(node, group, "enabled", enabled);
    loadParam(node, group, "pwmMin", pwmMin);
    loadParam(node, group, "pwmMax", pwmMax);
    loadBoolParam(node, group, "invert", invert);

    if (enabled)
    {
      enable();
    }
  }

  void enable()
  {
    enabled = true;
    digitalWrite(enablePin, HIGH);
  }

  void disable()
  {
    enabled = false;
    digitalWrite(enablePin, LOW);
    analogWrite(torquePin, 0);
    velocity = 0.0f;
  }

  int read() const
  {
    if (statusPin < 0)
    {
      return 0;
    }

    return digitalRead(statusPin);
  }

  void write(float command)
  {
    if (!enabled)
    {
      return;
    }

    float speed = constrain(abs(command), 0.0f, 1.0f);
    int direction = command >= 0.0f ? 1 : -1;

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
    float nextVelocity = direction * limitedSpeed;

    digitalWrite(directionPin, direction > 0 ? HIGH : LOW);
    analogWrite(torquePin, pwm);

    velocity = nextVelocity;
  }

  void debug(ros::NodeHandle& node, const char* group)
  {
    char buffer[256] = {0};
    snprintf(
      buffer,
      sizeof(buffer),
      "%s: pwmMin=%d pwmMax=%d %s%s",
      group,
      pwmMin,
      pwmMax,
      enabled ? "enabled" : "disabled",
      invert ? " invert" : "");
    node.loginfo(buffer);
  }
};
