/*
                                                                                     ███████
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████
 stepperMotor.h
 Step/dir stepper motor driver
 Copyright (C) 2026 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#ifdef ROS
  #include <ros.h>
  #include "params.h"
#endif

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

class StepperMotor
{
public:
  static const unsigned int STEP_PULSE_US = 4;

public:
  int enablePin;
  int stepPin;
  int directionPin;

  bool enabled;
  float stallThreshold;
  bool invert;
  bool stopBeforeReverse;
  float stepRateMin;
  float stepRateMax;
  float velocity;
  float current;
  int rawStatus;
  uint32_t lastUpdateUs;

public:
  StepperMotor():
    enablePin(0),
    stepPin(0),
    directionPin(0),
    enabled(false),
    stallThreshold(1.0f),
    invert(false),
    stopBeforeReverse(false),
    stepRateMin(0.0f),
    stepRateMax(2000.0f),
    velocity(0.0f),
    current(0.0f),
    rawStatus(0),
    lastUpdateUs(0)
  {
  }

  void initialize(
    int enable,
    int step,
    int direction,
    float minRate = 0.0f,
    float maxRate = 2000.0f,
    bool invertCommand = false,
    float stallCurrentThreshold = 1.0f)
  {
    enablePin = enable;
    stepPin = step;
    directionPin = direction;
    stepRateMin = minRate;
    stepRateMax = maxRate;
    invert = invertCommand;
    stallThreshold = stallCurrentThreshold;
    stopBeforeReverse = false;
    velocity = 0.0f;
    current = 0.0f;
    rawStatus = 0;
    lastUpdateUs = micros();

    pinMode(enablePin, OUTPUT);
    pinMode(stepPin, OUTPUT);
    pinMode(directionPin, OUTPUT);

    digitalWrite(stepPin, LOW);
    digitalWrite(directionPin, LOW);
    disable();
  }

  #ifdef ROS
  void loadSettings(ros::NodeHandle& node, const char* group)
  {
    loadBoolParam(node, group, "enabled", enabled);
    loadParam(node, group, "stallThreshold", stallThreshold);
    loadBoolParam(node, group, "stopBeforeReverse", stopBeforeReverse);
    loadBoolParam(node, group, "invert", invert);
    loadParam(node, group, "stepRateMin", stepRateMin);
    loadParam(node, group, "stepRateMax", stepRateMax);

    if (enabled)
    {
      enable();
    }
  }
  #endif

  void enable()
  {
    enabled = true;
    digitalWrite(enablePin, HIGH);
  }

  void disable()
  {
    enabled = false;
    digitalWrite(enablePin, LOW);
    velocity = 0.0f;
  }

  float read()
  {
    current = 0.0f;
    return current;
  }

  int readRaw() const
  {
    return rawStatus;
  }

  void write(float command)
  {
    uint32_t nowUs = micros();

    if (!enabled)
    {
      lastUpdateUs = nowUs;
      return;
    }

    float speed = constrain(abs(command), 0.0f, 1.0f);
    int direction = command >= 0.0f ? 1 : -1;

    if (invert)
    {
      direction *= -1;
    }

    float nextVelocity = direction * speed;
    bool reversedDirection =
      (velocity > 0.0f && nextVelocity < 0.0f) ||
      (velocity < 0.0f && nextVelocity > 0.0f);

    if (stopBeforeReverse && reversedDirection)
    {
      velocity = 0.0f;
      lastUpdateUs = nowUs;
      return;
    }

    if (speed == 0.0f)
    {
      velocity = 0.0f;
      lastUpdateUs = nowUs;
      return;
    }

    digitalWrite(directionPin, direction > 0 ? HIGH : LOW);

    float stepRate = stepRateMin + speed * (stepRateMax - stepRateMin);

    if (stepRate <= 0.0f)
    {
      velocity = nextVelocity;
      lastUpdateUs = nowUs;
      return;
    }

    uint32_t intervalUs = uint32_t(1000000.0f / stepRate);

    if (intervalUs == 0)
    {
      intervalUs = 1;
    }

    uint32_t elapsedUs = nowUs - lastUpdateUs;
    int pulses = int(elapsedUs / intervalUs);

    if (pulses > 64)
    {
      pulses = 64;
    }

    for (int i = 0; i < pulses; ++i)
    {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(STEP_PULSE_US);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(STEP_PULSE_US);
    }

    if (pulses > 0)
    {
      lastUpdateUs += uint32_t(pulses) * intervalUs;
    }
    else
    {
      lastUpdateUs = nowUs;
    }

    velocity = nextVelocity;
  }

  #ifdef ROS
  void debug(ros::NodeHandle& node, const char* group)
  {
    char buffer[256] = {0};
    char minRate_s[16];
    char maxRate_s[16];

    dtostrf(stepRateMin, 0, 2, minRate_s);
    dtostrf(stepRateMax, 0, 2, maxRate_s);

    snprintf(
      buffer,
      sizeof(buffer),
      "%s: stepRateMin=%s stepRateMax=%s %s%s",
      group,
      minRate_s,
      maxRate_s,
      enabled ? "enabled" : "disabled",
      invert ? " invert" : "");
    node.loginfo(buffer);
  }
  #endif
};
