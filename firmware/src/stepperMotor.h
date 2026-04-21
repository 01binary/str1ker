/*
                                                                                     ███████
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████
 stepperMotor.h
 Step/direction stepper motor driver
 Copyright (C) 2026 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <AccelStepper.h>
#include <ros.h>
#include "params.h"

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

class StepperMotor
{
public:
  static const int DRIVER_INTERFACE = AccelStepper::DRIVER;

public:
  int enablePin;
  int stepPin;
  int directionPin;

  bool enabled;
  bool invert;
  float acceleration;
  float maxSpeed;
  uint16_t minPulseWidthUs;
  float velocity;

public:
  StepperMotor():
    enablePin(0),
    stepPin(0),
    directionPin(0),
    enabled(false),
    invert(false),
    acceleration(2000.0f),
    maxSpeed(2000.0f),
    minPulseWidthUs(2),
    velocity(0.0f),
    driver(nullptr)
  {
  }

  ~StepperMotor()
  {
    if (driver != nullptr)
    {
      delete driver;
      driver = nullptr;
    }
  }

public:
  void initialize(
    int enable,
    int step,
    int direction,
    float accelerationValue = 2000.0f,
    float maxSpeedValue = 2000.0f,
    uint16_t minPulseWidthValueUs = 2,
    bool invertDirection = false)
  {
    if (driver != nullptr)
    {
      delete driver;
      driver = nullptr;
    }

    enablePin = enable;
    stepPin = step;
    directionPin = direction;
    acceleration = accelerationValue;
    maxSpeed = maxSpeedValue;
    minPulseWidthUs = minPulseWidthValueUs;
    invert = invertDirection;
    velocity = 0.0f;

    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin, LOW);

    driver = new AccelStepper(DRIVER_INTERFACE, stepPin, directionPin);
    if (driver == nullptr)
    {
      enabled = false;
      return;
    }

    driver->setEnablePin(enablePin);
    driver->setPinsInverted(invert, false, false);
    driver->setMinPulseWidth(minPulseWidthUs);
    driver->setMaxSpeed(maxSpeed);
    driver->setAcceleration(acceleration);
    driver->setSpeed(0.0f);
    driver->disableOutputs();

    enabled = false;
  }

  void loadSettings(ros::NodeHandle& node, const char* group)
  {
    loadBoolParam(node, group, "enabled", enabled);
    loadBoolParam(node, group, "invert", invert);
    loadParam(node, group, "acceleration", acceleration);
    loadParam(node, group, "maxSpeed", maxSpeed);
    loadParam(node, group, "minPulseWidthUs", minPulseWidthUs);

    if (driver != nullptr)
    {
      driver->setPinsInverted(invert, false, false);
      driver->setMinPulseWidth(minPulseWidthUs);
      driver->setMaxSpeed(maxSpeed);
      driver->setAcceleration(acceleration);
    }

    if (enabled)
    {
      enable();
    }
    else
    {
      disable();
    }
  }

  void enable()
  {
    enabled = true;

    if (driver != nullptr)
    {
      driver->enableOutputs();
    }

    digitalWrite(enablePin, HIGH);
  }

  void disable()
  {
    enabled = false;
    velocity = 0.0f;

    if (driver != nullptr)
    {
      driver->setSpeed(0.0f);
      driver->disableOutputs();
    }

    digitalWrite(enablePin, LOW);
  }

  void write(float command)
  {
    if (!enabled || driver == nullptr)
    {
      return;
    }

    float speed = constrain(command, -1.0f, 1.0f);
    float targetSpeed = speed * maxSpeed;

    driver->setSpeed(targetSpeed);
    driver->runSpeed();
    velocity = speed;
  }

  void debug(ros::NodeHandle& node, const char* group)
  {
    char buffer[256] = {0};
    char acceleration_s[16];
    char maxSpeed_s[16];

    dtostrf(acceleration, 0, 2, acceleration_s);
    dtostrf(maxSpeed, 0, 2, maxSpeed_s);

    snprintf(
      buffer,
      sizeof(buffer),
      "%s: acceleration=%s maxSpeed=%s minPulseWidthUs=%u %s%s",
      group,
      acceleration_s,
      maxSpeed_s,
      minPulseWidthUs,
      enabled ? "enabled" : "disabled",
      invert ? " invert" : "");
    node.loginfo(buffer);
  }

private:
  AccelStepper* driver;
};
