/*
                                                                                     ███████
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████
 pca9685_motor.h
 Bidirectional motor output using PCA9685 + enable pin
 Copyright (C) 2026 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <Adafruit_PWMServoDriver.h>

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

class MuxMotor
{
public:
  static const uint16_t PWM_MAX = 4095;

public:
  Adafruit_PWMServoDriver& pwm;
  int enPin;
  uint8_t lpwmChannel;
  uint8_t rpwmChannel;
  uint16_t pwmMin;
  uint16_t pwmMax;
  bool invert;
  bool enabled;
  float command;

public:
  explicit MuxMotor(Adafruit_PWMServoDriver& driver):
    pwm(driver),
    enPin(-1),
    lpwmChannel(0),
    rpwmChannel(0),
    pwmMin(0),
    pwmMax(PWM_MAX),
    invert(false),
    enabled(false),
    command(0.0f)
  {
  }

public:
  void initialize(
    int enablePin,
    uint8_t leftChannel,
    uint8_t rightChannel,
    uint16_t minimum = 0,
    uint16_t maximum = PWM_MAX,
    bool invertCommand = false)
  {
    enPin = enablePin;
    lpwmChannel = leftChannel;
    rpwmChannel = rightChannel;
    pwmMin = minimum;
    pwmMax = maximum;
    invert = invertCommand;
    command = 0.0f;

    pinMode(enPin, OUTPUT);
    digitalWrite(enPin, LOW);

    disable();
  }

  void enable()
  {
    enabled = true;
    digitalWrite(enPin, HIGH);
  }

  void disable()
  {
    enabled = false;
    digitalWrite(enPin, LOW);
    setChannels(0, 0);
    command = 0.0f;
  }

  void stop()
  {
    setChannels(0, 0);
    command = 0.0f;
  }

  void write(float nextCommand)
  {
    if (!enabled)
    {
      stop();
      return;
    }

    float limited = constrain(nextCommand, -1.0f, 1.0f);

    if (invert)
    {
      limited *= -1.0f;
    }

    float magnitude = fabsf(limited);
    uint16_t pwmValue = 0;

    if (magnitude > 0.0f)
    {
      pwmValue = uint16_t(magnitude * float(PWM_MAX));

      if (pwmValue < pwmMin)
      {
        pwmValue = pwmMin;
      }
      else if (pwmValue > pwmMax)
      {
        pwmValue = pwmMax;
      }
    }

    uint16_t left = limited < 0.0f ? pwmValue : 0;
    uint16_t right = limited > 0.0f ? pwmValue : 0;

    setChannels(left, right);
    command = limited;
  }

private:
  void setChannels(uint16_t left, uint16_t right)
  {
    pwm.setPWM(lpwmChannel, 0, left);
    pwm.setPWM(rpwmChannel, 0, right);
  }
};
