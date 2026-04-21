
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
#include "params.h"

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

class Motor
{
public:
  // Teensy ADC and current-sense conversion constants
  static constexpr float ADC_REFERENCE_VOLTS = 3.3f;
  static constexpr float ADC_MAX_COUNTS = 4095.0f;

  // BTS7960/IBT-2 current sense:
  // I_load = V_is * (kILIS / R_is)
  // kILIS is typically ~8500 (device-dependent spread in datasheet)
  static constexpr float CURRENT_SENSE_RESISTOR_OHMS = 5100.0f;
  static constexpr float CURRENT_SENSE_KILIS = 8500.0f;
  static constexpr float CURRENT_SENSE_AMPS_PER_VOLT =
    CURRENT_SENSE_KILIS / CURRENT_SENSE_RESISTOR_OHMS;

  static const unsigned int PWM_MAX = 255;
  static const unsigned int PWM_FREQ = 20000;
  typedef void (*PwmWriter)(int channelId, int pwmValue);

public:
  int lpwmPin;
  int rpwmPin;
  int isPin;
  int enPin;

  bool enabled;
  int pwmMin;
  int pwmMax;
  float stallThreshold;
  bool invert;
  bool stopBeforeReverse;

  float velocity;
  float current;
  int rawCurrent;
  PwmWriter pwmWriter;

public:
  Motor():
    lpwmPin(0),
    rpwmPin(0),
    isPin(0),
    enPin(0),
    enabled(false),
    pwmMin(0),
    pwmMax(PWM_MAX),
    stallThreshold(1.0),
    invert(false),
    stopBeforeReverse(false),
    velocity(0.0),
    current(0.0),
    rawCurrent(0),
    pwmWriter(nullptr)
  {
  }

public:
  void initialize(
    int en,
    int lpwm,
    int rpwm,
    int is,
    int min = 0,
    int max = PWM_MAX,
    bool invertCommand = false,
    float stallCurrentThreshold = 1.0,
    PwmWriter onPwmWrite = nullptr)
  {
    enPin = en;
    lpwmPin = lpwm;
    rpwmPin = rpwm;
    isPin = is;
    pwmMin = min;
    pwmMax = max;
    invert = invertCommand;
    stopBeforeReverse = false;
    stallThreshold = stallCurrentThreshold;
    velocity = 0.0;
    current = 0.0;
    pwmWriter = onPwmWrite;

    pinMode(enPin, OUTPUT);
    digitalWrite(enPin, LOW);

    if (pwmWriter == nullptr)
    {
      pinMode(lpwmPin, OUTPUT);
      pinMode(rpwmPin, OUTPUT);
      analogWriteFrequency(lpwmPin, PWM_FREQ);
      analogWriteFrequency(rpwmPin, PWM_FREQ);
    }

    writePwm(lpwmPin, 0);
    writePwm(rpwmPin, 0);
  }

  void loadSettings(ros::NodeHandle& node, const char* group)
  {
    loadBoolParam(node, group, "enabled", enabled);
    loadParam(node, group, "pwmMin", pwmMin);
    loadParam(node, group, "pwmMax", pwmMax);
    loadParam(node, group, "stallThreshold", stallThreshold);
    loadBoolParam(node, group, "stopBeforeReverse", stopBeforeReverse);
    loadBoolParam(node, group, "invert", invert);

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

    if (enabled)
    {
      enable();
    }
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
    writePwm(lpwmPin, 0);
    writePwm(rpwmPin, 0);
    velocity = 0.0f;
  }

  float read()
  {
    if (isPin < 0)
    {
      rawCurrent = 0;
      current = 0.0f;
      return current;
    }

    rawCurrent = analogRead(isPin);
    float sensedVoltage = (float(rawCurrent) / ADC_MAX_COUNTS) * ADC_REFERENCE_VOLTS;
    current = sensedVoltage * CURRENT_SENSE_AMPS_PER_VOLT;
    return current;
  }

  int readRaw()
  {
    return rawCurrent;
  }

  void write(float command)
  {
    if (!enabled) return;

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

    bool reversedDirection =
      (velocity > 0.0f && nextCommand < 0.0f) ||
      (velocity < 0.0f && nextCommand > 0.0f);

    if (stopBeforeReverse && reversedDirection)
    {
      writePwm(lpwmPin, 0);
      writePwm(rpwmPin, 0);
      velocity = 0.0f;
      return;
    }

    if (nextCommand != velocity)
    {
      int lpwm = direction < 0 ? 0 : pwm;
      int rpwm = direction > 0 ? 0 : pwm;

      writePwm(lpwmPin, lpwm);
      writePwm(rpwmPin, rpwm);

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
      "%s: pwmMin=%d pwmMax=%d %s stallThreshold=%s current=%s%s%s",
      group, pwmMin, pwmMax,
      enabled ? "enabled" : "disabled",
      thresh_s, current_s,
      invert ? " invert" : "",
      stopBeforeReverse ? " stopBeforeReverse" : "");

    node.loginfo(buffer);
  }

private:
  void writePwm(int channelOrPin, int value)
  {
    if (pwmWriter != nullptr)
    {
      pwmWriter(channelOrPin, value);
      return;
    }

    analogWrite(channelOrPin, value);
  }
};
