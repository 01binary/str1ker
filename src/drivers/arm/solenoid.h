
/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 solenoid.h
 Solenoid Driver
 Copyright (C) 2025 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

class Solenoid
{
public:
  int triggerPin;
  uint32_t lastTriggered;
  uint32_t holdMs;
  bool active;

public:
  Solenoid():
    triggerPin(0),
    lastTriggered(0),
    holdMs(0),
    active(false)
  {
  }

public:
  void initialize(int trigger)
  {
    triggerPin = trigger;
    pinMode(triggerPin, OUTPUT);
    digitalWrite(triggerPin, LOW);
    active = false;
    holdMs = 0;
    lastTriggered = 0;
  }

  void write(float holdTime)
  {
    if (holdTime <= 0.0f)
    {
      digitalWrite(triggerPin, LOW);
      active = false;
      holdMs = 0;
      return;
    }

    holdMs = uint32_t(holdTime * 1000.0f);
    lastTriggered = millis();
    active = true;
    digitalWrite(triggerPin, HIGH);
  }

  void update()
  {
    if (!active)
    {
      return;
    }

    uint32_t now = millis();
    if ((now - lastTriggered) >= holdMs)
    {
      digitalWrite(triggerPin, LOW);
      active = false;
    }
  }
};
