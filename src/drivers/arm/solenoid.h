
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

public:
  Solenoid(): triggerPin(0)
  {
  }

public:
  void initialize(int trigger)
  {
    triggerPin = trigger;
    pinMode(triggerPin, OUTPUT);
    digitalWrite(triggerPin, LOW);
  }

  void write(double holdTime)
  {
    digitalWrite(triggerPin, HIGH);
    delay(int(holdTime * 1000));
    digitalWrite(triggerPin, LOW);
  }
};
