/*
                                                                                     ███████
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████
 servoMotor.h
 Servo-Motor Driver
 Copyright (C) 2026 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <Servo.h>
#include <ros.h>
#include "params.h"

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

class ServoMotor
{
public:
  int signalPin;
  int position;
  Servo servo;

public:
  ServoMotor():
    signalPin(0),
    position(0)
  {
  }

  void initialize(int pin)
  {
    signalPin = pin;
    position = 0;
    servo.attach(signalPin);
  }

  void write(int nextPosition)
  {
    position = constrain(nextPosition, 0, 180);
    servo.write(position);
  }

  int read() const
  {
    return position;
  }
};
