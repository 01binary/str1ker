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

#include <PWMServo.h> // Servo-motor control on Teensy
#include <ros.h>      // Robot Operating System
#include "params.h"   // Parameter loading

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

class ServoMotor
{
public:
  int signalPin;
  int position;
  PWMServo servo;

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
