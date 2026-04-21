/*
                                                                                     ███████
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████
 servoMotor.h
 Basic hobby servo wrapper
 Copyright (C) 2026 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <Servo.h>

#ifdef ROS
  #include <ros.h>
  #include "params.h"
#endif

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
    position(90)
  {
  }

  void initialize(int pin, int initialPosition = 90)
  {
    signalPin = pin;
    position = initialPosition;
    servo.attach(signalPin);
    servo.write(position);
  }

  #ifdef ROS
  void loadSettings(ros::NodeHandle& node, const char* group)
  {
    loadParam(node, group, "position", position);
    servo.write(position);
  }
  #endif

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
