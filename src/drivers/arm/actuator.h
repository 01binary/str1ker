
/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 actuator.h
 Actuator consisting of motor and encoder
 Copyright (C) 2025 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include "pid.h"

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

template <typename EncoderType, typename MotorType> class Actuator
{
public:
  enum ControlMode
  {
    VELOCITY,
    POSITION
  };

public:
  ControlMode mode;
  MotorType motor;
  EncoderType encoder;
  PID controller;

  double position;
  double velocity;
  double current;
  double stallThreshold;
  bool stalled;

public:
  Actuator():
    mode(VELOCITY),
    position(0.0),
    velocity(0.0),
    current(0.0),
    stallThreshold(0.0),
    stalled(false)
  {
  }

public:
  void readSettings()
  {
    controller.readSettings();
    encoder.readSettings();
    motor.readSettings();
  }

  void writeSettings()
  {
    controller.writeSettings();
    encoder.writeSettings();
    motor.writeSettings();
  }

  void update(double timeStep)
  {
    position = encoder.read();
    current = motor.read();
    stalled = current > stallThreshold;

    if (mode == POSITION)
    {
      velocity = controller.update(position, timeStep);
    }

    motor.write(velocity);
  }

  void writePosition(double command)
  {
    mode = POSITION;
    controller.start(command);
  }

  void writeVelocity(double command)
  {
    mode = VELOCITY;
    velocity = command;
  }
};
