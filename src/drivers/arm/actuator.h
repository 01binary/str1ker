
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
| Forward Declarations
\*----------------------------------------------------------*/

struct ConfigurationGroup;

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

  float position;
  float velocity;
  float current;
  float stallThreshold;
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
  void loadSettings(ros::NodeHandle& node, const char* group)
  {
    controller.loadSettings(node, (String(group) + "/controller").c_str());
    encoder.loadSettings(node, (String(group) + "/encoder").c_str());
    motor.loadSettings(node, (String(group) + "/motor").c_str());
  }

  void update(float timeStep)
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

  void writePosition(float command)
  {
    mode = POSITION;
    controller.start(command);
  }

  void writeVelocity(float command)
  {
    mode = VELOCITY;
    velocity = command;
  }

  void dump(ros::NodeHandle& node, const char* group)
  {
    controller.dump(node, (String(group) + "/controller").c_str());
    encoder.dump(node, (String(group) + "/encoder").c_str());
    motor.dump(node, (String(group) + "/motor").c_str());
  }
};
