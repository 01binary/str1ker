
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
#include "params.h"

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
  ros::NodeHandle& node;
  const char* group;
  const char* name;
  ControlMode mode;
  MotorType motor;
  EncoderType encoder;
  PID controller;

  float position;
  float velocity;
  float current;
  bool stalled;

public:
  Actuator(
    ros::NodeHandle& rosNode,
    const char* groupName,
    const char* actuatorName
  ):
    node(rosNode),
    group(groupName),
    name(actuatorName),
    mode(VELOCITY),
    position(0.0),
    velocity(0.0),
    current(0.0),
    stalled(false)
  {
  }

public:
  void loadSettings()
  {
    char path[64] = {0};
    char controllerGroup[64] = {0};
    char encoderGroup[64] = {0};
    char motorGroup[64] = {0};

    makeGroupPath(path, sizeof(path), group, name);
    makeGroupPath(controllerGroup, sizeof(controllerGroup), group, "controller");
    makeGroupPath(encoderGroup, sizeof(encoderGroup), group, "encoder");
    makeGroupPath(motorGroup, sizeof(motorGroup), group, "motor");

    controller.loadSettings(node, controllerGroup);
    encoder.loadSettings(node, encoderGroup);
    motor.loadSettings(node, motorGroup);

    debug();
  }

  void update(float timeStep)
  {
    position = encoder.read();
    current = motor.read();
    stalled = current > motor.stallThreshold;

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

  float getPosition() const
  {
    return position;
  }

  float getVelocity() const
  {
    return velocity;
  }

  float getCurrent() const
  {
    return current;
  }

  bool isStalled() const
  {
    return stalled;
  }

  void debug()
  {
    char path[64] = {0};
    char controllerGroup[64] = {0};
    char encoderGroup[64] = {0};
    char motorGroup[64] = {0};

    makeGroupPath(path, sizeof(path), group, name);
    makeGroupPath(controllerGroup, sizeof(controllerGroup), path, "controller");
    makeGroupPath(encoderGroup, sizeof(encoderGroup), path, "encoder");
    makeGroupPath(motorGroup, sizeof(motorGroup), path, "motor");

    controller.debug(node, controllerGroup);
    encoder.debug(node, encoderGroup);
    motor.debug(node, motorGroup);
  }
};
