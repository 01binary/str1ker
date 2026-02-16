/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 joint.h
 Joint mapping and actuator wrapper
 Copyright (C) 2025 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros.h>
#include <str1ker/VelocityCommand.h>
#include <str1ker/PositionCommand.h>
#include <str1ker/StateFeedback.h>
#include "actuator.h"
#include "motor.h"

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

class joint
{
public:
  const char* name;

  float str1ker::VelocityCommand::*velocity;
  float str1ker::PositionCommand::*position;

  float str1ker::StateFeedback::*statePosition;
  float str1ker::StateFeedback::*stateVelocity;
  uint8_t str1ker::StateFeedback::*stateStalled;

public:
  joint(
    const char* jointName,
    float str1ker::VelocityCommand::*velocityMember,
    float str1ker::PositionCommand::*positionMember,
    float str1ker::StateFeedback::*statePositionMember,
    float str1ker::StateFeedback::*stateVelocityMember,
    uint8_t str1ker::StateFeedback::*stateStalledMember):
    name(jointName),
    velocity(velocityMember),
    position(positionMember),
    statePosition(statePositionMember),
    stateVelocity(stateVelocityMember),
    stateStalled(stateStalledMember)
  {
  }

  virtual ~joint() {}

  virtual void initialize() = 0;
  virtual void load(ros::NodeHandle& node) = 0;
  virtual void update(float timeStep) = 0;

  virtual void writeVelocity(const str1ker::VelocityCommand& msg) = 0;
  virtual void writePosition(const str1ker::PositionCommand& msg) = 0;
  virtual void writeState(str1ker::StateFeedback& msg) = 0;
};

template <typename EncoderType> class jointImpl : public joint
{
public:
  typedef Actuator<EncoderType, Motor> actuator;
  typedef void (*initializer)(actuator& actuatorRef);

public:
  actuator actuatorDevice;
  initializer init;

public:
  jointImpl(
    const char* jointName,
    float str1ker::VelocityCommand::*velocityMember,
    float str1ker::PositionCommand::*positionMember,
    float str1ker::StateFeedback::*statePositionMember,
    float str1ker::StateFeedback::*stateVelocityMember,
    uint8_t str1ker::StateFeedback::*stateStalledMember,
    initializer initializeHardware):
    joint(
      jointName,
      velocityMember,
      positionMember,
      statePositionMember,
      stateVelocityMember,
      stateStalledMember
    ),
    init(initializeHardware)
  {
  }

  void initialize() override
  {
    if (init)
    {
      init(actuatorDevice);
    }
  }

  void load(ros::NodeHandle& node) override
  {
    actuatorDevice.loadSettings(node, name);
  }

  void update(float timeStep) override
  {
    actuatorDevice.update(timeStep);
  }

  void writeVelocity(const str1ker::VelocityCommand& msg) override
  {
    actuatorDevice.writeVelocity(msg.*velocity);
  }

  void writePosition(const str1ker::PositionCommand& msg) override
  {
    actuatorDevice.writePosition(msg.*position);
  }

  void writeState(str1ker::StateFeedback& msg) override
  {
    msg.*statePosition = actuatorDevice.getPosition();
    msg.*stateVelocity = actuatorDevice.getVelocity();
    msg.*stateStalled = actuatorDevice.isStalled() ? 1 : 0;
  }
};
