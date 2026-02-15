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

  float str1ker::StateFeedback::*state_position;
  float str1ker::StateFeedback::*state_velocity;
  uint8_t str1ker::StateFeedback::*state_stalled;

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
    state_position(statePositionMember),
    state_velocity(stateVelocityMember),
    state_stalled(stateStalledMember)
  {
  }

  virtual ~joint() {}

  virtual void initialize() = 0;
  virtual void load(ros::NodeHandle& node) = 0;
  virtual void update(float timeStep) = 0;

  virtual void write_velocity(const str1ker::VelocityCommand& msg) = 0;
  virtual void write_position(const str1ker::PositionCommand& msg) = 0;
  virtual void write_state(str1ker::StateFeedback& msg) = 0;
};

template <typename EncoderType> class joint_impl : public joint
{
public:
  typedef Actuator<EncoderType, Motor> actuator_t;
  typedef void (*init_t)(actuator_t& actuator);

public:
  actuator_t actuator;
  init_t init;

public:
  joint_impl(
    const char* jointName,
    float str1ker::VelocityCommand::*velocityMember,
    float str1ker::PositionCommand::*positionMember,
    float str1ker::StateFeedback::*statePositionMember,
    float str1ker::StateFeedback::*stateVelocityMember,
    uint8_t str1ker::StateFeedback::*stateStalledMember,
    init_t initializeHardware):
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
      init(actuator);
    }
  }

  void load(ros::NodeHandle& node) override
  {
    actuator.loadSettings(node, name);
  }

  void update(float timeStep) override
  {
    actuator.update(timeStep);
  }

  void write_velocity(const str1ker::VelocityCommand& msg) override
  {
    actuator.writeVelocity(msg.*velocity);
  }

  void write_position(const str1ker::PositionCommand& msg) override
  {
    actuator.writePosition(msg.*position);
  }

  void write_state(str1ker::StateFeedback& msg) override
  {
    msg.*state_position = actuator.getPosition();
    msg.*state_velocity = actuator.getVelocity();
    msg.*state_stalled = actuator.isStalled() ? 1 : 0;
  }
};
