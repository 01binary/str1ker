
/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 interface.h
 ROS Interface
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
#include <str1ker/GripperCommand.h>
#include <str1ker/StateFeedback.h>
#include "actuator.h"
#include "motor.h"
#include "solenoid.h"
#include "encoder.h"

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

extern const int STARTUP_DELAY;
const String NAMESPACE = "arm/";                // Node namespace
const String VELOCITY = NAMESPACE + "velocity"; // Velocity command topic
const String POSITION = NAMESPACE + "position"; // Position command topic
const String GRIPPER = NAMESPACE + "gripper";   // Gripper command topic
const String STATE = NAMESPACE + "state";       // State feedback topic

/*----------------------------------------------------------*\
| Definitions
\*----------------------------------------------------------*/

typedef ros::Subscriber<str1ker::VelocityCommand> VelocitySubscriber;
typedef ros::Subscriber<str1ker::PositionCommand> PositionSubscriber;
typedef ros::Subscriber<str1ker::GripperCommand> GripperSubscriber;

/*----------------------------------------------------------*\
| Forward Declarations
\*----------------------------------------------------------*/

extern Actuator<FusionEncoder, Motor> base;
extern Actuator<Potentiometer, Motor> shoulder;
extern Actuator<Potentiometer, Motor> elbow;
extern Solenoid gripper;

void velocityCommand(const str1ker::VelocityCommand& msg);
void positionCommand(const str1ker::PositionCommand& msg);
void gripperCommand(const str1ker::GripperCommand& msg);
void stateFeedback();

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

ros::NodeHandle node;
str1ker::StateFeedback stateFeedbackMsg;
ros::Publisher statePub(STATE.c_str(), &stateFeedbackMsg);
VelocitySubscriber velocitySub(VELOCITY.c_str(), velocityCommand);
PositionSubscriber positionSub(POSITION.c_str(), positionCommand);
GripperSubscriber gripperSub(GRIPPER.c_str(), gripperCommand);

/*----------------------------------------------------------*\
| Functions
\*----------------------------------------------------------*/

ros::NodeHandle& initializeRos()
{
  node.initNode();

  while (!node.connected())
  {
    node.spinOnce();
  }

  node.advertise(statePub);
  node.subscribe(velocitySub);
  node.subscribe(positionSub);
  node.subscribe(gripperSub);
  node.negotiateTopics();

  delay(STARTUP_DELAY);

  return node;
}

void velocityCommand(const str1ker::VelocityCommand& msg)
{
  base.writeVelocity(msg.base);
  shoulder.writeVelocity(msg.shoulder);
  elbow.writeVelocity(msg.elbow);
}

void positionCommand(const str1ker::PositionCommand& msg)
{
  base.writePosition(msg.base);
  shoulder.writePosition(msg.shoulder);
  elbow.writePosition(msg.elbow);
}

void gripperCommand(const str1ker::GripperCommand& msg)
{
  gripper.write(msg.holdTime);
}

void stateFeedback()
{
  stateFeedbackMsg.basePosition = base.position;
  stateFeedbackMsg.baseVelocity = base.velocity;
  stateFeedbackMsg.baseStalled = base.stalled;

  stateFeedbackMsg.shoulderPosition = shoulder.position;
  stateFeedbackMsg.shoulderVelocity = shoulder.velocity;
  stateFeedbackMsg.shoulderStalled = shoulder.stalled;

  stateFeedbackMsg.elbowPosition = elbow.position;
  stateFeedbackMsg.elbowVelocity = elbow.velocity;
  stateFeedbackMsg.elbowStalled = elbow.stalled;

  statePub.publish(&stateFeedbackMsg);
}
