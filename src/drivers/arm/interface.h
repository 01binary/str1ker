
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
#include "encoder.h"

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const char VELOCITY[] = "velocity";   // Velocity command topic
const char POSITION[] = "position";   // Position command topic
const char GRIPPER[] = "gripper";     // Gripper command topic
const char STATE[] = "state";         // State feedback topic

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
ros::Publisher statePub(STATE, &stateFeedbackMsg);
VelocitySubscriber velocitySub(VELOCITY, velocityCommand);
PositionSubscriber positionSub(POSITION, positionCommand);
GripperSubscriber gripperSub(GRIPPER, gripperCommand);

/*----------------------------------------------------------*\
| Functions
\*----------------------------------------------------------*/

void initializeRosInterface()
{
  node.initNode();
  node.advertise(statePub);
  node.subscribe(velocitySub);
  node.subscribe(positionSub);
  node.subscribe(gripperSub);
  node.negotiateTopics();
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
  stateFeedbackMsg.basePosition = basePosition;
  stateFeedbackMsg.baseVelocity = baseVelocity;
  stateFeedbackMsg.baseStalled = baseStalled;

  stateFeedbackMsg.shoulderPosition = shoulderPosition;
  stateFeedbackMsg.shoulderVelocity = shoulderVelocity;
  stateFeedbackMsg.shoulderStalled = shoulderStalled;

  stateFeedbackMsg.elbowPosition = elbowPosition;
  stateFeedbackMsg.elbowVelocity = elbowVelocity;
  stateFeedbackMsg.elbowStalled = elbowStalled;

  statePub.publish(&msg);
}
