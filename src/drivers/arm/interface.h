
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

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros.h>
#include <str1ker/VelocityCommand.h>
#include <str1ker/VelocityFeedback.h>
#include <str1ker/PositionCommand.h>
#include <str1ker/PositionFeedback.h>

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

// Velocity command topic (see VelocityCommand.msg)
const char VELOCITY_COMMAND[] = "velocity";

// Position command topic (see PositionCommand.msg)
const char POSITION_COMMAND[] = "position";

// State feedback topic (see StateFeedback.msg)
const char STATE_FEEDBACK[] = "state";

/*----------------------------------------------------------*\
| Forward Declarations
\*----------------------------------------------------------*/

void velocityCommand(const str1ker::VelocityCommand& msg);
void positionCommand(const str1ker::PositionCommand& msg);
void gripperCommand(const str1ker::GripperCommand& msg);
void stateFeedback();

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

// ROS node
ros::NodeHandle node;

// State feedback publisher
ros::Publisher statePub(
  STATE_FEEDBACK, &stateFeedbackMsg);

// Velocity command subscriber
ros::Subscriber<str1ker::VelocityCommand> velocitySub(
  VELOCITY_COMMAND, velocityCommand);

// Position command subscriber
ros::Subscriber<str1ker::PositionCommand> positionSub(
  POSITION_COMMAND, positionCommand);

// Gripper command subscriber
ros::Subscriber<str1ker::GripperCommand> gripperSub(
  GRIPPER_COMMAND, gripperCommand);

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
  str1ker::StateFeedback stateFeedbackMsg;

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
