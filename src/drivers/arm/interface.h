
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

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

extern const int STARTUP_DELAY;
extern const char ARM_NAMESPACE[];    // Node namespace
extern const char VELOCITY_TOPIC[];   // Velocity command topic
extern const char POSITION_TOPIC[];   // Position command topic
extern const char GRIPPER_TOPIC[];    // Gripper command topic
extern const char STATE_TOPIC[];      // State feedback topic

/*----------------------------------------------------------*\
| Definitions
\*----------------------------------------------------------*/

typedef ros::Subscriber<str1ker::VelocityCommand> VelocitySubscriber;
typedef ros::Subscriber<str1ker::PositionCommand> PositionSubscriber;
typedef ros::Subscriber<str1ker::GripperCommand> GripperSubscriber;

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

extern ros::NodeHandle node;
extern str1ker::StateFeedback stateFeedbackMsg;
extern ros::Publisher statePub;
extern VelocitySubscriber velocitySub;
extern PositionSubscriber positionSub;
extern GripperSubscriber gripperSub;

/*----------------------------------------------------------*\
| Functions
\*----------------------------------------------------------*/

ros::NodeHandle& initializeRos();
