/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 solenoid.cpp

 Solenoid Controller Implementation
 Created 1/19/2021

 Copyright (C) 2021 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <unistd.h>
#include <ros/ros.h>
#include "robot.h"
#include "solenoid.h"
#include "controllerFactory.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace str1ker;

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const char solenoid::TYPE[] = "solenoid";

/*----------------------------------------------------------*\
| solenoid implementation
\*----------------------------------------------------------*/

REGISTER_CONTROLLER(solenoid)

solenoid::solenoid(robot& robot, const char* path) :
    controller(robot, path),
    m_channel(0)
{
}

const char* solenoid::getType()
{
    return solenoid::TYPE;
}

bool solenoid::init(ros::NodeHandle node)
{
    if (!m_enable) return true;

    ROS_INFO("  initialized %s %s on %s %d", getPath(), getType(), m_topic.c_str(), m_channel);

    return true;
}

void solenoid::trigger(double durationSeconds)
{
    if (!m_enable) return;

    useconds_t durationMicroseconds = durationSeconds / 1000000.0;

    // TODO: publish to channel?
    // what if that's not fast enough?

    // TODO set high

    usleep(durationMicroseconds);

    // TODO set low

    usleep(durationMicroseconds);

    setLastError(NULL);
}

void solenoid::deserialize(ros::NodeHandle node)
{
    controller::deserialize(node);

    if (!ros::param::get(getControllerPath("topic"), m_topic))
        ROS_WARN("%s did not specify topic", getPath());

    if (!ros::param::get(getControllerPath("channel"), m_channel))
        ROS_WARN("%s did not specify channel", getPath());
}

controller* solenoid::create(robot& robot, const char* path)
{
    return new solenoid(robot, path);
}
