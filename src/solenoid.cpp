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

    m_pub = node.advertise<Pwm>(m_topic.c_str(), QUEUE_SIZE);

    ROS_INFO("  initialized %s %s on %s channel %d",
        getPath(), getType(), m_topic.c_str(), m_channel);

    return true;
}

void solenoid::trigger(double durationSec)
{
    if (!m_enable) return;

    Pwm msg;
    msg.channels[0].channel = m_channel;
    msg.channels[0].mode = PwmChannel::MODE_DIGITAL;
    msg.channels[0].value = 1;
    msg.channels[0].duration = uint8_t(durationSec / 1000000.0);

    m_pub.publish(msg);

    setLastError(NULL);
}

void solenoid::configure(ros::NodeHandle node)
{
    controller::configure(node);

    if (!ros::param::get(getControllerPath("topic"), m_topic))
        ROS_WARN("%s did not specify output topic, using %s", getPath(), m_topic.c_str());

    if (!ros::param::get(getControllerPath("channel"), m_channel))
        ROS_WARN("%s did not specify output channel, using %d", getPath(), m_channel);
}

controller* solenoid::create(robot& robot, const char* path)
{
    return new solenoid(robot, path);
}
