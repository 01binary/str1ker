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

using namespace std;
using namespace str1ker;

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const char solenoid::TYPE[] = "solenoid";

/*----------------------------------------------------------*\
| solenoid implementation
\*----------------------------------------------------------*/

REGISTER_CONTROLLER(solenoid)

solenoid::solenoid(ros::NodeHandle node, string path) :
    controller(node, TYPE, path),
    m_channel(0),
    m_triggerDurationSec(DEFAULT_TRIGGER_DURATION_SEC),
    m_triggered(false),
    m_resetTime(0)
{
}

bool solenoid::configure()
{
    controller::configure();

    if (!ros::param::get(getChildPath("topic"), m_topic))
        ROS_WARN("%s did not specify output topic, using %s", getPath().c_str(), m_topic.c_str());

    if (!ros::param::get(getChildPath("channel"), m_channel))
        ROS_WARN("%s did not specify output channel, using %d", getPath().c_str(), m_channel);

    if (!ros::param::get(getChildPath("triggerSeconds"), m_triggerDurationSec))
        ROS_WARN("%s did not specify trigger duration, using %g sec", getPath().c_str(), m_triggerDurationSec);

    return true;
}

bool solenoid::init()
{
    if (!m_enable) return true;

    m_pub = m_node.advertise<Pwm>(m_topic.c_str(), QUEUE_SIZE);

    ROS_INFO("  initialized %s %s on %s channel %d trigger %g sec",
        getPath().c_str(), getType().c_str(), m_topic.c_str(), m_channel, m_triggerDurationSec);

    return true;
}

void solenoid::trigger()
{
    if (!m_enable) return;

    Pwm msg;
    msg.channels[0].channel = m_channel;
    msg.channels[0].mode = PwmChannel::MODE_DIGITAL;
    msg.channels[0].value = 1;
    msg.channels[0].duration = uint8_t(m_triggerDurationSec * 1000.0);

    m_pub.publish(msg);

    m_triggered = true;
    m_resetTime = ros::Time::now() + ros::Duration(m_triggerDurationSec);
}

bool solenoid::isTriggered()
{
    return m_triggered;
}

void solenoid::update(ros::Time time, ros::Duration period)
{
    if (!m_enable) return;

    if (m_triggered && time > m_resetTime)
    {
        m_triggered = false;
    }
}

controller* solenoid::create(ros::NodeHandle node, string path)
{
    return new solenoid(node, path);
}
