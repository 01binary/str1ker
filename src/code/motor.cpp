/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 motor.cpp

 PWM Motor Controller Implementation
 Created 1/19/2021

 Copyright (C) 2021 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <math.h>
#include <ros/ros.h>
#include "robot.h"
#include "motor.h"
#include "controllerFactory.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace std;
using namespace str1ker;

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const char motor::TYPE[] = "motor";

/*----------------------------------------------------------*\
| motor implementation
\*----------------------------------------------------------*/

REGISTER_CONTROLLER(motor)

motor::motor(robot& robot, const char* path):
    controller(robot, path),
    m_topic("/robot/pwm"),
    m_lpwm(0),
    m_rpwm(1)
{
}

const char* motor::getType()
{
    return motor::TYPE;
}

bool motor::init(ros::NodeHandle node)
{
    if (!m_enable) return true;

    m_pub = node.advertise<Pwm>(getPath(), QUEUE_SIZE);

    if (m_encoder && !m_encoder->init(node))
    {
        ROS_ERROR("  failed to initialize %s encoder", getPath());
        return false;
    }

    ROS_INFO("  initialized %s %s on %s %d LPWM %d RPWM",
        getPath(), getType(), m_topic.c_str(), m_lpwm, m_rpwm);

    return true;
}

double motor::getPos()
{
    return m_encoder ? m_encoder->getPos() : 0.0;
}

double motor::getMinPos()
{
    return m_encoder ? m_encoder->getMinPos() : 0.0;
}

double motor::getMaxPos()
{
    return m_encoder ? m_encoder->getMaxPos() : 1.0;
}

double motor::getVelocity()
{
    return m_velocity;
}

bool motor::setVelocity(double velocity)
{
    uint8_t dutyCycle = uint8_t(abs(velocity) * double(DUTY_CYCLE));

    Pwm msg;

    // RPWM
    msg.channel1 = m_rpwm;
    msg.dutyCycle1 = velocity >= 0 ? dutyCycle : 0;

    // LPWM
    msg.channel2 = m_lpwm;
    msg.dutyCycle2 = velocity >= 0 ? 0 : dutyCycle;

    m_pub.publish(msg);

    m_velocity = velocity;
    setLastError(NULL);
    return true;
}

void motor::deserialize(ros::NodeHandle node)
{
    controller::deserialize(node);

    if (!ros::param::get(getControllerPath("topic"), m_topic))
        ROS_WARN("%s no PWM topic specified, default /robot/pwm", getPath());

    if (!ros::param::get(getControllerPath("LPWM"), m_lpwm))
        ROS_WARN("%s no LPWM channel specified, default 0", getPath());

    if (!ros::param::get(getControllerPath("RPWM"), m_lpwm))
        ROS_WARN("%s no RPWM channel specified, default 1", getPath());

    m_encoder = shared_ptr<potentiometer>(controllerFactory::deserialize<potentiometer>(
        m_robot, getPath(), "encoder", node));

    if (!m_encoder)
        ROS_WARN("%s failed to load encoder", getPath());
}

controller* motor::create(robot& robot, const char* path)
{
    return new motor(robot, path);
}
