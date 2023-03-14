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

    if (m_encoder && !m_encoder->init(node))
    {
        ROS_ERROR("  failed to initialize %s encoder", getPath());
        return false;
    }

    m_pub = node.advertise<Pwm>(getPath(), QUEUE_SIZE);

    ROS_INFO("  initialized %s %s on %s channels %d LPWM %d RPWM",
        getPath(), getType(), m_topic.c_str(), m_lpwm, m_rpwm);

    return true;
}

double motor::getPos()
{
    return m_encoder ? m_encoder->getPos() : 0.0;
}

double motor::getVelocity()
{
    return m_velocity;
}

bool motor::setVelocity(double velocity)
{
    uint8_t dutyCycle = uint8_t(abs(velocity) * double(DUTY_CYCLE));

    Pwm msg;
    msg.channels.resize(2);

    // RPWM
    msg.channels[0].channel = m_rpwm;
    msg.channels[0].mode = MODE_ANALOG;
    msg.channels[0].value = velocity >= 0 ? dutyCycle : 0;
    msg.channels[0].duration = 0;

    // LPWM
    msg.channels[1].channel = m_lpwm;
    msg.channels[1].mode = MODE_ANALOG;
    msg.channels[1].value = velocity >= 0 ? 0 : dutyCycle;
    msg.channels[1].duration = 0;

    m_pub.publish(msg);

    m_velocity = velocity;
    setLastError(NULL);
    return true;
}

void motor::configure(ros::NodeHandle node)
{
    controller::configure(node);

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
