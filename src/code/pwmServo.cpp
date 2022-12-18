/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 pwmServo.cpp

 PWM Servo Controller Implementation
 Created 1/19/2021

 Copyright (C) 2021 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <math.h>
#include <pigpiod_if2.h>
#include <ros/ros.h>
#include "pwmServo.h"
#include "controllerFactory.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace std;
using namespace str1ker;

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const char pwmServo::TYPE[] = "servo";

/*----------------------------------------------------------*\
| pwmServo implementation
\*----------------------------------------------------------*/

REGISTER_CONTROLLER(pwmServo)

pwmServo::pwmServo(const char* path):
    servo(path),
    m_gpioLPWM(0),
    m_gpioRPWM(0),
    m_encoder(NULL)
{
}

const char* pwmServo::getType()
{
    return pwmServo::TYPE;
}

bool pwmServo::init()
{
    if (!m_enable) return true;

    gpioSetMode(m_gpioLPWM, PI_OUTPUT);
    gpioSetMode(m_gpioRPWM, PI_OUTPUT);

    if (m_encoder && !m_encoder->init())
    {
        ROS_ERROR("  failed to initialize %s encoder", getPath());
        return false;
    }

    ROS_INFO("  initialized %s %s on GPIO %d RPWM and %d LPWM",
        getPath(), getType(), m_gpioRPWM, m_gpioLPWM);

    return true;
}

double pwmServo::getPos()
{
    if (m_encoder) return m_encoder->getPos();

    return 0.0;
}

void pwmServo::setPos(double pos)
{
    if (!m_enable) return;

    double cur = getPos();
    bool forward = (pos - cur) >= 0;
    double delta = 0.0;

    double lastCur = 0;

    gpioPWM(m_gpioRPWM, forward ? MAX_SPEED : 0);
    gpioPWM(m_gpioLPWM, forward ? 0 : MAX_SPEED);

    do
    {
        usleep(10000);

        cur = getPos();
        delta = cur - pos;

#ifdef DEBUG
        if (abs(lastCur - cur) > 0.02)
            ROS_INFO("%g -> %g, delta %g", cur, pos, delta);
#endif

        lastCur = cur;

    } while (forward ? delta < 0 : delta > 0);

    gpioPWM(m_gpioRPWM, 0);
    gpioPWM(m_gpioLPWM, 0);

    sleep(1);
}

void pwmServo::deltaPos(double delta)
{
    double pos = getPos();
    double norm = max(min(pos + delta, 1.0), 0.0);

#ifdef DEBUG
    ROS_INFO("setPos %g + %g = %g", pos, delta, norm);
#endif

    setPos(norm);
}

void pwmServo::deserialize(ros::NodeHandle node)
{
    servo::deserialize(node);

    ros::param::get(getControllerPath("gpioLPWM"), m_gpioLPWM);
    ros::param::get(getControllerPath("gpioRPWM"), m_gpioRPWM);

    m_encoder = controllerFactory::deserialize<potentiometer>(getPath(), "encoder", node);

    if (!m_encoder)
    {
        ROS_WARN("%s failed to load encoder", getPath());
    }
}

void pwmServo::publish()
{
    if (m_encoder) m_encoder->publish();
}

controller* pwmServo::create(const char* path)
{
    return new pwmServo(path);
}
