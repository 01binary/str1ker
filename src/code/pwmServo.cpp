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
#include <pigpio.h>
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
    m_lpwm(0),
    m_rpwm(0),
    m_pot(NULL)
{
}

const char* pwmServo::getType()
{
    return pwmServo::TYPE;
}

bool pwmServo::init()
{
    if (!m_enable) return true;

    gpioSetMode(m_lpwm, PI_OUTPUT);
    gpioSetMode(m_rpwm, PI_OUTPUT);

    if (m_pot && !m_pot->init())
        return false;

    ROS_INFO("  initialized %s %s on pins %d RPWM and %d LPWM",
        getPath(), getType(), m_rpwm, m_lpwm);

    return true;
}

double pwmServo::getPos()
{
    if (m_pot) return m_pot->getPos();

    return 0.0;
}

void pwmServo::setPos(double pos)
{
    if (!m_enable) return;

    double cur = getPos();
    bool forward = (pos - cur) >= 0;
    double delta = 0.0;

    double lastCur = 0;

    gpioPWM(m_rpwm, forward ? MAX_SPEED : 0);
    gpioPWM(m_lpwm, forward ? 0 : MAX_SPEED);

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

    gpioPWM(m_rpwm, 0);
    gpioPWM(m_lpwm, 0);

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

    ros::param::get(getControllerPath("lpwm"), m_lpwm);
    ros::param::get(getControllerPath("rpwm"), m_rpwm);

    m_pot = controllerFactory::deserialize<potentiometer>(getPath(), "feedback", node);

    if (!m_pot)
    {
        ROS_WARN("%s failed to load encoder", getPath());
    }
}

void pwmServo::publish()
{
    if (m_pot) m_pot->publish();
}

controller* pwmServo::create(const char* path)
{
    return new pwmServo(path);
}
