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

 Uses pigpiod C interface: https://abyz.me.uk/rpi/pigpio/pdif2.html

 Copyright (C) 2021 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <math.h>
#include <pigpiod_if2.h>
#include <ros/ros.h>
#include "robot.h"
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

const char pwmServo::TYPE[] = "pwmServo";

/*----------------------------------------------------------*\
| pwmServo implementation
\*----------------------------------------------------------*/

REGISTER_CONTROLLER(pwmServo)

pwmServo::pwmServo(robot& robot, const char* path):
    servo(robot, path),
    m_gpioLPWM(0),
    m_gpioRPWM(0),
    m_minSpeed(1.0),
    m_maxSpeed(1.0)
{
}

const char* pwmServo::getType()
{
    return pwmServo::TYPE;
}

bool pwmServo::init()
{
    if (!m_enable) return true;

    set_mode(m_robot.getGpio(), m_gpioLPWM, PI_OUTPUT);
    set_mode(m_robot.getGpio(), m_gpioRPWM, PI_OUTPUT);

    if (m_encoder && !m_encoder->init())
    {
        ROS_ERROR("  failed to initialize %s encoder", getPath());
        return false;
    }

    ROS_INFO("  initialized %s %s on GPIO %d RPWM and %d LPWM (%g to %g ramp)",
        getPath(), getType(), m_gpioRPWM, m_gpioLPWM, m_minSpeed, m_maxSpeed);

    return true;
}

double pwmServo::getPos()
{
    return m_encoder ? m_encoder->getPos() : 0.0;
}

void pwmServo::setPos(double target)
{
    // TODO: need to advertise a service for this (what does MoveIt need?)
    if (!m_enable) return;

    ros::Rate rate(4);

    double pos = getPos();
    double lastPos = pos;
    double initialDistance = abs(target - pos);
    double direction = (target - pos) >= 0.0 ? 1.0 : -1.0;
    double distance = 0.0;

    // Start moving
    if (!setVelocity(m_minSpeed * direction)) return;

    do
    {
        // Wait until servo reaches position
        rate.sleep();
        pos = getPos();
        lastPos = pos;
        distance = direction > 0 ? target - pos : pos - target;

        // Ramp speed
        double ramp = max(distance, 0.0) / initialDistance;
        double speed = rampSpeed(ramp);
        setVelocity(speed * direction);

#ifdef DEBUG
    if (abs(lastPos - pos) > 0.02)
        ROS_INFO("%g -> %g, distance %g, speed %g, ramp %g", pos, target, distance, speed, ramp);
#endif

    } while (distance > 0);

    // Stop
    setVelocity(0.0);

    // Allow time before next command
    sleep(1);
}

double pwmServo::getMinSpeed()
{
    return m_minSpeed;
}

double pwmServo::getMaxSpeed()
{
    return m_maxSpeed;
}

double pwmServo::rampSpeed(double ramp)
{
    const double RAMP[] = { 0.0, 1.0, 0.0 };
    int index = int(sizeof(RAMP) / sizeof(double) * ramp);
    return m_minSpeed + RAMP[index] * (m_maxSpeed - m_minSpeed);
}

bool pwmServo::setVelocity(double speed)
{
    bool forward = speed >= 0;
    unsigned int dutyCycle = abs(speed * DUTY_CYCLE);

    // Set RPWM pulse width
    int result = set_PWM_dutycycle(m_robot.getGpio(), m_gpioRPWM, forward ? dutyCycle : 0);

    if (result < 0) {
        handlePwmError(result);
        return false;
    }

    // Set LPWM pulse width
    result = set_PWM_dutycycle(m_robot.getGpio(), m_gpioLPWM, forward ? 0 : dutyCycle);

    if (result < 0) {
        handlePwmError(result);
        return false;
    }

    setLastError(NULL);
    return true;
}

void pwmServo::handlePwmError(int error)
{
    switch (error) {
        case PI_BAD_USER_GPIO:
            setLastError("invalid GPIO handle");
            break;
        case PI_NOT_PERMITTED:
            setLastError("access to GPIO pins denied");
            break;
        case PI_BAD_DUTYCYCLE:
            setLastError("invalid duty cycle");
            break;
        case PI_BAD_MODE:
            setLastError("invalid mode");
            break;
    }
}

void pwmServo::deserialize(ros::NodeHandle node)
{
    servo::deserialize(node);

    ros::param::get(getControllerPath("gpioLPWM"), m_gpioLPWM);
    ros::param::get(getControllerPath("gpioRPWM"), m_gpioRPWM);

    if (!ros::param::get(getControllerPath("minSpeed"), m_minSpeed))
        m_minSpeed = 1.0;
    
    if (!ros::param::get(getControllerPath("maxSpeed"), m_maxSpeed))
        m_maxSpeed = 1.0;

    m_encoder = shared_ptr<potentiometer>(controllerFactory::deserialize<potentiometer>(
        m_robot, getPath(), "encoder", node));

    if (!m_encoder)
    {
        ROS_WARN("%s failed to load encoder", getPath());
    }
}

controller* pwmServo::create(robot& robot, const char* path)
{
    return new pwmServo(robot, path);
}
