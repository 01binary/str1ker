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
#include <pigpiod_if2.h>
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
    m_gpio(0)
{
}

solenoid::solenoid(robot& robot, const char* path, int gpio) :
    controller(robot, path),
    m_gpio(gpio)
{
}

const char* solenoid::getType()
{
    return solenoid::TYPE;
}

bool solenoid::init()
{
    if (!m_enable) return true;
    
    set_mode(m_robot.getGpio(), m_gpio, PI_OUTPUT);

    ROS_INFO("  initialized %s %s on GPIO %d", getPath(), getType(), m_gpio);

    return true;
}

void solenoid::trigger(double durationSeconds)
{
    if (!m_enable) return;

    useconds_t durationMicroseconds = durationSeconds / 1000000.0;

    if (gpio_write(m_robot.getGpio(), m_gpio, 1) < 0) {
        setLastError("failed to write gpio high");
        return;
    }

    usleep(durationMicroseconds);

    if (gpio_write(m_robot.getGpio(), m_gpio, 0) < 0) {
        setLastError("failed to write gpio low");
        return;
    }

    usleep(durationMicroseconds);

    setLastError(NULL);
}

void solenoid::deserialize(ros::NodeHandle node)
{
    controller::deserialize(node);

    ros::param::get(getControllerPath("gpio"), m_gpio);

    // TODO: advertise action
}

controller* solenoid::create(robot& robot, const char* path)
{
    return new solenoid(robot, path);
}
