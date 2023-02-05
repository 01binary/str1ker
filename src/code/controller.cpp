/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 controller.cpp

 Controller Base Class Implementation
 Created 1/19/2021

 Copyright (C) 2021 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <algorithm>
#include <ros/ros.h>
#include "robot.h"
#include "solenoid.h"
#include "pwmServo.h"
#include "dynamixelPro.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace str1ker;
using namespace std;

/*----------------------------------------------------------*\
| controller implementation
\*----------------------------------------------------------*/

controller::controller(robot& robot, const char* path) :
    m_robot(robot),
    m_name(robot::getControllerName(path)),
    m_path(path),
    m_enable(true),
    m_error(NULL)
{
}

robot& controller::getRobot()
{
    return m_robot;
}

const char* controller::getName()
{
    return m_name.c_str();
}

const char* controller::getPath()
{
    return m_path.c_str();
}

const bool controller::isEnabled()
{
    return m_enable;
}

const char* controller::getLastError()
{
    return m_error;
}

void controller::setLastError(const char* error)
{
    m_error = error;
}

void controller::deserialize(ros::NodeHandle node)
{
    string indent;
    indent.resize((max(count(m_path.begin(), m_path.end(), '/') - 2, 1)) * 2, ' ');

    ROS_INFO("%sloading %s %s", indent.c_str(), getName(), getType());

    ros::param::get(getControllerPath("enable"), m_enable);
}

bool controller::init()
{
    return true;
}

void controller::publish()
{
}

string controller::getControllerPath(const char* controllerName)
{
    return m_path + "/" + controllerName;
}
