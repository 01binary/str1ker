/*
                                                                                     @@@@@@@                  
 @@@@@@@@@@@@  @@@@@@@@@@@@   @@@@@@@@@@@@       @  @@@@@@@@@@@@@  @           @  @@@       @@@  @@@@@@@@@@@@ 
@              @ @           @            @    @ @  @              @        @@@      @@@@@@@    @            @
 @@@@@@@@@@@@  @   @         @@@@@@@@@@@@@   @   @   @             @   @@@@@      @@@       @@@ @@@@@@@@@@@@@ 
             @ @     @       @            @      @    @@           @@@@      @                  @            @
 @@@@@@@@@@@@  @       @     @            @      @      @@@@@@@@@  @          @   @@@       @@@ @            @
                                                                                     @@@@@@@                  
 controller.cpp

 Controller base class implementation
 Created 1/19/2021

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

controller::controller(const char* path) :
    m_name(robot::getControllerName(path)),
    m_path(path),
    m_enable(true)
{
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
