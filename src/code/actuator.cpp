/*
                                                                                     @@@@@@@                  
 @@@@@@@@@@@@  @@@@@@@@@@@@   @@@@@@@@@@@@       @  @@@@@@@@@@@@@  @           @  @@@       @@@  @@@@@@@@@@@@ 
@              @ @           @            @    @ @  @              @        @@@      @@@@@@@    @            @
 @@@@@@@@@@@@  @   @         @@@@@@@@@@@@@   @   @   @             @   @@@@@      @@@       @@@ @@@@@@@@@@@@@ 
             @ @     @       @            @      @    @@           @@@@      @                  @            @
 @@@@@@@@@@@@  @       @     @            @      @      @@@@@@@@@  @          @   @@@       @@@ @            @
                                                                                     @@@@@@@                  
 actuator.cpp

 Actuator base class implementation
 Created 1/19/2021

 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros/ros.h>
#include "robot.h"
#include "solenoid.h"
#include "pwmServo.h"
#include "dynamixelServo.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace str1ker;
using namespace std;

/*----------------------------------------------------------*\
| actuator implementation
\*----------------------------------------------------------*/

actuator::actuator(const char* path) :
    m_name(robot::getComponentName(path)),
    m_path(path),
    m_enable(true)
{
}

const char* actuator::getName()
{
    return m_name.c_str();
}

const char* actuator::getPath()
{
    return m_path.c_str();
}

const bool actuator::isEnabled()
{
    return m_enable;
}

void actuator::deserialize()
{
    ROS_INFO("    loading %s %s", getName(), getType());

    ros::param::get(getComponentPath("enable"), m_enable);
}

string actuator::getComponentPath(const char* componentName)
{
    return m_path + "/" + componentName;
}
