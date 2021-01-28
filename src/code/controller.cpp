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

void controller::deserialize()
{
    ROS_INFO("    loading %s %s", getName(), getType());

    ros::param::get(getControllerPath("enable"), m_enable);
}

string controller::getControllerPath(const char* controllerName)
{
    return m_path + "/" + controllerName;
}
