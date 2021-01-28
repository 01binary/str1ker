/*
                                                                                     @@@@@@@                  
 @@@@@@@@@@@@  @@@@@@@@@@@@   @@@@@@@@@@@@       @  @@@@@@@@@@@@@  @           @  @@@       @@@  @@@@@@@@@@@@ 
@              @ @           @            @    @ @  @              @        @@@      @@@@@@@    @            @
 @@@@@@@@@@@@  @   @         @@@@@@@@@@@@@   @   @   @             @   @@@@@      @@@       @@@ @@@@@@@@@@@@@ 
             @ @     @       @            @      @    @@           @@@@      @                  @            @
 @@@@@@@@@@@@  @       @     @            @      @      @@@@@@@@@  @          @   @@@       @@@ @            @
                                                                                     @@@@@@@                  
 arm.cpp

 Robot arm implementation
 Created 1/19/2021

 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros/ros.h>
#include "robot.h"
#include "controllerFactory.h"
#include "arm.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace str1ker;
using namespace std;

/*----------------------------------------------------------*\
| arm implementation
\*----------------------------------------------------------*/

arm::arm(
    const char* path,
    servo* shoulder,
    linear* upperarm,
    linear* forearm,
    solenoid* trigger) :
        m_path(path),
        m_name(robot::getControllerName(path)),
        m_shoulder(shoulder),
        m_upperarm(upperarm),
        m_forearm(forearm),
        m_trigger(trigger)
{
}

arm::arm(const char* path) :
    m_path(path),
    m_name(robot::getControllerName(path)),
    m_shoulder(NULL),
    m_upperarm(NULL),
    m_forearm(NULL),
    m_trigger(NULL)
{
}

arm::~arm()
{
    delete m_shoulder;
    delete m_upperarm;
    delete m_forearm;
    delete m_trigger;
}

bool arm::init()
{
    if (m_shoulder && !m_shoulder->init()) return false;
    if (m_upperarm && !m_upperarm->init()) return false;
    if (m_forearm && !m_forearm->init()) return false;
    if (m_trigger && !m_trigger->init()) return false;

    return true;
}

void arm::rotate(double deltaRad)
{
    if (!m_shoulder) return;

    ROS_INFO("rotate %s by %g", m_shoulder->getPath(), deltaRad);

    return m_shoulder->rotate(deltaRad);
}

void arm::raise(double amount)
{
    if (!m_upperarm) return;

    ROS_INFO("raise %s by %g", m_upperarm->getPath(), amount);

    return m_upperarm->extend(amount);
}

void arm::lower(double amount)
{
    if (!m_upperarm) return;

    ROS_INFO("lower %s by %g", m_upperarm->getPath(), amount);

    return m_upperarm->contract(amount);
}

void arm::extend(double amount)
{
    if (!m_forearm) return;

    ROS_INFO("extend %s by %g", m_forearm->getPath(), amount);

    return m_forearm->extend(amount);
}

void arm::contract(double amount)
{
    if (!m_forearm) return;

    ROS_INFO("contract %s by %g", m_forearm->getPath(), amount);

    return m_forearm->contract(amount);
}

void arm::trigger(double durationSeconds)
{
    if (!m_trigger) return;

    ROS_INFO("trigger %s for %g", m_trigger->getPath(), durationSeconds);

    return m_trigger->trigger(durationSeconds);
}

void arm::deserialize()
{
    ROS_INFO("  loading %s arm", m_name.c_str());

    m_shoulder = controllerFactory::deserialize<servo>(m_path.c_str(), "shoulder");
    m_upperarm = controllerFactory::deserialize<linear>(m_path.c_str(), "upperarm");
    m_forearm = controllerFactory::deserialize<linear>(m_path.c_str(), "forearm");
    m_trigger = controllerFactory::deserialize<solenoid>(m_path.c_str(), "trigger");
}
