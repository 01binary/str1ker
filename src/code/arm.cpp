/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 arm.cpp

 Robot Arm Controller Implementation
 Created 1/19/2021

 Copyright (C) 2021 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "robot.h"
#include "controllerFactory.h"
#include "arm.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace str1ker;
using namespace std;

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const char arm::TYPE[] = "arm";
const char* arm::JOINT_NAMES[] = { "shoulder", "upperarm", "forearm" };

/*----------------------------------------------------------*\
| arm implementation
\*----------------------------------------------------------*/

REGISTER_CONTROLLER(arm)

arm::arm(robot& robot, const char* path) :
    controller(robot, path),
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

const char* arm::getType()
{
    return arm::TYPE;
}

void arm::rotate(double delta)
{
    if (!m_shoulder) return;

    ROS_INFO("rotate %s by %g", m_shoulder->getPath(), delta);

    return m_shoulder->deltaPos(delta);
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

void arm::deserialize(ros::NodeHandle node)
{
    controller::deserialize(node);

    m_shoulder = controllerFactory::deserialize<servo>(m_robot, m_path.c_str(), "shoulder", node);
    m_upperarm = controllerFactory::deserialize<linear>(m_robot, m_path.c_str(), "upperarm", node);
    m_forearm = controllerFactory::deserialize<linear>(m_robot, m_path.c_str(), "forearm", node);
    m_trigger = controllerFactory::deserialize<solenoid>(m_robot, m_path.c_str(), "trigger", node);

    m_pub = node.advertise<sensor_msgs::JointState>(
        getPath(),
        PUBLISH_QUEUE_SIZE
    );
}

bool arm::init()
{
    if (m_shoulder && !m_shoulder->init()) return false;
    if (m_upperarm && !m_upperarm->init()) return false;
    if (m_forearm && !m_forearm->init()) return false;
    if (m_trigger && !m_trigger->init()) return false;

    return true;
}

void arm::update()
{
    if (m_shoulder) m_shoulder->update();
    if (m_upperarm) m_upperarm->update();
    if (m_forearm) m_forearm->update();
    if (m_trigger) m_trigger->update();

    string jointPrefix = getPath();
    int numJoints = sizeof(JOINT_NAMES) / sizeof(char*);
    double jointPositions[] = {
        m_shoulder ? m_shoulder->getPos() : 0.0,
        m_upperarm ? m_upperarm->getPos() : 0.0,
        m_forearm ? m_forearm->getPos() : 0.0,
    };

    sensor_msgs::JointState jointState;
    jointState.name.resize(numJoints);
    jointState.position.resize(numJoints);

    for (int joint = 0; joint < numJoints; joint++) {
        jointState.name[joint] = jointPrefix + "/" + JOINT_NAMES[joint];
        jointState.position[joint] = jointPositions[joint];
    }

    m_pub.publish(jointState);
}

controller* arm::create(robot& robot, const char* path)
{
    return new arm(robot, path);
}
