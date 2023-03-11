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
    controller(robot, path)
{
}

const char* arm::getType()
{
    return arm::TYPE;
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

    m_shoulder = shared_ptr<servo>(controllerFactory::deserialize<servo>(m_robot, m_path.c_str(), "shoulder", node));
    m_upperarm = shared_ptr<servo>(controllerFactory::deserialize<servo>(m_robot, m_path.c_str(), "upperarm", node));
    m_forearm = shared_ptr<servo>(controllerFactory::deserialize<servo>(m_robot, m_path.c_str(), "forearm", node));
    m_trigger = shared_ptr<solenoid>(controllerFactory::deserialize<solenoid>(m_robot, m_path.c_str(), "trigger", node));
}

bool arm::init(ros::NodeHandle node)
{
    const char* path = getPath();
    char jointPath[128] = {0};
    strcpy(jointPath, path);
    strcat(jointPath, "/");

    char* jointName = jointPath + strlen(jointPath);
    int numJoints = sizeof(JOINT_NAMES) / sizeof(char*);
    m_jointPaths.resize(numJoints);

    for (int joint = 0; joint < numJoints; joint++)
    {
        strcpy(jointName, JOINT_NAMES[joint]);
        m_jointPaths[joint] = jointPath;
    }

    m_pub = node.advertise<sensor_msgs::JointState>(
        getPath(),
        PUBLISH_QUEUE_SIZE
    );

    if (m_shoulder && !m_shoulder->init(node)) return false;
    if (m_upperarm && !m_upperarm->init(node)) return false;
    if (m_forearm && !m_forearm->init(node)) return false;
    if (m_trigger && !m_trigger->init(node)) return false;

    return true;
}

void arm::update()
{
    double jointPositions[] =
    {
        m_shoulder ? m_shoulder->getPos() : 0.0,
        m_upperarm ? m_upperarm->getPos() : 0.0,
        m_forearm ? m_forearm->getPos() : 0.0,
    };

    double jointVelocities[] =
    {
        m_shoulder ? m_shoulder->getVelocity() : 0.0,
        m_upperarm ? m_upperarm->getVelocity() : 0.0,
        m_forearm ? m_forearm->getVelocity() : 0.0,
    };

    sensor_msgs::JointState jointState;
    jointState.header.stamp = ros::Time::now();
    int numJoints = m_jointPaths.size();

    jointState.name.resize(numJoints);
    jointState.position.resize(numJoints);
    jointState.velocity.resize(numJoints);

    for (int joint = 0; joint < numJoints; joint++)
    {
        jointState.name[joint] = m_jointPaths[joint];
        jointState.position[joint] = jointPositions[joint];
        jointState.velocity[joint] = jointVelocities[joint];
    }

    m_pub.publish(jointState);
}

controller* arm::create(robot& robot, const char* path)
{
    return new arm(robot, path);
}
