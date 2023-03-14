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
const char* arm::JOINTS[] = { "shoulder", "upperarm", "forearm" };

/*----------------------------------------------------------*\
| arm implementation
\*----------------------------------------------------------*/

REGISTER_CONTROLLER(arm)

arm::arm(robot& robot, const char* path) :
    controller(robot, path),
    m_hw(this, robot.getNode()),
    m_lastUpdate(0)
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

    int numJoints = sizeof(JOINTS) / sizeof(char*);

    m_jointPositions.resize(numJoints);
    m_jointVelocities.resize(numJoints);
    m_jointEfforts.resize(numJoints);
    m_jointPosCommands.resize(numJoints);
    m_jointVelCommands.resize(numJoints);

    for (int joint = 0; joint < numJoints; joint++)
    {
        m_joints.push_back(shared_ptr<servo>(controllerFactory::deserialize<servo>(
            m_robot, m_path.c_str(), JOINTS[joint], node)));
    }
}

bool arm::init(ros::NodeHandle node)
{
    const char* path = getPath();
    char jointPath[128] = {0};
    strcpy(jointPath, path);
    strcat(jointPath, "/");

    char* jointName = jointPath + strlen(jointPath);
    m_jointPaths.resize(m_joints.size());

    for (int joint = 0; joint < m_joints.size(); joint++)
    {
        // Build joint path
        strcpy(jointName, JOINTS[joint]);
        m_jointPaths[joint] = jointPath;

        // Initialize joint actuator
        if (!m_joints[joint]->init(node))
            return false;

        // Register joint state interface
        hardware_interface::JointStateHandle stateHandle(
            jointPath,
            &m_jointPositions[joint],
            &m_jointVelocities[joint],
            NULL
        );

        m_states.registerHandle(stateHandle);

        // Register joint position interface
        hardware_interface::JointHandle posHandle(stateHandle, &m_jointPosCommand[joint]);
        m_position.registerHandle(posHandle);

        // Register joint velocity interface
        hardware_interface::JointHandle velHandle(state, &m_jointVelCommand[joint]);
        m_velocity.registerHandle(velHandle);

        // Register joint limits interface
        joint_limits_interface::JointLimits lim;
        joint_limits_interface::getJointLimits(jointPath, node, lim);
        joint_limits_interface::VelocityJointSaturationHandle limHandle(velHandle, lim);
        m_limits.registerHandle(limHandle);
    }

    // Register interfaces for all joints
    registerInterface(&m_state);
    registerInterface(&m_position);
    registerInterface(&m_velocity);
    registerInterface(&m_limits);

    return true;
}

void arm::update()
{
    ros::Time time = ros::Time::now();
    ros::Duration period = time - m_lastUpdate;

    for (int joint = 0; joint < m_joints.size(); joint++)
    {
        m_jointPositions[joint] = m_joint[joint]->getPos();
        m_jointVelocities[joint] = m_joint[joint]->getVelocity();
    }

    m_hw->update(time, period);

    for (int joint = 0; joint < m_joints.size(); joint++)
    {
        if (m_jointVelCommands[joint]) {
            m_joints[joint]->setVelocity(m_jointVelCommands[joint]);
        } else {
            // TODO: this should have its own loop
            m_joints[joint]->setPos(m_jointPosCommands[joint]);
        }
    }

    m_lastUpdate = time;
}

controller* arm::create(robot& robot, const char* path)
{
    return new arm(robot, path);
}
