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
// TODO: make these load dynamically
const char* arm::ACTUATORS[] = { "base", "upperarm_actuator", "forearm_actuator" };

/*----------------------------------------------------------*\
| arm implementation
\*----------------------------------------------------------*/

REGISTER_CONTROLLER(arm)

arm::arm(robot& robot, const char* path) :
    controller(robot, path),
    m_controllers(this, robot.getNode()),
    m_lastUpdate(0)
{
}

const char* arm::getType()
{
    return arm::TYPE;
}

void arm::trigger(double durationSeconds)
{
    if (!m_wrist) return;

    ROS_INFO("trigger %s for %g", m_wrist->getPath(), durationSeconds);

    return m_wrist->trigger(durationSeconds);
}

void arm::configure(ros::NodeHandle node)
{
    controller::configure(node);

    // Configure actuators
    int numActuators = sizeof(ACTUATORS) / sizeof(char*);

    m_actuatorPos.resize(numActuators);
    m_actuatorVel.resize(numActuators);
    m_actuatorEfforts.resize(numActuators);
    m_actuatorVelCommands.resize(numActuators);

    for (int actuator = 0; actuator < numActuators; actuator++)
    {
        auto pActuator = shared_ptr<motor>(controllerFactory::deserialize<motor>(
            m_robot,
            m_path.c_str(),
            ACTUATORS[actuator],
            node
        ));

        m_actuators.push_back(pActuator);
    }

    // Configure limits
    string description;

    if (ros::param::get("robot_description", description))
    {
        if (model.initString(description))
        {
            for (int actuator = 0; actuator < numActuators; actuator++)
            {
                auto jointName = m_actuators[actuator]->getName();
                auto joint = model->getJoint(jointName);

                joint_limits_interface::JointLimits limits;

                if (joint_limits_interface::getJointLimits(joint, limits))
                    ROS_INFO("%s loaded limits", getPath());
                else
                    ROS_WARN("no limits for %s in robot_description", getPath());

                m_actuatorLimits.push_back(limits);
            }
        }
    }
    else
    {
        ROS_WARN("no limits for %s, could not found robot_description", getPath());
    }
}

bool arm::init(ros::NodeHandle node)
{
    // Initialize actuators
    string basePath = getPath() + "/";
    m_actuatorPaths.resize(m_actuators.size());

    for (int actuator = 0; actuator < m_actuators.size(); actuator++)
    {
        // Build actuator path
        string path = basePath + ACTUATORS[actuator];
        m_actuatorPaths[actuator] = actuatorPath;

        // Initialize actuator
        if (!m_actuators[actuator]->init(node))
            return false;

        // Register state interface
        hardware_interface::ActuatorStateHandle actuatorState(
            actuatorPath,
            &m_actuatorPos[actuator],
            &m_actuatorVel[actuator],
            &m_actuatorEfforts[actuator]
        );

        m_stateInterface.registerHandle(actuatorState);

        // Register velocity interface
        hardware_interface::ActuatorHandle actuatorVelocity(
            actuatorState,
            &m_actuatorVelCommands[actuator]
        );

        m_velInterface.registerHandle(actuatorVelocity);

        // Register limits interface
        auto jointHandle = hardware_interface::JointHandle(
            actuatorState,
            &m_actuatorVelCommands[actuator]
        );

        const joint_limits_interface::VelocityJointSaturationHandle
            actuatorLimits(jointHandle, m_actuatorLimits[actuator]);
        
        m_satInterface.registerHandle(actuatorLimits);
    }

    // Register interfaces for all joints
    registerInterface(&m_stateInterface);
    registerInterface(&m_velInterface);
    registerInterface(&m_satInterface);

    return true;
}

void arm::update()
{
    ros::Time time = ros::Time::now();
    ros::Duration period = time - m_lastUpdate;

    read();

    m_controllers.update(time, period);
    m_satInterface.enforceLimits(period);

    write();

    m_lastUpdate = time;
}

void arm::read()
{
    for (int actuator = 0; actuator < m_actuators.size(); actuator++)
    {
        m_actuatorPos[actuator] = m_actuators[actuator]->getPos();
        m_actuatorVel[actuator] = m_actuators[actuator]->getVelocity();
    }
}

void arm::write()
{
    for (int actuator = 0; actuator < m_actuators.size(); actuator++)
    {
        m_actuators[actuator]->setVelocity(m_actuatorVelCommands[actuator]);
    }
}

controller* arm::create(robot& robot, const char* path)
{
    return new arm(robot, path);
}
