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
const char* arm::ACTUATORS[] = { "shoulder", "upperarm", "forearm" };

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

    int numActuators = sizeof(ACTUATORS) / sizeof(char*);

    m_actuatorPos.resize(numActuators);
    m_actuatorVel.resize(numActuators);
    m_actuatorEfforts.resize(numActuators);
    m_actuatorVelCommands.resize(numActuators);

    for (int actuator = 0; actuator < numActuators; actuator++)
    {
        m_actuators.push_back(shared_ptr<motor>(controllerFactory::deserialize<motor>(
            m_robot,
            m_path.c_str(),
            ACTUATORS[actuator],
            node
        )));
    }
}

bool arm::init(ros::NodeHandle node)
{
    const char* path = getPath();
    char actuatorPath[128] = {0};
    strcpy(actuatorPath, path);
    strcat(actuatorPath, "/");

    char* actuatorName = actuatorPath + strlen(actuatorPath);
    m_actuatorPaths.resize(m_actuators.size());

    for (int actuator = 0; actuator < m_actuators.size(); actuator++)
    {
        // Build actuator path
        strcpy(actuatorName, ACTUATORS[actuator]);
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
        hardware_interface::ActuatorHandle actuatorVel(
            actuatorState,
            &m_actuatorVelCommands[actuator]
        );

        m_velInterface.registerHandle(actuatorVel);
    }

    // Register interfaces for all joints
    registerInterface(&m_stateInterface);
    registerInterface(&m_velInterface);

    return true;
}

void arm::update()
{
    ros::Time time = ros::Time::now();
    ros::Duration period = time - m_lastUpdate;

    for (int actuator = 0; actuator < m_actuators.size(); actuator++)
    {
        m_actuatorPos[actuator] = m_actuators[actuator]->getPos();
        m_actuatorVel[actuator] = m_actuators[actuator]->getVelocity();
    }

    m_controllers.update(time, period);

    // TODO enforce limits? maybe inside setVelocity

    for (int actuator = 0; actuator < m_actuators.size(); actuator++)
    {
        m_actuators[actuator]->setVelocity(m_actuatorVelCommands[actuator]);
    }

    m_lastUpdate = time;
}

controller* arm::create(robot& robot, const char* path)
{
    return new arm(robot, path);
}
