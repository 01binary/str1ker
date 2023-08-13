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

#include <set>
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
    if (!m_solenoid) return;

    ROS_INFO("trigger %s for %g", m_solenoid->getPath(), durationSeconds);

    return m_solenoid->trigger(durationSeconds);
}

void arm::configure(ros::NodeHandle node)
{
    controller::configure(node);

    // Configure actuators
    vector<string> params;
    set<string> actuatorNames;

    ros::param::getParamNames(params);

    for (int param = 0; param < params.size(); param++)
    {
        auto paramName = params[param];

        if (paramName.find(m_path) == 0)
        {
            size_t paramNameStart = m_path.length() + 1;
            size_t paramNameEnd = paramName.find('/', paramNameStart);

            if (paramNameEnd == -1) continue;

            string actuatorName = paramName.substr(paramNameStart, paramNameEnd - paramNameStart);
            actuatorNames.insert(actuatorName);
        }
    }

    for (auto actuatorName: actuatorNames)
    {
        auto actuatorController = shared_ptr<controller>(controllerFactory::deserialize(
            m_robot,
            m_path.c_str(),
            actuatorName.c_str(),
            node
        ));

        if (strcmp(actuatorController->getType(), "solenoid") == 0)
        {
            m_solenoid = dynamic_pointer_cast<solenoid>(actuatorController);
        }
        else if (strcmp(actuatorController->getType(), "motor") == 0)
        {
            m_actuators.push_back(dynamic_pointer_cast<motor>(actuatorController));
        }
    }

    size_t numActuators = m_actuators.size();
    m_actuatorPos.resize(numActuators);
    m_actuatorVel.resize(numActuators);
    m_actuatorEfforts.resize(numActuators);
    m_actuatorVelCommands.resize(numActuators);

    // Configure limits
    string description;

    if (ros::param::get("robot_description", description))
    {
        urdf::Model model;

        if (model.initString(description))
        {
            for (int actuator = 0; actuator < numActuators; actuator++)
            {
                auto jointName = m_actuators[actuator]->getName();
                auto joint = model.getJoint(jointName);

                joint_limits_interface::JointLimits limits;

                if (!joint_limits_interface::getJointLimits(joint, limits))
                    ROS_WARN("no limits for %s in robot_description", m_actuators[actuator]->getName());

                m_actuatorLimits.push_back(limits);
            }
        }
    }
    else
    {
        ROS_WARN("no limits loaded for %s, could not found robot_description", getPath());
    }
}

bool arm::init(ros::NodeHandle node)
{
    // Initialize momentary actuators
    if (!m_solenoid->init(node))
        return false;

    // Initialize velocity/position actuators
    for (int actuator = 0; actuator < m_actuators.size(); actuator++)
    {
        // Initialize actuator controller
        auto name = m_actuators[actuator]->getName();

        if (!m_actuators[actuator]->init(node))
            return false;

        // Register state interface
        hardware_interface::ActuatorStateHandle actuatorState(
            name,
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
        if (m_actuatorLimits.size() > actuator)
        {
            hardware_interface::JointStateHandle jointState(
                name,
                &m_actuatorPos[actuator],
                &m_actuatorVel[actuator],
                &m_actuatorEfforts[actuator]
            );

            hardware_interface::JointHandle jointHandle(
                jointState,
                &m_actuatorVel[actuator]
            );

            joint_limits_interface::VelocityJointSaturationHandle
                actuatorLimits(jointHandle, m_actuatorLimits[actuator]);
            
            m_satInterface.registerHandle(actuatorLimits);
        }
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

    debug();

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

void arm::debug()
{
    const double THRESHOLD = 0.01;

    static bool s_initialized = false;
    static double s_vel[3];
    static double s_pos[3];

    if (m_actuators.size() < 3) return;

    if (!s_initialized)
    {
        s_vel[0] = 0.0;
        s_vel[1] = 0.0;
        s_vel[2] = 0.0;

        s_pos[0] = 0.0;
        s_pos[1] = 0.0;
        s_pos[2] = 0.0;

        s_initialized = true;
    }

    if (
        abs(m_actuatorPos[0] - s_pos[0]) >= THRESHOLD ||
        abs(m_actuatorPos[1] - s_pos[1]) >= THRESHOLD ||
        abs(m_actuatorPos[2] - s_pos[2]) >= THRESHOLD ||
        abs(m_actuatorVelCommands[0] - s_vel[0]) >= THRESHOLD ||
        abs(m_actuatorVelCommands[1] - s_vel[1]) >= THRESHOLD ||
        abs(m_actuatorVelCommands[2] - s_vel[2]) >= THRESHOLD
    )
    {
        ROS_INFO(
            "   base [%g >> %g] shoulder [%g >> %g] elbow [%g >> %g]",
            m_actuatorPos[0],
            m_actuatorVelCommands[0],
            m_actuatorPos[1],
            m_actuatorVelCommands[1],
            m_actuatorPos[2],
            m_actuatorVelCommands[2]
        );
    }

    s_pos[0] = m_actuatorPos[0];
    s_pos[1] = m_actuatorPos[1];
    s_pos[2] = m_actuatorPos[2];

    s_vel[0] = m_actuatorVelCommands[0];
    s_vel[1] = m_actuatorVelCommands[1];
    s_vel[2] = m_actuatorVelCommands[2];
}

controller* arm::create(robot& robot, const char* path)
{
    return new arm(robot, path);
}
