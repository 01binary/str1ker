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
    m_controllerManager(this, robot.getNode()),
    m_lastUpdate(0)
{
}

const char* arm::getType()
{
    return arm::TYPE;
}

void arm::trigger()
{
    m_actuatorVelCommands[m_jointNames.size() - 1] = 1.0;
}

void arm::configure(ros::NodeHandle node)
{
    controller::configure(node);

    // Configure encoders and actuators
    vector<string> params;
    ros::param::getParamNames(params);
    set<string> jointNames;

    for (int param = 0; param < params.size(); param++)
    {
        auto paramName = params[param];

        if (paramName.find(m_path) == 0)
        {
            size_t paramNameStart = m_path.length() + 1;
            size_t paramNameEnd = paramName.find('/', paramNameStart);

            if (paramNameEnd == -1) continue;

            string jointName = paramName.substr(
                paramNameStart, paramNameEnd - paramNameStart);

            jointNames.insert(jointName);
        }
    }

    for (auto jointName: jointNames)
    {
        auto actuator = shared_ptr<controller>(
            controllerFactory::deserialize(
                m_robot,
                m_path.c_str(),
                (jointName + "/actuator").c_str(),
                node
            )
        );

        auto enc = shared_ptr<controller>(
            controllerFactory::deserialize(
                m_robot,
                m_path.c_str(),
                (jointName + "/encoder").c_str(),
                node
            )
        );

        if (actuator)
        {
            if (strcmp(actuator->getType(), "solenoid") == 0)
            {
                m_solenoid = dynamic_pointer_cast<solenoid>(actuator);
            }
            else if (strcmp(actuator->getType(), "motor") == 0)
            {
                m_actuators.push_back(dynamic_pointer_cast<motor>(actuator));
            }
        }

        if (enc)
        {
            m_encoders.push_back(dynamic_pointer_cast<encoder>(enc));
        }

        m_jointNames.push_back(jointName);
    }

    size_t numJoints = m_jointNames.size();

    m_actuatorPos.resize(numJoints);
    m_actuatorVel.resize(numJoints);
    m_actuatorEfforts.resize(numJoints);
    m_actuatorVelCommands.resize(numJoints);

    // Configure limits for all joints including solenoid
    string description;

    if (ros::param::get("robot_description", description))
    {
        urdf::Model model;

        if (model.initString(description))
        {
            for (int jointIndex = 0; jointIndex < numJoints; jointIndex++)
            {
                auto jointName = m_jointNames[jointIndex];
                auto joint = model.getJoint(jointName);

                joint_limits_interface::JointLimits limits;

                if (!joint_limits_interface::getJointLimits(joint, limits))
                    ROS_WARN("no limits for %s in robot_description", jointName.c_str());

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
    // Initialize velocity/position actuators and their encoders
    for (int jointIndex = 0; jointIndex < m_jointNames.size(); jointIndex++)
    {
        // Initialize actuator controller
        auto jointName = m_jointNames[jointIndex];

        if (jointIndex < m_actuators.size())
        {
            // Initialize motor
            if (!m_actuators[jointIndex]->init(node))
                return false;

            // Initialize encoder
            if (!m_encoders[jointIndex]->init(node))
                return false;
        }
        else
        {
            // Initialize solenoid
            if (!m_solenoid->init(node))
                return false;
        }

        // Register state interface
        hardware_interface::JointStateHandle actuatorState(
            jointName,
            &m_actuatorPos[jointIndex],
            &m_actuatorVel[jointIndex],
            &m_actuatorEfforts[jointIndex]
        );

        m_actuatorPos[jointIndex] = 0.0;
        m_actuatorVel[jointIndex] = 0.0;
        m_actuatorEfforts[jointIndex] = 0.0;
        m_stateInterface.registerHandle(actuatorState);

        // Register velocity interface
        hardware_interface::JointHandle actuatorVelocity(
            actuatorState,
            &m_actuatorVelCommands[jointIndex]
        );

        m_actuatorVelCommands[jointIndex] = 0.0;
        m_velInterface.registerHandle(actuatorVelocity);

        // Register limits interface
        hardware_interface::JointStateHandle jointState(
            jointName,
            &m_actuatorPos[jointIndex],
            &m_actuatorVel[jointIndex],
            &m_actuatorEfforts[jointIndex]
        );

        hardware_interface::JointHandle jointHandle(
            jointState,
            &m_actuatorVel[jointIndex]
        );

        joint_limits_interface::VelocityJointSaturationHandle
            actuatorLimits(jointHandle, m_actuatorLimits[jointIndex]);
        
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

    for (auto actuator: m_actuators)
        actuator->update();

    for (auto enc: m_encoders)
        enc->update();

    m_solenoid->update();
    m_controllerManager.update(time, period);
    m_satInterface.enforceLimits(period);

    debug();

    write();

    m_lastUpdate = time;
}

void arm::read()
{
    for (int jointIndex = 0; jointIndex < m_jointNames.size(); jointIndex++)
    {
        if (jointIndex < m_actuators.size())
        {
            // Read motor and encoder
            m_actuatorPos[jointIndex] = m_encoders[jointIndex]->getPos();
            m_actuatorVel[jointIndex] = m_actuators[jointIndex]->getVelocity();
        }
        else
        {
            // Read solenoid
            auto solenoidLimit = m_actuatorLimits[m_actuators.size()];

            m_actuatorPos[m_actuators.size()] = m_solenoid->isTriggered()
                ? solenoidLimit.max_position
                : solenoidLimit.min_position;

            m_actuatorVel[m_actuators.size()] = m_solenoid->isTriggered()
                ? solenoidLimit.max_velocity
                : 0.0;
        }
    }
}

void arm::write()
{
    for (int jointIndex = 0; jointIndex < m_actuators.size(); jointIndex++)
    {
        if (jointIndex < m_actuators.size())
        {
            // Write motor
            m_actuators[jointIndex]->command(m_actuatorVelCommands[jointIndex]);
        }
        else
        {
            // Write solenoid
            if (m_actuatorVelCommands[jointIndex] > 0.0)
            {
                m_solenoid->trigger();
                m_actuatorVelCommands[jointIndex] = 0.0;
            }
        }
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
