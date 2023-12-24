
/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 hardware.h
 Robot hardware drivers implementation
 Created 12/22/2023
 Copyright (C) 2023 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <set>
#include "controllerFactory.h"
#include "hardware.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace str1ker;
using namespace std;

/*----------------------------------------------------------*\
| hardware implementation
\*----------------------------------------------------------*/

hardware::hardware(ros::NodeHandle node)
    : m_node(node)
    , m_controllerManager(this, node) // TODO default rate
    , m_lastUpdate(0)
{
}

bool hardware::configure(const char* controllerNamespace)
{
    ros::param::get(string(controllerNamespace) + "/rate", m_rate);

    m_controllers = controllerFactory::deserialize(m_node, controllerNamespace);

    for (auto controller : m_controllers)
    {
        if (!controller->configure())
            return false;

        auto groupName = controller->getParentName();
        m_groups[groupName].push_back(controller);
    }

    string description;

    if (ros::param::get("robot_description", description))
    {
        urdf::Model model;

        if (model.initString(description))
        {
            for (auto controller : m_controllers)
            {
                if (controller->getType() == solenoid::TYPE || controller->getType() == motor::TYPE)
                {
                    joint_limits_interface::JointLimits limits;
                    auto jointName = controller->getParentName();
                    auto joint = model.getJoint(jointName);

                    if (!joint_limits_interface::getJointLimits(joint, limits))
                        ROS_WARN("no limits for %s in robot_description", jointName.c_str());

                    m_limits[jointName] = limits;
                }
            }
        }
    }
    else
    {
        ROS_WARN("no limits loaded for %s, could not found robot_description", controllerNamespace);
    }

    return true;
}

bool hardware::init()
{
    // Initialize hardware controllers

    for (auto controller: m_controllers)
    {
        if (!controller->init())
            return false;
    }

    // Initialize hardware state

    for (auto group: m_groups)
    {
        m_pos[group.first] = 0.0;
        m_vel[group.first] = 0.0;
        m_effort[group.first] = 0.0;
        m_commands[group.first] = 0.0;

        // Register state interface
        hardware_interface::JointStateHandle actuatorState(
            group.first,
            &m_pos[group.first],
            &m_vel[group.first],
            &m_effort[group.first]
        );

        m_stateInterface.registerHandle(actuatorState);

        // Register velocity interface
        hardware_interface::JointHandle actuatorVelocity(
            actuatorState,
            &m_commands[group.first]
        );

        m_commands[group.first] = 0.0;
        m_velInterface.registerHandle(actuatorVelocity);

        // Register limits interface
        hardware_interface::JointStateHandle jointState(
            group.first,
            &m_pos[group.first],
            &m_vel[group.first],
            &m_effort[group.first]
        );

        hardware_interface::JointHandle jointHandle(
            jointState,
            &m_vel[group.first]
        );

        joint_limits_interface::VelocityJointSaturationHandle
            actuatorLimits(jointHandle, m_limits[group.first]);
        
        m_satInterface.registerHandle(actuatorLimits);
    }

    // Register hardware interfaces

    registerInterface(&m_stateInterface);
    registerInterface(&m_velInterface);
    registerInterface(&m_satInterface);

    return true;
}

void hardware::update()
{
    ros::Time time = ros::Time::now();
    ros::Duration period = time - m_lastUpdate;

    read();

    for (auto controller: m_controllers)
        controller->update(time, period);

    m_controllerManager.update(time, period);
    m_satInterface.enforceLimits(period);

    debug();

    write();

    m_lastUpdate = time;
}

void hardware::read()
{
    for (auto group : m_groups)
    {
        for (auto controller: group.second)
        {
            if (controller->getType() == solenoid::TYPE)
            {
                solenoid* sol = dynamic_cast<solenoid*>(controller.get());
                auto solenoidLimit = m_limits[group.first];

                m_pos[group.first] = sol->isTriggered()
                    ? solenoidLimit.max_position
                    : solenoidLimit.min_position;

                m_vel[group.first] = sol->isTriggered()
                    ? solenoidLimit.max_velocity
                    : 0.0;
            }
            else if (controller->getType() == encoder::TYPE)
            {
                encoder* enc = dynamic_cast<encoder*>(controller.get());
                m_pos[group.first] = enc->getPos();
                
            }
            else if (controller->getType() == motor::TYPE)
            {
                motor* mtr = dynamic_cast<motor*>(controller.get());
                m_vel[group.first] = mtr->getVelocity();
            }
        }
    }
}

void hardware::write()
{
    for (auto group : m_groups)
    {
        for (auto controller: group.second)
        {
            if (controller->getType() == solenoid::TYPE && m_commands[group.first] > 0.0)
            {
                m_commands[group.first] = 0.0;

                solenoid* sol = dynamic_cast<solenoid*>(controller.get());
                sol->trigger();
            }
            else if (controller->getType() == motor::TYPE)
            {
                motor* mtr = dynamic_cast<motor*>(controller.get());
                mtr->command(m_commands[group.first]);
            }
        }
    }
}

void hardware::debug()
{
}

/*----------------------------------------------------------*\
| Node entry point implementation
\*----------------------------------------------------------*/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hardware");

    ros::NodeHandle node;
    ros::Rate rate(6);

    hardware hw(node);

    if (!hw.configure("robot") || !hw.init())
    {
        ROS_FATAL("  hardware failed to initialize");
        return 1;
    }

    while(node.ok())
    {
        hw.update();
        rate.sleep();
    }

    return 0;
}
