
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

hardware::hardware(ros::NodeHandle node, string configNamespace)
    : m_node(node)
    , m_namespace(configNamespace)
    , m_controllerManager(this, node)
    , m_rate(DEFAULT_RATE)
    , m_lastUpdate(0)
    , m_debug(false)
{
}

bool hardware::configure()
{
    // Load settings

    ros::param::get(m_namespace + "/rate", m_rate);
    ros::param::get(m_namespace + "/debug", m_debug);

    // Load controllers

    m_controllers = controllerFactory::fromNamespace(m_node, m_namespace);

    for (auto controller : m_controllers)
    {
        if (!controller->configure())
            return false;

        auto groupName = controller->getParentName();
        m_groups[groupName].push_back(controller);
    }

    // Load description

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
        ROS_WARN("no limits loaded for %s, could not found robot_description", m_namespace.c_str());
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
        m_cmd[group.first] = 0.0;

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
            &m_cmd[group.first]
        );

        m_cmd[group.first] = 0.0;
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

    if (m_debug) debug();

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
            if (controller->getType() == solenoid::TYPE && m_cmd[group.first] > 0.0)
            {
                m_cmd[group.first] = 0.0;

                solenoid* sol = dynamic_cast<solenoid*>(controller.get());
                sol->trigger();
            }
            else if (controller->getType() == motor::TYPE)
            {
                motor* mtr = dynamic_cast<motor*>(controller.get());
                mtr->command(m_cmd[group.first]);
            }
        }
    }
}

void hardware::debug()
{
    for (auto group : m_groups)
    {
        auto pos = m_pos[group.first];
        auto vel = m_vel[group.first];
        auto cmd = m_cmd[group.first];
        auto eff = m_effort[group.first];
        
        ROS_INFO_NAMED(
            "hardware",
            "%s: pos %g vel %g eff %g cmd %g",
            group.first.c_str(), pos, vel, eff, cmd);
    }
}

void hardware::run()
{
    ros::Rate rate(m_rate);

    while(m_node.ok())
    {
        update();
        ros::spinOnce();
        rate.sleep();
    }
}

/*----------------------------------------------------------*\
| Node entry point implementation
\*----------------------------------------------------------*/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hardware");

    puts("                                   █████                                              ");
    puts("                                   █   █                                              ");
    puts("                                   █   █                                              ");
    puts(" █       █    ████    ████████      █ █     █         █    ████    ████████  ████████ ");
    puts(" █████████  ████████  ████████      █ █     █    █    █  ████████  ████████  ████████ ");
    puts(" █       █ █        █ █       █     █ █      ████ ████  █        █ █       █ ████████ ");
    puts("                                  █     █                                             ");
    puts("                                   █   █                                              ");
    puts("                                  █  █  █                                             ");
    puts("                                  █  █  █     ██████                                  ");
    puts("                                  █  █  █   ██  ██  ██                                ");
    puts("                                   █████      ██████                                  ");

    ros::NodeHandle node;
    hardware hw(node, "robot");

    if (!hw.configure() || !hw.init())
    {
        ROS_FATAL("hardware failed to initialize");
        return 1;
    }

    hw.run();

    return 0;
}
