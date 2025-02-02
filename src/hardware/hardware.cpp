
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

    ros::param::get(m_namespace + "/publish_rate", m_rate);
    ros::param::get(m_namespace + "/debug", m_debug);

    // Load description

    string description;

    if (ros::param::get("robot_description", description))
    {
        urdf::Model model;

        if (model.initString(description))
        {
            // TODO: update how limits are loaded
            /*for (auto controller : m_controllers)
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
            }*/
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
    // Initialize hardware state

    /*for (auto group: m_groups)
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

        // Register position interface
        hardware_interface::JointHandle actuatorCommand(
            actuatorState,
            &m_cmd[group.first]
        );

        m_cmd[group.first] = 0.0;
        m_posInterface.registerHandle(actuatorCommand);

        // Register limits interface
        hardware_interface::JointStateHandle jointState(
            group.first,
            &m_pos[group.first],
            &m_vel[group.first],
            &m_effort[group.first]
        );

        hardware_interface::JointHandle jointHandle(
            jointState,
            &m_pos[group.first]
        );

        joint_limits_interface::VelocityJointSaturationHandle
            actuatorLimits(jointHandle, m_limits[group.first]);
        
        m_satInterface.registerHandle(actuatorLimits);
    }*/

    // Register hardware interfaces

    registerInterface(&m_stateInterface);
    registerInterface(&m_posInterface);
    registerInterface(&m_satInterface);

    return true;
}

void hardware::update()
{
    ros::Time time = ros::Time::now();
    ros::Duration period = time - m_lastUpdate;

    read();

    m_controllerManager.update(time, period);
    m_satInterface.enforceLimits(period);

    if (m_debug) debug();

    write();

    m_lastUpdate = time;
}

void hardware::read()
{
    // TODO
}

void hardware::write()
{
    // TODO
}

void hardware::debug()
{
    /*for (auto group : m_groups)
    {
        auto pos = m_pos[group.first];
        auto vel = m_vel[group.first];
        auto cmd = m_cmd[group.first];
        auto eff = m_effort[group.first];
        
        ROS_INFO_NAMED(
            "hardware",
            "%s: pos %g vel %g eff %g cmd %g",
            group.first.c_str(), pos, vel, eff, cmd);
    }*/
}

void hardware::run()
{
    ros::Rate rate(m_rate);
    ros::AsyncSpinner spinner(3);

    // Controller manager will deadlock with non-async spinner
    spinner.start();

    while(m_node.ok())
    {
        update();
        rate.sleep();
    }
}

/*----------------------------------------------------------*\
| Node entry point implementation
\*----------------------------------------------------------*/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hardware");

    ros::NodeHandle node;
    hardware hw(node, "robot");

    ROS_INFO("loading hardware");

    if (!hw.configure() || !hw.init())
    {
        ROS_FATAL("hardware failed to initialize");
        return 1;
    }

    ROS_INFO("hardware initialized successfully");

    hw.run();

    return 0;
}
