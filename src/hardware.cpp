
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
    }

    size_t numControllers = m_controllers.size();
    m_pos.resize(numControllers);
    m_vel.resize(numControllers);
    m_effort.resize(numControllers);
    m_commands.resize(numControllers);

    string description;

    if (ros::param::get("robot_description", description))
    {
        urdf::Model model;

        if (model.initString(description))
        {
            for (auto controller: m_controllers)
            {
                auto joint = model.getJoint(controller->getName());

                joint_limits_interface::JointLimits limits;

                if (!joint_limits_interface::getJointLimits(joint, limits))
                    ROS_WARN("no limits for %s in robot_description", controller->getPath());

                m_limits.push_back(limits);
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
    for (int n = 0; n < m_controllers.size(); n++)
    {
        // Initialize hardware controller
        if (!m_controllers[n]->init())
            return false;

        // Initialize hardware state
        m_pos[n] = 0.0;
        m_vel[n] = 0.0;
        m_effort[n] = 0.0;
        m_commands[n] = 0.0;

        // Register state interface
        hardware_interface::JointStateHandle actuatorState(
            m_controllers[n]->getName(),
            &m_pos[n],
            &m_vel[n],
            &m_effort[n]
        );

        m_stateInterface.registerHandle(actuatorState);

        // Register velocity interface
        hardware_interface::JointHandle actuatorVelocity(
            actuatorState,
            &m_commands[n]
        );

        m_commands[n] = 0.0;
        m_velInterface.registerHandle(actuatorVelocity);

        // Register limits interface
        hardware_interface::JointStateHandle jointState(
            m_controllers[n]->getName(),
            &m_pos[n],
            &m_vel[n],
            &m_effort[n]
        );

        hardware_interface::JointHandle jointHandle(
            jointState,
            &m_vel[n]
        );

        joint_limits_interface::VelocityJointSaturationHandle
            actuatorLimits(jointHandle, m_limits[n]);
        
        m_satInterface.registerHandle(actuatorLimits);
    }

    // Register interfaces for all joints
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
    for (int n = 0; n < m_controllers.size(); n++)
    {
        if (m_controllers[n]->getType() == solenoid::TYPE)
        {
            solenoid* sol = dynamic_cast<solenoid*>(m_controllers[n].get());
            auto solenoidLimit = m_limits[n];

            m_pos[n] = sol->isTriggered()
                ? solenoidLimit.max_position
                : solenoidLimit.min_position;

            m_vel[n] = sol->isTriggered()
                ? solenoidLimit.max_velocity
                : 0.0;
        }
        else if (m_controllers[n]->getType() == encoder::TYPE)
        {
            encoder* enc = dynamic_cast<encoder*>(m_controllers[n].get());
            m_pos[n] = enc->getPos();
            
        }
        else if (m_controllers[n]->getType() == motor::TYPE)
        {
            motor* mtr = dynamic_cast<motor*>(m_controllers[n].get());
            m_vel[n] = mtr->getVelocity();
        }
    }
}

void hardware::write()
{
    for (int n = 0; n < m_controllers.size(); n++)
    {
        if (m_controllers[n]->getType() == solenoid::TYPE && m_commands[n] > 0.0)
        {
            solenoid* sol = dynamic_cast<solenoid*>(m_controllers[n].get());
            sol->trigger();

            m_commands[n] = 0.0;
        }
        else if (m_controllers[n]->getType() == motor::TYPE)
        {
            motor* mtr = dynamic_cast<motor*>(m_controllers[n].get());
            mtr->command(m_commands[n]);
        }
    }
}

void hardware::debug()
{
    const double THRESHOLD = 0.01;

    static bool s_initialized = false;
    static double s_vel[3];
    static double s_pos[3];

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
        abs(m_pos[0] - s_pos[0]) >= THRESHOLD ||
        abs(m_pos[1] - s_pos[1]) >= THRESHOLD ||
        abs(m_pos[2] - s_pos[2]) >= THRESHOLD ||
        abs(m_commands[0] - s_vel[0]) >= THRESHOLD ||
        abs(m_commands[1] - s_vel[1]) >= THRESHOLD ||
        abs(m_commands[2] - s_vel[2]) >= THRESHOLD
    )
    {
        ROS_INFO(
            "   base [%g >> %g] shoulder [%g >> %g] elbow [%g >> %g]",
            m_pos[0],
            m_commands[0],
            m_pos[1],
            m_commands[1],
            m_pos[2],
            m_commands[2]
        );
    }

    s_pos[0] = m_pos[0];
    s_pos[1] = m_pos[1];
    s_pos[2] = m_pos[2];

    s_vel[0] = m_commands[0];
    s_vel[1] = m_commands[1];
    s_vel[2] = m_commands[2];
}

/*----------------------------------------------------------*\
| Node entry point implementation
\*----------------------------------------------------------*/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "solenoid_state_publisher");

    ros::NodeHandle node;
    ros::Rate rate(6);

    hardware hw(node);

    if (!hw.configure("robot") || !hw.init())
    {
        ROS_FATAL("  Hardware failed to initialize");
        return 1;
    }

    while(node.ok())
    {
        hw.update();
        rate.sleep();
    }

    return 0;
}
