/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 arm.h

 Robot Arm Controller
 Created 1/19/2021

 Copyright (C) 2021 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/actuator_command_interface.h>
#include <actuator_limits_interface/actuator_limits_interface.h>
#include <actuator_limits_interface/actuator_limits.h>
#include <hardware_interface/robot_hw.h>

#include "controller.h"
#include "motor.h"
#include "solenoid.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| arm class
\*----------------------------------------------------------*/

class arm : public controller, public hardware_interface::RobotHW
{
public:
    // Controller type
    static const char TYPE[];

    // Names of actuators to publish
    static const char* ACTUATORS[];

private:
    // Paths of actuators to publish
    std::vector<std::string> m_actuatorPaths;

    // Actuator controllers
    std::vector<std::shared_ptr<motor>> m_actuators;

    // Trigger solenoid
    std::shared_ptr<solenoid> m_trigger;

    // Actuator positions
    std::vector<double> m_actuatorPos;

    // Actuator velocities
    std::vector<double> m_actuatorVel;

    // Actuator efforts (not used)
    std::vector<double> m_actuatorEfforts;

    // Actuator velocity commands
    std::vector<double> m_actuatorVelCommands;

    // Joint hardware interfaces
    controller_manager::ControllerManager m_controllers;
    hardware_interface::ActuatorStateInterface m_stateInterface;
    hardware_interface::VelocityActuatorInterface m_velInterface;
    actuator_limits_interface::VelocityJointSoftLimitsInterface m_limInterface;

    // Last update time
    ros::Time m_lastUpdate;

public:
    arm(class robot& robot, const char* path);

public:
    // Get display type
    virtual const char* getType();

    // Trigger arm
    void trigger(double durationSeconds = 0.023);

    // Load arm controller settings
    virtual void deserialize(ros::NodeHandle node);

    // Initialize arm controllers
    virtual bool init(ros::NodeHandle node);

    // Update joints
    virtual void update();

public:
    // Create instance
    static controller* create(class robot& robot, const char* path);
};

} // namespace str1ker
