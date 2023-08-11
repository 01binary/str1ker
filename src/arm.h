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
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <urdf/model.h>

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

private:
    // Paths of actuators to publish
    std::vector<std::string> m_actuatorPaths;

    // Actuator controllers
    std::vector<std::shared_ptr<motor>> m_actuators;
    std::shared_ptr<solenoid> m_solenoid;

    // Actuator positions
    std::vector<double> m_actuatorPos;

    // Actuator velocities
    std::vector<double> m_actuatorVel;

    // Actuator limits
    std::vector<joint_limits_interface::JointLimits> m_actuatorLimits;

    // Actuator efforts (not used)
    std::vector<double> m_actuatorEfforts;

    // Actuator velocity commands
    std::vector<double> m_actuatorVelCommands;

    // Joint hardware interfaces
    controller_manager::ControllerManager m_controllers;
    hardware_interface::ActuatorStateInterface m_stateInterface;
    hardware_interface::VelocityActuatorInterface m_velInterface;
    joint_limits_interface::VelocityJointSaturationInterface m_satInterface;

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
    virtual void configure(ros::NodeHandle node);

    // Initialize arm controllers
    virtual bool init(ros::NodeHandle node);

    // Update joints
    virtual void update();

private:
    // Read hardware state
    void read();

    // Send queued commands to hardware
    void write();

public:
    // Create instance
    static controller* create(class robot& robot, const char* path);
};

} // namespace str1ker
