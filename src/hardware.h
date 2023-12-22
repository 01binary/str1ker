/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 hardware.h
 Robot hardware drivers
 Created 12/22/2023
 Copyright (C) 2023 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <urdf/model.h>

#include "controller.h"
#include "motor.h"
#include "encoder.h"
#include "solenoid.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| hardware class
\*----------------------------------------------------------*/

class hardware : public hardware_interface::RobotHW
{
private:
    // ROS node
    ros::NodeHandle m_node;

    // Controller manager service
    controller_manager::ControllerManager m_controllerManager;

    // Paths of actuators to publish
    std::vector<std::string> m_actuatorPaths;

    // Joints
    std::vector<std::string> m_jointNames;

    // Actuator controllers (one for each joint)
    std::vector<std::shared_ptr<motor>> m_actuators;

    // Encoder controllers (one for each joint)
    std::vector<std::shared_ptr<encoder>> m_encoders;

    // Solenoid controller
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
    hardware_interface::JointStateInterface m_stateInterface;
    hardware_interface::VelocityJointInterface m_velInterface;
    joint_limits_interface::VelocityJointSaturationInterface m_satInterface;

    // Last update time
    ros::Time m_lastUpdate;

public:
    // Constructor
    hardware(ros::NodeHandle node);

public:
    // Load arm controller settings
    bool configure();

    // Initialize arm controllers
    bool init();

    // Update joints
    void update();

private:
    // Read hardware state
    void read();

    // Send queued commands to hardware
    void write();

    // Output velocity and state for each joint
    void debug();
};

} // namespace str1ker

/*----------------------------------------------------------*\
| Node entry point
\*----------------------------------------------------------*/

int main(int argc, char** argv);
