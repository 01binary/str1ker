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

#include <vector>
#include <string>
#include <memory>

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

#include "controllerFactory.h"
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
    // Default update rate 50 Hz
    const double DEFAULT_RATE = 50;

private:
    // The namespace for loading settings
    std::string m_namespace;

    // ROS node
    ros::NodeHandle m_node;

    // Update rate
    double m_rate;

    // Velocity controllers
    controller_manager::ControllerManager m_controllerManager;

    // Hardware controllers
    controllerArray m_controllers;
    std::map<std::string, std::vector<std::shared_ptr<controller>>> m_groups;

    // Hardware state
    std::map<std::string, double> m_pos;
    std::map<std::string, double> m_vel;
    std::map<std::string, double> m_effort;
    std::map<std::string, joint_limits_interface::JointLimits> m_limits;
    std::map<std::string, double> m_cmd;

    // Hardware interfaces
    hardware_interface::JointStateInterface m_stateInterface;
    hardware_interface::VelocityJointInterface m_velInterface;
    joint_limits_interface::VelocityJointSaturationInterface m_satInterface;

    // Last update time
    ros::Time m_lastUpdate;

    // Debugging enabled
    bool m_debug;

public:
    // Constructor
    hardware(ros::NodeHandle node, std::string configNamespace);

public:
    // Load arm controller settings
    bool configure();

    // Initialize arm controllers
    bool init();

    // Update joints
    void update();

    // Run real-time loop
    void run();

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
