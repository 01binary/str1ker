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
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>

#include "controller.h"
#include "servo.h"
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

    // Names of joints to publish
    static const char* JOINTS[];

private:
    // Paths of joints to publish
    std::vector<std::string> m_jointPaths;

    // Joints
    std::vector<std::shared_ptr<servo>> m_joints;

    // Trigger solenoid
    std::shared_ptr<solenoid> m_trigger;

    // Joint positions
    std::vector<double> m_jointPositions;

    // Joint velocities
    std::vector<double> m_jointVelocities;

    // Joint efforts (not used)
    std::vector<double> m_jointEfforts;

    // Last joint position command
    std::vector<double> m_jointPosCommands;

    // Last joint velocity command
    std::vector<double> m_jointVelCommands;

    // Joint hardware interfaces
    controller_manager::ControllerManager m_hw;
    hardware_interface::JointStateInterface m_state;
    hardware_interface::PositionJointInterface m_position;
    hardware_interface::VelocityJointInterface m_velocity;
    joint_limits_interface::VelocityJointSaturationInterface m_limits;

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
