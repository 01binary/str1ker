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

#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <urdf/model.h>

#include <str1ker/PositionCommand.h>
#include <str1ker/StateFeedback.h>

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
    static const double DEFAULT_RATE;

private:
    // The namespace for loading settings
    std::string m_namespace;

    // ROS node
    ros::NodeHandle m_node;

    // Update rate
    double m_rate;

    // ROS controller manager
    controller_manager::ControllerManager m_controllerManager;

    // Joint names controlled by this hardware interface
    std::vector<std::string> m_jointNames;

    // Hardware state
    std::map<std::string, double> m_pos;
    std::map<std::string, double> m_vel;
    std::map<std::string, double> m_effort;
    std::map<std::string, joint_limits_interface::JointLimits> m_limits;
    std::map<std::string, double> m_cmd;
    std::map<std::string, double> m_feedbackPos;
    std::map<std::string, double> m_feedbackVel;

    // Hardware interfaces
    hardware_interface::JointStateInterface m_stateInterface;
    hardware_interface::PositionJointInterface m_posInterface;
    joint_limits_interface::PositionJointSaturationInterface m_satInterface;

    // ROS serial topics
    std::string m_positionTopic;
    std::string m_stateTopic;
    
    // ROS control controller namespace used for joint discovery
    std::string m_controllerName;

    // ROS serial connections
    ros::Publisher m_positionPublisher;
    ros::Subscriber m_stateSubscriber;

    // Last update time
    ros::Time m_lastUpdate;

    // Last state feedback time
    ros::Time m_lastFeedback;

    // Debugging enabled
    bool m_debug;

    // Guards state shared with subscriber callback
    mutable std::mutex m_stateMutex;

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
    // Discover joints from loaded ROS control parameters
    bool loadJointNamesFromParams(std::vector<std::string>& jointNames) const;

    // Load joint limits from robot_description URDF
    bool loadJointLimitsFromUrdf();

    // Read one joint from state feedback
    bool readJoint(
        const str1ker::StateFeedback& msg,
        const std::string& jointName,
        double& position,
        double& velocity) const;

    // Write one joint into outgoing position command
    bool writeJoint(
        str1ker::PositionCommand& msg,
        const std::string& jointName,
        double command) const;

    // Callback for incoming state feedback
    void stateCallback(const str1ker::StateFeedbackConstPtr& msg);

    // Read hardware state
    void read();

    // Send queued commands to hardware
    void write();

    // Output position and command for each joint
    void debug();
};

} // namespace str1ker

/*----------------------------------------------------------*\
| Node entry point
\*----------------------------------------------------------*/

int main(int argc, char** argv);
