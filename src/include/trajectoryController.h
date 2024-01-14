/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 trajectoryController.h

 Joint trajectory controller
 Created 01/14/2024

 Copyright (C) 2024 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <string>
#include <vector>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <control_msgs/QueryTrajectoryState.h>

#include <controller_interface/controller.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <angles/angles.h>
#include <urdf/model.h>

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| trajectoryController class
\*----------------------------------------------------------*/

class trajectoryController: public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
  //
  // Types
  //

  enum controllerState
  {
    READY,
    EXECUTING,
    DONE
  };

  enum supportedJointTypes
  {
    REVOLUTE = 1,
    PRISMATIC = 3
  };

  struct jointInfo
  {
    std::string name;
    supportedJointTypes type;
    double min;
    double max;
    double tolerance;
    double timeout;
  };

  struct trajectoryPoint
  {
    double position;
    double velocity;
    double duration;
  };

private:
  //
  // Configuration
  //

  std::string m_name;
  std::vector<jointInfo> m_jointInfo;

  //
  // State
  //

  ros::NodeHandle m_controller;
  ros::Subscriber m_goalSub;
  ros::Publisher m_statePub;
  ros::Publisher m_feedbackPub;
  ros::Publisher m_resultPub;

  hardware_interface::RobotHW* m_hardware;
  std::vector<hardware_interface::JointHandle> m_joints;

  controllerState m_state;
  std::vector<trajectoryPoint> m_trajectory;
  ros::Time m_startTime;
  ros::Time m_lastTime;
  std::vector<control_toolbox::Pid> m_pid;
  std::vector<double> m_positions;
  std::vector<double> m_errors;
  std::vector<double> m_commands;

public:
  //
  // Initialization
  //

  bool init(hardware_interface::RobotHW* hw, ros::NodeHandle& manager, ros::NodeHandle& controller);

  //
  // Lifecycle
  //

  void starting(const ros::Time& time);
  void stopping(const ros::Time&);
  void update(const ros::Time& time, const ros::Duration& period);

  //
  // Interface
  //

  void trajectoryCallback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg);
  void beginTrajectory(ros::Time& time, std::vector<trajectoryPoint> waypoints);
  void runTrajectory(ros::Time& time, const ros::Duration& period);
  trajectoryPoint* sampleTrajectory(double timeFromStart);
  void endTrajectory();

  //
  // Debugging
  //

  void debug();
};

} // namespace str1ker
