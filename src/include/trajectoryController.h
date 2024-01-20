/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 trajectoryController.h

 A "forgiving" joint trajectory controller
 Created 01/14/2024

 Copyright (C) 2024 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

//
// Standard Library
//

#include <string>
#include <vector>

//
// Actions and Messages
//

#include <actionlib/server/action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <control_msgs/QueryTrajectoryState.h>

//
// MoveIt! Integration
//

#include <moveit_ros_control_interface/ControllerHandle.h>

//
// ROS Controllers
//

#include <controller_manager/controller_manager.h>
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

class trajectoryController
  : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
private:
  //
  // Types
  //

  enum trajectoryState
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

  struct joint_t
  {
    std::string name;
    supportedJointTypes type;
    hardware_interface::JointHandle handle;
    control_toolbox::Pid pid;

    double min;
    double max;
    double maxVelocity;
    double tolerance;
    double timeout;

    double goal = {0.0};
    double pos = {0.0};
    double vel = {0.0};
    double error = {0.0};
    double command = {0.0};

    bool completed = {false};
  };

  struct waypoint_t
  {
    std::vector<double> position;
    double time;
  };

  //
  // Constants
  //

  const double DISCRETE_TOLERANCE = 0.02;
  const double DEFAULT_TOLERANCE = 0.01;
  const double DEFAULT_TIMEOUT = 1.0;
  const double DEFAULT_P = 10.0;
  const double DEFAULT_I = 1.0;
  const double DEFAULT_D = 1.0;

private:
  //
  // Configuration
  //

  std::string m_name;
  std::vector<joint_t> m_joints;
  bool m_debug = false;

  //
  // Interface
  //

  ros::NodeHandle m_node;
  ros::Subscriber m_goalSub;
  ros::Publisher m_statePub;
  ros::Publisher m_feedbackPub;
  ros::Publisher m_resultPub;
  ros::ServiceServer m_stateService;
  std::shared_ptr<actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>> m_pGoalServer;
  hardware_interface::VelocityJointInterface* m_hardware;

  //
  // State
  //

  trajectoryState m_state;
  actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle m_goal;
  std::vector<waypoint_t> m_trajectory;
  ros::Time m_startTime;
  ros::Time m_lastTime;

public:
  //
  // Initialization
  //

  bool init(
    hardware_interface::VelocityJointInterface* hw,
    ros::NodeHandle& managerNode,
    ros::NodeHandle& node);

  //
  // Lifecycle
  //

  void starting(const ros::Time& time);
  void stopping(const ros::Time&);
  void update(const ros::Time& time, const ros::Duration& period);

  //
  // Interface
  //

  void trajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg);
  void trajectoryActionCallback(
    actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle goal);
  void trajectoryCancelCallback(
    actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle goal);
  bool queryStateService(control_msgs::QueryTrajectoryState::Request& req,
                         control_msgs::QueryTrajectoryState::Response& res);

  
  //
  // Trajectory management
  //
  
  void parseTrajectory(const trajectory_msgs::JointTrajectory& trajectory);
  void beginTrajectory(const ros::Time& time, const std::vector<waypoint_t>& waypoints);
  void runTrajectory(const ros::Time& time, const ros::Duration& period);
  const waypoint_t* sampleTrajectory(double timeFromStart, std::vector<double>& position);
  void endTrajectory();
};

/*----------------------------------------------------------*\
| trajectoryControllerHandle class
\*----------------------------------------------------------*/

class trajectoryControllerHandle
  : public moveit_controller_manager::MoveItControllerHandle
{
private:
  bool m_done;
  std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> m_actionClient;

public:
  trajectoryControllerHandle(const std::string& name, const std::string& action_ns);

public:
  bool sendTrajectory(const moveit_msgs::RobotTrajectory& trajectory) override;
  bool waitForExecution(const ros::Duration& timeout = ros::Duration(0)) override;
  moveit_controller_manager::ExecutionStatus getLastExecutionStatus() override;
  bool cancelExecution() override;
};

/*----------------------------------------------------------*\
| trajectoryControllerAllocator class
\*----------------------------------------------------------*/

class trajectoryControllerAllocator : public moveit_ros_control_interface::ControllerHandleAllocator
{
public:
  moveit_controller_manager::MoveItControllerHandlePtr alloc(
    const std::string& name, const std::vector<std::string>& resources) override;
};

} // namespace str1ker
