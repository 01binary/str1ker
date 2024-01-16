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

#include <actionlib/server/action_server.h>

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

    double pos = {0.0};
    double vel = {0.0};
    double posError = {0.0};
    double velError = {0.0};
    double command = {0.0};
  };

  struct waypoint_t
  {
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> time;
    std::vector<double> duration;
  };

  //
  // Constants
  //

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

  //
  // State
  //

  ros::NodeHandle m_node;
  ros::Subscriber m_goalSub;
  ros::Publisher m_statePub;
  ros::Publisher m_feedbackPub;
  ros::Publisher m_resultPub;
  std::shared_ptr<actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>> m_pGoalServer;
  ros::ServiceServer m_stateService;

  hardware_interface::VelocityJointInterface* m_hardware;

  trajectoryState m_state;
  std::vector<waypoint_t> m_trajectory;
  ros::Time m_startTime;
  ros::Time m_lastTime;

public:
  //
  // Initialization
  //

  bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& managerNode, ros::NodeHandle& node);

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
  void trajectoryActionCallback(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle goal);
  void trajectoryCancelCallback(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle goal);
  bool queryStateService(control_msgs::QueryTrajectoryState::Request& req,
                         control_msgs::QueryTrajectoryState::Response& res);

  
  //
  // Trajectory management
  //
  
  void parseTrajectory(const trajectory_msgs::JointTrajectory& trajectory);
  void beginTrajectory(const ros::Time& time, const std::vector<waypoint_t>& waypoints);
  void runTrajectory(const ros::Time& time, const ros::Duration& period);
  waypoint_t* sampleTrajectory(double timeFromStart);
  void endTrajectory();

  //
  // Debugging
  //

  void debug();
};

} // namespace str1ker
