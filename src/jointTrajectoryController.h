/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 jointTrajectoryController.h

 A "forgiving" joint trajectory controller with no hold trajectory
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
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <control_msgs/QueryTrajectoryState.h>

#include <controller_manager/controller_manager.h>
#include <controller_interface/controller.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <angles/angles.h>
#include <urdf/model.h>

/*----------------------------------------------------------*\
| Definitions
\*----------------------------------------------------------*/

typedef hardware_interface::VelocityJointInterface velocityHardware;
typedef controller_interface::Controller<velocityHardware> velocityController;
typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> trajectoryActionServer;

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| jointTrajectoryController class
\*----------------------------------------------------------*/

class jointTrajectoryController : public velocityController
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

    // Handle for sending velocity commands and reading encoders from robot hardware
    hardware_interface::JointHandle handle;

    // PID controller for calculating velocity
    control_toolbox::Pid pid;

    // Configuration from URDF
    double min;
    double max;
    double maxVelocity;

    // Configuration from YAML
    double tolerance;
    double timeout;

    // State
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
  std::shared_ptr<trajectoryActionServer> m_pGoalServer;
  velocityHardware* m_hardware;

  //
  // State
  //

  trajectoryState m_state;
  trajectoryActionServer::GoalHandle m_goal;
  std::vector<waypoint_t> m_trajectory;
  uint32_t m_seq;
  ros::Time m_startTime;
  ros::Time m_lastTime;

public:
  //
  // Initialization
  //

  bool init(velocityHardware* hw, ros::NodeHandle& managerNode, ros::NodeHandle& node);

  //
  // Lifecycle
  //

  void starting(const ros::Time& time);
  void stopping(const ros::Time&);
  void update(const ros::Time& time, const ros::Duration& period);

  //
  // ROS and MoveIt Interface
  //

  void trajectoryFeedback(const ros::Time& time, double trajectoryTime);
  void trajectoryGoalCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg);
  void trajectoryActionCallback(trajectoryActionServer::GoalHandle goal);
  void trajectoryCancelCallback(trajectoryActionServer::GoalHandle goal);
  bool trajectoryQueryCallback(control_msgs::QueryTrajectoryState::Request& req, control_msgs::QueryTrajectoryState::Response& res);

  //
  // Trajectory management
  //
  
  void parseTrajectory(const trajectory_msgs::JointTrajectory& trajectory);
  void beginTrajectory(const ros::Time& time, const std::vector<waypoint_t>& waypoints);
  void runTrajectory(const ros::Time& time, const ros::Duration& period);
  const waypoint_t* sampleTrajectory(double timeFromStart, std::vector<double>& position);
  void endTrajectory();
};

} // namespace str1ker
