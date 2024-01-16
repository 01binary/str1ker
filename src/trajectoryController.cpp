/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 trajectoryController.cpp

 Joint trajectory controller implementation
 Created 01/14/2024

 Copyright (C) 2024 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <urdf/model.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <pluginlib/class_list_macros.h>
#include "include/trajectoryController.h"
#include "controllerUtilities.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace std;
using namespace str1ker;

/*----------------------------------------------------------*\
| trajectoryController implementation
\*----------------------------------------------------------*/

bool trajectoryController::init(
    hardware_interface::VelocityJointInterface* hw,
    ros::NodeHandle& managerNode,
    ros::NodeHandle& node)
{
  m_node = node;
  m_hardware = hw;
  m_name = controllerUtilities::getControllerName(node.getNamespace());

  // Load joint names
  vector<string> jointNames;

  if (!node.getParam("joints", jointNames))
  {
    ROS_ERROR_NAMED(m_name.c_str(), "joints parameter is required");
    return false;
  }

  // Load description
  string description;

  if (!ros::param::get("robot_description", description))
  {
    ROS_ERROR_NAMED(m_name.c_str(), "robot description parameter is required");
    return false;
  }

  urdf::Model model;

  if (!model.initString(description))
  {
    ROS_ERROR_NAMED(m_name.c_str(), "failed to load %s", description.c_str());
    return false;
  }

  // Load joints
  for (auto jointName : jointNames)
  {
    auto jointModel = model.getJoint(jointName);

    joint_t joint;
    joint.name = jointName;
    joint.handle = m_hardware->getHandle(jointName);
    
    if (jointModel->type == urdf::Joint::PRISMATIC)
    {
      joint.type = supportedJointTypes::PRISMATIC;
    }
    else if (jointModel->type == urdf::Joint::REVOLUTE)
    {
      joint.type = supportedJointTypes::REVOLUTE;
    }
    else
    {
      ROS_WARN_NAMED(
        m_name.c_str(),
        "unsupported joint type for %s, only revolute and prismatic supported",
        jointName.c_str()
      );

      // Do not control unsupported joint types
      continue;
    }

    joint_limits_interface::JointLimits limits;

    if (!joint_limits_interface::getJointLimits(jointModel, limits))
    {
      ROS_WARN_NAMED(
        m_name.c_str(),
        "no limits for joint %s in robot_description",
        jointName.c_str()
      );

      // Do not control joints without limits specified
      continue;
    }

    joint.min = limits.min_position;
    joint.max = limits.max_position;
    joint.maxVelocity = limits.has_velocity_limits
      ? limits.max_velocity
      : 1.0;

    if (!node.getParam(string("constraints/") + jointName + "/goal", joint.tolerance))
    {
      if (!node.getParam(string("constraints/") + jointName + "/trajectory", joint.tolerance))
      {
        ROS_INFO_NAMED(
          m_name.c_str(),
          "no goal or trajectory tolerance for %s, default %g (meters or radians)",
          jointName.c_str(),
          DEFAULT_TOLERANCE
        );

        // Use default goal position tolerance for joints without tolerance specified
        // (Trajectory tolerance is not supported, only goal tolerance)
        joint.tolerance = DEFAULT_TOLERANCE;
      }
    }

    if (!node.getParam(string("constraints/") + jointName + "/time", joint.timeout))
    {
      ROS_INFO_NAMED(
        m_name.c_str(),
        "no goal time tolerance for %s, default %g seconds",
        jointName.c_str(),
        DEFAULT_TIMEOUT
      );

      // Use default goal time tolerance
      joint.timeout = DEFAULT_TIMEOUT;
    }

    if (!joint.pid.init(ros::NodeHandle(node, "gains/" + jointName)))
    {
      ROS_WARN_NAMED(
        m_name.c_str(),
        "no pid gains for %s, default %g %g %g",
        jointName.c_str(),
        DEFAULT_P,
        DEFAULT_I,
        DEFAULT_D
      );

      // Use default PID gains, clamp, and windup
      joint.pid.initPid(DEFAULT_P, DEFAULT_I, DEFAULT_D, 1.0, 0.0);
    }

    // Loaded joint successfully
    m_joints.push_back(joint);
  }

  // Subscribe to trajectory goals
  m_goalSub = m_node.subscribe<control_msgs::FollowJointTrajectoryActionGoal>(
    "follow_joint_trajectory", 1, &trajectoryController::trajectoryCallback, this
  );

  // Publish state
  m_statePub = m_node.advertise<control_msgs::JointTrajectoryControllerState>(
    "state", 1
  );

  // Publish trajectory feedback
  ros::Publisher m_feedbackPub = m_node.advertise<control_msgs::FollowJointTrajectoryFeedback>(
    "feedback", 1
  );

  // Publish trajectory result
  ros::Publisher m_resultPub = m_node.advertise<control_msgs::FollowJointTrajectoryResult>(
    "result", 1
  );

  // Initialization complete
  m_state = trajectoryState::READY;

  return true;
}

void trajectoryController::starting(const ros::Time& time)
{
  // Zero all joints and reset PID states
  for (auto joint : m_joints)
  {
    joint.pid.reset();
    joint.handle.setCommand(0.0);
  }
}

void trajectoryController::stopping(const ros::Time&)
{
  endTrajectory();
}

void trajectoryController::update(
  const ros::Time& time, const ros::Duration& period)
{
  runTrajectory(time, period);
}

void trajectoryController::trajectoryCallback(
  const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg)
{
  auto trajectory = msg->goal.trajectory;

  // Parse trajectory message joints
  vector<int> jointIndexes;

  for (auto jointName : trajectory.joint_names)
  {
    int foundJointIndex = -1;

    for (int jointIndex = 0; jointIndex < m_joints.size(); jointIndex++)
    {
      if (m_joints[jointIndex].name == jointName)
      {
        foundJointIndex = jointIndex;
        break;
      }
    }

    jointIndexes.push_back(foundJointIndex);
  }

  // Parse trajectory message waypoints
  vector<waypoint_t> waypoints(trajectory.points.size());
  double prevWaypointTime = 0.0;

  for (int waypointIndex = 0; waypointIndex < waypoints.size(); waypointIndex++)
  {
    auto sourceWaypoint = trajectory.points[waypointIndex];
    auto targetWaypoint = waypoints[waypointIndex];

    targetWaypoint.position.resize(jointIndexes.size());
    targetWaypoint.velocity.resize(jointIndexes.size());
    targetWaypoint.time.resize(jointIndexes.size());
    targetWaypoint.duration.resize(jointIndexes.size());

    for (int sourceJointIndex = 0; sourceJointIndex < jointIndexes.size(); sourceJointIndex++)
    {
      int targetJointIndex = jointIndexes[sourceJointIndex];
      if (targetJointIndex == -1) continue;

      double position = sourceWaypoint.positions[sourceJointIndex];
      double velocity = sourceWaypoint.velocities[sourceJointIndex];
      double waypointTime = sourceWaypoint.time_from_start.toSec();
      double duration = waypointTime - prevWaypointTime;
      prevWaypointTime = waypointTime;

      targetWaypoint.position[targetJointIndex] = position;
      targetWaypoint.velocity[targetJointIndex] = velocity;
      targetWaypoint.time[targetJointIndex] = waypointTime;
      targetWaypoint.duration[targetJointIndex] = duration;
    }
  }

  // Begin executing parsed trajectory
  beginTrajectory(ros::Time::now(), waypoints);
}

void trajectoryController::beginTrajectory(
  const ros::Time& time, const std::vector<waypoint_t>& waypoints)
{
  ROS_INFO_NAMED(
    m_name.c_str(),
    "starting trajectory with %d waypoints",
    (int)waypoints.size()
  );

  m_state = trajectoryState::EXECUTING;
  m_startTime = time;
  m_lastTime = time;
  m_trajectory = waypoints;
}

void trajectoryController::endTrajectory()
{
  for (auto joint : m_joints)
  {
    joint.handle.setCommand(0.0);
  }

  m_state = trajectoryState::DONE;
}

void trajectoryController::runTrajectory(
  const ros::Time& time, const ros::Duration& period)
{
  // TODO copy from motor_response
}

trajectoryController::waypoint_t* trajectoryController::sampleTrajectory(double timeFromStart)
{
  // TODO haven't done this before
  return nullptr;
}

void trajectoryController::debug()
{
  // TODO copy from motor_response
}

PLUGINLIB_EXPORT_CLASS(str1ker::trajectoryController, controller_interface::ControllerBase);
