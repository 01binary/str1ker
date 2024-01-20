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
#include "hardwareUtilities.h"
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

  // Load debug flag
  node.getParam("debug", m_debug);

  if (m_debug)
  {
    ROS_INFO_NAMED(m_name.c_str(), "Trajectory debugging enabled");
  }

  // Load joint names
  vector<string> jointNames;

  if (!node.getParam("joints", jointNames))
  {
    ROS_ERROR_NAMED(m_name.c_str(), "The joints parameter is required");
    return false;
  }

  // Load description
  string description;

  if (!ros::param::get("robot_description", description))
  {
    ROS_ERROR_NAMED(m_name.c_str(), "The robot_description parameter is required");
    return false;
  }

  urdf::Model model;

  if (!model.initString(description))
  {
    ROS_ERROR_NAMED(m_name.c_str(), "Failed to load %s", description.c_str());
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
        "Unsupported joint type for %s, only revolute and prismatic supported",
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
        "No limits for joint %s in robot_description",
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
          "No goal or trajectory tolerance for %s, default %g (meters or radians)",
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
        "No goal time tolerance for %s, default %g seconds",
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
        "No pid gains for %s, default %g %g %g",
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
  m_goalSub = m_node.subscribe(
    "command", 1, &trajectoryController::trajectoryCallback, this
  );

  // Publish state
  m_statePub = m_node.advertise<control_msgs::JointTrajectoryControllerState>(
    "state", 1
  );

  // Publish trajectory feedback
  m_feedbackPub = m_node.advertise<control_msgs::FollowJointTrajectoryFeedback>(
    "feedback", 1
  );

  // Publish trajectory result
  m_resultPub = m_node.advertise<control_msgs::FollowJointTrajectoryResult>(
    "result", 1
  );

  // Start trajectory goal action server
  m_pGoalServer.reset(new actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>(
    m_node,
    "follow_joint_trajectory",
    bind(&trajectoryController::trajectoryActionCallback, this, placeholders::_1),
    bind(&trajectoryController::trajectoryCancelCallback, this, placeholders::_1),
    false
  ));

  m_pGoalServer->start();

  // Advertise trajectory state service
  m_stateService = m_node.advertiseService(
    "query_state",
    &trajectoryController::queryStateService,
    this
  );

  // Initialization complete
  m_state = trajectoryState::READY;

  ROS_INFO_NAMED(m_name, "Trajectory controller plugin initialized");

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

void trajectoryController::update(const ros::Time& time, const ros::Duration& period)
{
  if (m_state == trajectoryState::EXECUTING)
  {
    runTrajectory(time, period);
  }
}

void trajectoryController::trajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
  parseTrajectory(*msg);
}

void trajectoryController::trajectoryActionCallback(
  actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle goal)
{
  if (!this->isRunning())
  {
    ROS_ERROR_NAMED(m_name, "Can't accept new action goals. Controller is not running.");
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    goal.setRejected(result);
    return;
  }

  parseTrajectory(goal.getGoal()->trajectory);
  goal.setAccepted();
  m_goal = goal;
}

void trajectoryController::trajectoryCancelCallback(
  actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle goal)
{
  endTrajectory();
}

bool trajectoryController::queryStateService(
  control_msgs::QueryTrajectoryState::Request& req,
  control_msgs::QueryTrajectoryState::Response& res)
{
  if (!this->isRunning())
  {
    ROS_ERROR_NAMED(m_name, "Can't sample trajectory. Controller is not running.");
    return false;
  }

  res.name.resize(m_joints.size());
  res.position.resize(m_joints.size());
  res.velocity.resize(m_joints.size());
  res.acceleration.resize(m_joints.size());

  vector<double> position;
  auto waypoint = sampleTrajectory(req.time.toSec(), position);
  if (!waypoint) return false;

  for (int jointIndex = 0; jointIndex < m_joints.size(); jointIndex++)
  {
    res.name[jointIndex] = m_joints[jointIndex].name;
    res.position[jointIndex] = position[jointIndex];
    res.velocity[jointIndex] = 0.0;
    res.acceleration[jointIndex] = 0.0;
  }

  return true;
}

void trajectoryController::parseTrajectory(const trajectory_msgs::JointTrajectory& trajectory)
{
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

    if (foundJointIndex == -1)
    {
      // Do not support joints that are not available
      ROS_ERROR_NAMED(
        m_name.c_str(),
        "Found joint %s in trajectory not claimed by this controller, aborting",
        jointName.c_str());

      return;
    }

    jointIndexes.push_back(foundJointIndex);
  }

  // Parse trajectory message waypoints
  vector<waypoint_t> waypoints;
  double prevWaypointTime = 0.0;

  for (int waypointIndex = 0; waypointIndex < trajectory.points.size(); waypointIndex++)
  {
    const trajectory_msgs::JointTrajectoryPoint& sourceWaypoint = trajectory.points[waypointIndex];
    double waypointTime = sourceWaypoint.time_from_start.toSec();

    if (waypointIndex && waypointTime - prevWaypointTime < DISCRETE_TOLERANCE)
      continue;

    waypoint_t targetWaypoint;
    targetWaypoint.time = waypointTime;
    targetWaypoint.position.resize(jointIndexes.size());

    for (int sourceJointIndex = 0; sourceJointIndex < jointIndexes.size(); sourceJointIndex++)
    {
      int targetJointIndex = jointIndexes[sourceJointIndex];
      targetWaypoint.position[targetJointIndex] = sourceWaypoint.positions[sourceJointIndex];
    }
 
    prevWaypointTime = targetWaypoint.time;

    waypoints.push_back(targetWaypoint);
  }

  // Begin executing parsed trajectory
  beginTrajectory(ros::Time::now(), waypoints);
}

void trajectoryController::beginTrajectory(
  const ros::Time& time, const std::vector<waypoint_t>& waypoints)
{
  ROS_INFO_NAMED(
    m_name.c_str(),
    "Starting trajectory with %d waypoints",
    (int)waypoints.size()
  );

  m_state = trajectoryState::EXECUTING;
  m_startTime = time;
  m_lastTime = time;
  m_trajectory = waypoints;

  for (joint_t& joint : m_joints)
  {
    joint.pos = joint.handle.getPosition();
    joint.vel = joint.handle.getVelocity();
    joint.error = 0.0;
    joint.command = 0.0;
    joint.completed = false;

    if (!waypoints.empty())
    {
      joint.goal = waypoints.front().position[&joint - &m_joints.front()];
    }
  }
}

void trajectoryController::endTrajectory()
{
  m_state = trajectoryState::DONE;

  ROS_INFO_NAMED(m_name.c_str(), "Ending trajectory");

  for (auto joint : m_joints)
  {
    joint.handle.setCommand(0.0);
  }

  control_msgs::FollowJointTrajectoryResult result;
  result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;

  // Notify action server goal is completed
  if (m_goal.isValid())
  {
    m_goal.setSucceeded(result);
  }

  // Publish trajectory result
  m_resultPub.publish(result);
}

void trajectoryController::runTrajectory(const ros::Time& time, const ros::Duration& period)
{
  vector<double> position;
  double trajectoryTime = (time - m_startTime).toSec();

  auto waypoint = sampleTrajectory(trajectoryTime, position);
  if (!waypoint) return;

  bool isLastWaypoint = waypoint - &m_trajectory.front() == m_trajectory.size() - 1;

  // Read joints
  for (joint_t& joint : m_joints)
  {
    // Update joint position
    joint.pos = joint.handle.getPosition();

    // Update joint velocity
    joint.vel = ceil(joint.handle.getVelocity() * 100.0) / 100.0;
  }

  // Write joints
  int completed = 0;

  for (joint_t& joint : m_joints)
  {
    if (joint.completed)
    {
      // Joint completed its trajectory or timed out
      completed++;
      continue;
    }

    // Get goal position
    joint.goal = waypoint->position[&joint - &m_joints.front()];

    // Calculate position error
    if (joint.type == supportedJointTypes::REVOLUTE)
    {
      angles::shortest_angular_distance_with_large_limits(
        joint.pos, joint.goal, joint.min, joint.max, joint.error);
    }
    else if (joint.type == supportedJointTypes::PRISMATIC)
    {
      joint.error = joint.goal - joint.pos;
    }

    // Stop if executed trajectory and ended within goal tolerance
    if (isLastWaypoint && abs(joint.error) <= joint.tolerance)
    {
      joint.completed = true;

      ROS_INFO_NAMED(
        m_name.c_str(),
        "Joint %s trajectory completed: position %g within %g of %g, tolerance %g",
        joint.name.c_str(),
        joint.pos,
        abs(joint.error),
        joint.goal,
        joint.tolerance
      );

      continue;
    }

    // Stop if exceeded time tolerance
    if (trajectoryTime >= joint.timeout)
    {
      joint.completed = true;

      ROS_INFO_NAMED(
        m_name.c_str(),
        "Joint %s trajectory timeout %g seconds",
        joint.name.c_str(),
        joint.timeout
      );

      continue;
    }

    // Calculate command
    joint.command = clamp(
      joint.pid.computeCommand(joint.error, period),
      -joint.maxVelocity,
      joint.maxVelocity
    );

    joint.handle.setCommand(joint.command);
  }

  // Publish state
  control_msgs::JointTrajectoryControllerState trajectoryState;
  trajectoryState.header.stamp = time; // TODO seq and frame id
  trajectoryState.joint_names.resize(m_joints.size());

  trajectoryState.actual.time_from_start = ros::Duration(trajectoryTime);
  trajectoryState.actual.positions.resize(m_joints.size());
  trajectoryState.actual.velocities.resize(m_joints.size());
  trajectoryState.actual.accelerations.resize(m_joints.size());
  trajectoryState.actual.effort.resize(m_joints.size());

  trajectoryState.desired.time_from_start = ros::Duration(trajectoryTime);
  trajectoryState.desired.positions.resize(m_joints.size());
  trajectoryState.desired.velocities.resize(m_joints.size());
  trajectoryState.desired.accelerations.resize(m_joints.size());
  trajectoryState.desired.effort.resize(m_joints.size());

  trajectoryState.error.time_from_start = ros::Duration(trajectoryTime);
  trajectoryState.error.positions.resize(m_joints.size());
  trajectoryState.error.velocities.resize(m_joints.size());
  trajectoryState.error.accelerations.resize(m_joints.size());
  trajectoryState.error.effort.resize(m_joints.size());

  for (int jointIndex = 0; jointIndex < m_joints.size(); jointIndex++)
  {
    trajectoryState.joint_names[jointIndex] = m_joints[jointIndex].name;

    trajectoryState.actual.positions[jointIndex] = m_joints[jointIndex].pos;
    trajectoryState.actual.velocities[jointIndex] = m_joints[jointIndex].vel;
    trajectoryState.actual.accelerations[jointIndex] = 0.0;
    trajectoryState.actual.effort[jointIndex] = 0.0;

    trajectoryState.desired.positions[jointIndex] = m_joints[jointIndex].goal;
    trajectoryState.desired.velocities[jointIndex] = m_joints[jointIndex].vel;
    trajectoryState.desired.accelerations[jointIndex] = 0.0;
    trajectoryState.desired.effort[jointIndex] = 0.0;

    trajectoryState.error.positions[jointIndex] = m_joints[jointIndex].error;
    trajectoryState.error.velocities[jointIndex] = 0.0;
    trajectoryState.error.accelerations[jointIndex] = 0.0;
    trajectoryState.error.effort[jointIndex] = 0.0;
  }

  m_statePub.publish(trajectoryState);

  // Publish feedback
  control_msgs::FollowJointTrajectoryFeedback trajectoryFeedback;
  trajectoryFeedback.header.stamp = time; // TODO seq and frame id
  trajectoryFeedback.joint_names.resize(m_joints.size());
  
  trajectoryFeedback.actual.time_from_start = ros::Duration(trajectoryTime);
  trajectoryFeedback.actual.positions.resize(m_joints.size());
  trajectoryFeedback.actual.velocities.resize(m_joints.size());
  trajectoryFeedback.actual.accelerations.resize(m_joints.size());
  trajectoryFeedback.actual.effort.resize(m_joints.size());

  trajectoryFeedback.desired.time_from_start = ros::Duration(trajectoryTime);
  trajectoryFeedback.desired.positions.resize(m_joints.size());
  trajectoryFeedback.desired.velocities.resize(m_joints.size());
  trajectoryFeedback.desired.accelerations.resize(m_joints.size());
  trajectoryFeedback.desired.effort.resize(m_joints.size());

  trajectoryFeedback.error.time_from_start = ros::Duration(trajectoryTime);
  trajectoryFeedback.error.positions.resize(m_joints.size());
  trajectoryFeedback.error.velocities.resize(m_joints.size());
  trajectoryFeedback.error.accelerations.resize(m_joints.size());
  trajectoryFeedback.error.effort.resize(m_joints.size());

  for (int jointIndex = 0; jointIndex < m_joints.size(); jointIndex++)
  {
    trajectoryFeedback.joint_names[jointIndex] = m_joints[jointIndex].name;

    trajectoryFeedback.actual.positions[jointIndex] = m_joints[jointIndex].pos;
    trajectoryFeedback.actual.velocities[jointIndex] = m_joints[jointIndex].vel;
    trajectoryFeedback.actual.accelerations[jointIndex] = 0.0;
    trajectoryFeedback.actual.effort[jointIndex] = 0.0;

    trajectoryFeedback.desired.positions[jointIndex] = m_joints[jointIndex].goal;
    trajectoryFeedback.desired.velocities[jointIndex] = m_joints[jointIndex].vel;
    trajectoryFeedback.desired.accelerations[jointIndex] = 0.0;
    trajectoryFeedback.desired.effort[jointIndex] = 0.0;

    trajectoryFeedback.error.positions[jointIndex] = m_joints[jointIndex].error;
    trajectoryFeedback.error.velocities[jointIndex] = 0.0;
    trajectoryFeedback.error.accelerations[jointIndex] = 0.0;
    trajectoryFeedback.error.effort[jointIndex] = 0.0;
  }

  m_feedbackPub.publish(trajectoryFeedback);

  if (m_goal.isValid())
  {
    m_goal.publishFeedback(trajectoryFeedback);
  }

  // Emit current state
  if (m_debug)
  {
    int trajectoryIndex = waypoint - &m_trajectory.front();

    ROS_INFO(
      "[waypoint %d] time %#.4g",
      trajectoryIndex,
      trajectoryTime
    );

    for (const joint_t& joint : m_joints)
    {
      ROS_INFO(
        "\t%-24.24spos %#+.3g\tvel %#+.3g\tcmd %#+.3g\t%s goal %#+.3g\terr %#+.3g\t%s",
        joint.name.c_str(),
        joint.pos,
        joint.vel,
        joint.command,
        joint.command > 0.0 ? "->" : joint.command < 0.0 ? "<-" : "  ",
        joint.goal,
        ceil(joint.error * 1000.0) / 1000.0,
        joint.completed ? "done" : ""
      );
    }
  }

  // End trajectory if all joints reached goal positions or timed out
  if (completed == m_joints.size())
  {
    endTrajectory();
  }
}

const trajectoryController::waypoint_t* trajectoryController::sampleTrajectory(
  double timeFromStart, vector<double>& position)
{
  if (m_trajectory.empty()) return nullptr;

  for (const waypoint_t& waypoint : m_trajectory)
  {
    if (waypoint.time >= timeFromStart)
    {
      int nextIndex = &waypoint - &m_trajectory.front() + 1;
      
      if (nextIndex >= m_trajectory.size())
      {
        // Clamp position when out of trajectory bounds
        position = waypoint.position;
      }
      else
      {
        // Interpolate position of each joint when between waypoints
        position.resize(waypoint.position.size());

        for (int jointIndex = 0; jointIndex < waypoint.position.size(); jointIndex++)
        {
          double x = timeFromStart;
          double x1 = waypoint.time;
          double y1 = waypoint.position[jointIndex];
          double x2 = m_trajectory[nextIndex].time;
          double y2 = m_trajectory[nextIndex].position[jointIndex];

          position[jointIndex] = y1 + (x - x1) * ((y2 - y1) / (x2 - x1));
        }
      }

      return &waypoint;
    }
  }

  // Clamp position when out of trajectory bounds
  position = m_trajectory.back().position;
  return &m_trajectory.back();
}

/*----------------------------------------------------------*\
| trajectoryControllerHandle implementation
\*----------------------------------------------------------*/

trajectoryControllerHandle::trajectoryControllerHandle(const string& name, const string& action_ns)
  : moveit_controller_manager::MoveItControllerHandle(name)
  , m_done(true)
{
  string actionName = name + "/" + action_ns;

  m_actionClient = make_shared<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>(
    actionName, true);

  m_actionClient->waitForServer(ros::Duration(20.0));

  if (!m_actionClient->isServerConnected())
  {
    ROS_ERROR_NAMED(
      getName().c_str(),
      "Action client failed to connect to %s",
      actionName.c_str());

    m_actionClient.reset();
  }
}

bool trajectoryControllerHandle::sendTrajectory(const moveit_msgs::RobotTrajectory& trajectory)
{
  if (!m_actionClient)
  {
    ROS_ERROR_NAMED(
      getName().c_str(),
      "Action client not connected, could not send trajectory");

    return false;
  }

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory.joint_trajectory;

  m_actionClient->sendGoal(
    goal,
    [this](const auto& state, const auto& result)
    {
      ROS_DEBUG_NAMED(getName().c_str(), "Trajectory Controller Completed Trajectory");
      m_done = true;
    },
    [this]
    {
      ROS_DEBUG_NAMED(getName().c_str(), "Trajectory Controller Beginning Trajectory");
    },
    [this](const auto& feedback)
    {
      ROS_DEBUG_NAMED(getName().c_str(), "Trajectory Controller Received Feedback");
    });

  m_done = false;

  return true;
}

bool trajectoryControllerHandle::waitForExecution(const ros::Duration& timeout)
{
  if (m_actionClient && !m_done)
    return m_actionClient->waitForResult(ros::Duration(5.0));

  return true;
}

moveit_controller_manager::ExecutionStatus trajectoryControllerHandle::getLastExecutionStatus()
{
  return moveit_controller_manager::ExecutionStatus::SUCCEEDED;
}

bool trajectoryControllerHandle::cancelExecution()
{
  if (!m_actionClient)
    return false;

  m_actionClient->cancelGoal();
  m_done = true;

  return true;
}

/*----------------------------------------------------------*\
| trajectoryControllerAllocator implementation
\*----------------------------------------------------------*/

moveit_controller_manager::MoveItControllerHandlePtr trajectoryControllerAllocator::alloc(
  const string& name, const vector<string>& resources)
{
  return make_shared<trajectoryControllerHandle>(
    name, "follow_joint_trajectory");
}

/*----------------------------------------------------------*\
| Exports
\*----------------------------------------------------------*/

PLUGINLIB_EXPORT_CLASS(str1ker::trajectoryController, controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(str1ker::trajectoryControllerAllocator, moveit_ros_control_interface::ControllerHandleAllocator);
