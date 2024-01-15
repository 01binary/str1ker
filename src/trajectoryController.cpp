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

#include "include/trajectoryController.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace str1ker;

/*----------------------------------------------------------*\
| trajectoryController implementation
\*----------------------------------------------------------*/

bool trajectoryController::init(
    hardware_interface::VelocityJointInterface* hw,
    ros::NodeHandle& manager,
    ros::NodeHandle& controller)
{
  ROS_INFO("<<<<<<<<< My custom trajectory controller initializing");

  m_controller = controller;
  m_hardware = hw;

  // TODO set controller name from node handle namespace

  // TODO load settings

  // TODO load URDF

  // TODO get joint handles from hw
  for (int jointIndex = 0; jointIndex < m_jointInfo.size(); jointIndex++)
  {
    m_joints[jointIndex] = m_hardware->getHandle(m_jointInfo[jointIndex].name);
  }

  // Subscribe to trajectory goals
  m_goalSub = m_controller.subscribe<control_msgs::FollowJointTrajectoryActionGoal>(
    "follow_joint_trajectory", 1, trajectoryCallback
  );

  // Publish state
  m_statePub = m_controller.advertise<control_msgs::JointTrajectoryControllerState>(
    "state", 1
  );

  // Publish trajectory feedback
  ros::Publisher m_feedbackPub = m_controller.advertise<control_msgs::FollowJointTrajectoryFeedback>(
    "feedback", 1
  );

  // Publish trajectory result
  ros::Publisher m_resultPub = m_controller.advertise<control_msgs::FollowJointTrajectoryResult>(
    "result", 1
  );

  return true;
}

void trajectoryController::starting(const ros::Time& time)
{

}

void trajectoryController::stopping(const ros::Time&)
{

}

void trajectoryController::update(const ros::Time& time, const ros::Duration& period)
{

}

void trajectoryController::trajectoryCallback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg)
{

}

void trajectoryController::beginTrajectory(ros::Time& time, std::vector<trajectoryPoint> waypoints)
{

}

void trajectoryController::runTrajectory(ros::Time& time, const ros::Duration& period)
{

}

trajectoryController::trajectoryPoint* trajectoryController::sampleTrajectory(double timeFromStart)
{
    return nullptr;
}

void trajectoryController::endTrajectory()
{

}

void trajectoryController::debug()
{

}

