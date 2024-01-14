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

#include "trajectoryController.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace str1ker;

/*----------------------------------------------------------*\
| trajectoryController implementation
\*----------------------------------------------------------*/

bool trajectoryController::init(
    hardware_interface::RobotHW* hw,
    ros::NodeHandle& manager,
    ros::NodeHandle& controller)
{

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

trajectoryPoint* trajectoryController::sampleTrajectory(double timeFromStart)
{

}

void trajectoryController::endTrajectory()
{

}

void trajectoryController::debug()
{

}

