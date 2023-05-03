/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 context.cpp

 Motion Planning Context
 Created 05/01/2023

 Copyright (C) 2023 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include "include/context.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace std;
using namespace Eigen;
using namespace moveit::core;
using namespace planning_scene;
using namespace planning_interface;
using namespace robot_trajectory;
using namespace trajectory_msgs;
using namespace robot_state;
using namespace str1ker;

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const char* PluginContext::PLUGIN_NAME = "str1ker::PluginContext";

/*----------------------------------------------------------*\
| PluginContext implementation
\*----------------------------------------------------------*/

PluginContext::PluginContext(const string& group)
    : PlanningContext(string(PLUGIN_NAME), group)
{
}

PluginContext::~PluginContext()
{
}

bool PluginContext::solve(MotionPlanResponse& res)
{
    MotionPlanDetailedResponse detailed;
    bool success = solve(detailed);

    if (success)
    {
        res.trajectory_ = detailed.trajectory_.front();
        res.planning_time_ = detailed.processing_time_.front();
    }

    res.error_code_ = detailed.error_code_;

    return success;
}

bool PluginContext::solve(MotionPlanDetailedResponse& res)
{
    if (!request_.goal_constraints.size())
    {
        ROS_ERROR_NAMED(PLUGIN_NAME, "No goal constraints specified");
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
        return false;
    }

    auto pModel = planning_scene_->getRobotModel();
    auto pGroup = pModel->getJointModelGroup(group_);

    RobotStatePtr pStartState(new robot_state::RobotState(planning_scene_->getCurrentState()));
    robotStateMsgToRobotState(
        planning_scene_->getTransforms(),
        request_.start_state,
        *pStartState);

    RobotStatePtr pGoalState(new robot_state::RobotState(pModel));
    auto constraints = request_.goal_constraints.front().joint_constraints;
    double timeout = request_.allowed_planning_time;

    for (auto constraint: constraints)
    {
        pGoalState->setJointPositions(
            constraint.joint_name,
            &constraint.position);
    }

    pGoalState->enforceBounds(pGroup);
    pGoalState->update();

    RobotTrajectoryPtr trajectory(new RobotTrajectory(pModel, pGroup));
    trajectory->addPrefixWayPoint(pStartState, 0.0);
    trajectory->addSuffixWayPoint(pGoalState, 1.0);

    res.start_state_ = request_.start_state;
    res.description_.push_back("Str1ker Plan");
    res.processing_time_.push_back(0.0);
    res.trajectory_.push_back(trajectory);
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

    return true;
}

bool PluginContext::terminate()
{
    return true;
}

void PluginContext::clear()
{
}
