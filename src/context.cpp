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

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include "include/context.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/robot_state.h>

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace std;
using namespace moveit::core;
using namespace moveit_msgs;
using namespace planning_scene;
using namespace planning_interface;
using namespace robot_trajectory;
using namespace trajectory_msgs;
using namespace robot_state;
using namespace Eigen;
using namespace str1ker;

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const char* PluginContext::PLUGIN_NAME = "str1ker::PluginContext";

/*----------------------------------------------------------*\
| PluginContext implementation
\*----------------------------------------------------------*/

PluginContext::PluginContext(const std::string& group)
    : PlanningContext(PLUGIN_NAME, group)
{
}

PluginContext::~PluginContext()
{
}

bool PluginContext::solve(MotionPlanResponse& res)
{
    MotionPlanDetailedResponse detailed;
    bool success = solve(detailed);

    res.error_code_ = detailed.error_code_;

    if (success)
    {
        res.trajectory_ = detailed.trajectory_.front();
        res.planning_time_ = detailed.processing_time_.front();
    }

    return success;
}

bool PluginContext::solve(MotionPlanDetailedResponse& res)
{
    if (!validateRequest()) return false;

    const RobotModelConstPtr& model = planning_scene_->getRobotModel();
    const JointModelGroup* pGroup = model->getJointModelGroup(group_);

    const vector<const JointModel*> joints = pGroup->getJointModels();
    size_t numSupportedJoints = count_if(joints.begin(), joints.end(),
        [](const JointModel* pJoint)
        {
            return pJoint->getType() == JointModel::REVOLUTE ||
                   pJoint->getType() == JointModel::PRISMATIC;
        });

    vector<double> solution;
    solution.resize(numSupportedJoints);

    RobotStatePtr pGoalState(new robot_state::RobotState(robot_model_));
    pGoalState->setFromIK(
        pGroup,
        extractGoalPose(),
        request_.allowed_planning_time,
        solution);

    RobotTrajectoryPtr trajectory(new RobotTrajectory(model, pGroup));
    trajectory->addPrefixWayPoint(request_.start_state, 0.0);
    trajectory->addSuffixWayPoint(pGoalState, 1.0);

    res.start_state_ = request_.start_state;
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    res.description_.push_back("PlannerPlugin");
    res.processing_time_.push_back(0.0);
    res.trajectory_.push_back(trajectory);

    return true;
}

Isometry3d PluginContext::extractGoalPose()
{
    const Vector3d& position = request_
        .goal_constraints
        .front()
        .position_constraints
        .front()
        .constraint_region
        .primitive_poses
        .front()
        .position;

    return Eigen::Translation3d(
        position.x(),
        position.y(),
        position.z());
}

bool PluginContext::validateRequest()
{
    if (request_.goal_constraints.size() != 1)
    {
        ROS_ERROR_NAMED(PLUGIN_NAME,
            "Goal constraints do not specify a goal");
        return false;
    }

    const Constraints& constraints = request_.goal_constraints.front();

    if (constraints.position_constraints.size() != 1)
    {
        ROS_ERROR_NAMED(PLUGIN_NAME,
            "Goal constraints do not specify a pose constraint");
        return false;
    }

    const PositionConstraint& positionConstraint = constraints
        .position_constraints
        .front();

    if (positionConstraint.constraint_region.primitives.size() != 1)
    {
        ROS_ERROR_NAMED(PLUGIN_NAME,
            "Goal constraints do not specify a primitive pose constraint");
        return false;
    }

    return true;
}

bool PluginContext::terminate()
{
    return true;
}

void PluginContext::clear()
{
}
