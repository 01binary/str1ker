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

const int PluginContext::STEPS = 16;
const double PluginContext::STEP_DURATION = 0.0;
const size_t PluginContext::QUINTIC_COEFFICIENTS = 6;
const char* PluginContext::PLUGIN_NAME = "str1ker::PluginContext";

/*----------------------------------------------------------*\
| Types
\*----------------------------------------------------------*/

union QuinticSplineCoefficients
{
    struct
    {
        double a, b, c, d, e, g;
    };

    double all[6];
};

/*----------------------------------------------------------*\
| PluginContext implementation
\*----------------------------------------------------------*/

PluginContext::PluginContext(const string& group)
    : PlanningContext(string(PLUGIN_NAME), group),
      m_useQuinticInterpolation(false)
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
    auto pStartState = getStartState();
    auto pGoalState = getGoalState();
    auto constraints = request_.goal_constraints.front().joint_constraints;
    double timeout = request_.allowed_planning_time;

    RobotTrajectoryPtr trajectory(new RobotTrajectory(pModel, pGroup));
    vector<RobotStatePtr> jointTrajectories;
    
    if (m_useQuinticInterpolation)
    {
        jointTrajectories = interpolateQuintic(constraints, pStartState, pGoalState, STEPS);
    }
    else
    {
        jointTrajectories = interpolateLinear(constraints, pStartState, pGoalState, STEPS);
    }

    trajectory->clear();
    trajectory->addPrefixWayPoint(pStartState, 0.0);

    for (const RobotStatePtr& pState: jointTrajectories)
    {
        trajectory->addSuffixWayPoint(pState, STEP_DURATION);
    }

    trajectory->addSuffixWayPoint(pGoalState, STEP_DURATION);

    res.start_state_ = request_.start_state;
    res.description_.push_back("plan");
    res.processing_time_.push_back(0.0);
    res.trajectory_.push_back(trajectory);
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

    return true;
}

RobotStatePtr PluginContext::getStartState() const
{
    const RobotState& currentState = planning_scene_->getCurrentState();
    RobotStatePtr pStartState(new robot_state::RobotState(currentState));
    return pStartState;
}

RobotStatePtr PluginContext::getGoalState() const
{
    auto pModel = planning_scene_->getRobotModel();
    auto pGroup = pModel->getJointModelGroup(group_);
    RobotStatePtr pGoalState(new robot_state::RobotState(pModel));
    auto constraints = request_.goal_constraints.front().joint_constraints;

    for (auto constraint: constraints)
    {
        pGoalState->setJointPositions(constraint.joint_name, &constraint.position);
    }

    pGoalState->enforceBounds(pGroup);
    pGoalState->update();

    return pGoalState;
}

vector<RobotStatePtr> PluginContext::interpolateQuintic(
    const vector<moveit_msgs::JointConstraint>& constraints,
    const RobotStatePtr pStartState,
    const RobotStatePtr pEndState,
    int steps)
{
    size_t joints = constraints.size();
    vector<RobotStatePtr> trajectory(steps);
    vector<double> powers(QUINTIC_COEFFICIENTS);

    // Calculate time powers
    powers[0] = 1.0;
    powers[1] = double(steps);

    for (size_t index = 2; index < QUINTIC_COEFFICIENTS; index++)
        powers[index] = powers[index - 1] * powers[1];

    // Calculate quintic spline coefficients for each joint
    vector<QuinticSplineCoefficients> coefficients(joints);

    for (size_t jointIndex = 0; jointIndex < joints; jointIndex++)
    {
        const string& jointName = constraints[jointIndex].joint_name;
        double startCoefficient = *pStartState->getJointPositions(jointName);
        double endCoefficient = *pEndState->getJointPositions(jointName);

        coefficients[jointIndex].a = startCoefficient;
        coefficients[jointIndex].b = 0;
        coefficients[jointIndex].c = 0;
        coefficients[jointIndex].d = (-20 * startCoefficient + 20 * endCoefficient) / (2 * powers[3]);
        coefficients[jointIndex].e = (30 * startCoefficient - 30 * endCoefficient) / (2 * powers[4]);
        coefficients[jointIndex].g = (-12 * startCoefficient + 12 * endCoefficient) / (2 * powers[5]);
    }

    // Fill in joint positions at each time step
    for (size_t step = 0; step < steps; step++)
    {
        RobotStatePtr pState(new RobotState(pStartState->getRobotModel()));

        for (size_t jointIndex = 0; jointIndex < joints; jointIndex++)
        {
            const string& jointName = constraints[jointIndex].joint_name;
            double jointState = 0.0;

            for (size_t coeffIndex = 0;
                coeffIndex < QUINTIC_COEFFICIENTS;
                coeffIndex++)
            {
                jointState
                    += powers[coeffIndex]
                    * coefficients[jointIndex].all[coeffIndex];
            }

            pState->setJointPositions(jointName, &jointState);
        }

        trajectory[step] = pState;
    }

    return trajectory;
}

vector<RobotStatePtr> PluginContext::interpolateLinear(
    const vector<moveit_msgs::JointConstraint>& constraints,
    const RobotStatePtr pStartState,
    const RobotStatePtr pEndState,
    int steps)
{
    vector<RobotStatePtr> trajectory(steps);

    for (size_t step = 0; step < steps; step++)
    {
        RobotStatePtr pState(new RobotState(pStartState->getRobotModel()));
        pStartState->interpolate(*pEndState, double(step) / double(steps), *pState);
        trajectory[step] = pState;
    }

    return trajectory;
}

bool PluginContext::terminate()
{
    return true;
}

void PluginContext::clear()
{
}
