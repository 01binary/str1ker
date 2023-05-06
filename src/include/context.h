/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 context.h

 Motion Planning Context
 Created 05/01/2023

 Copyright (C) 2023 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <string>
#include <moveit/planning_interface/planning_interface.h>

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| PluginContext class
\*----------------------------------------------------------*/

class PluginContext: public planning_interface::PlanningContext
{
private:
    static const int STEPS;
    static const double STEP_DURATION;
    static const size_t QUINTIC_COEFFICIENTS;
    static const char* PLUGIN_NAME;

private:
    bool m_useQuinticInterpolation;

public:
    PluginContext(const std::string& group);
    ~PluginContext() override;

public:
    bool solve(planning_interface::MotionPlanResponse& res) override;
    bool solve(planning_interface::MotionPlanDetailedResponse& res) override;
    bool terminate() override;
    void clear() override;

private:
    robot_state::RobotStatePtr getStartState() const;
    robot_state::RobotStatePtr getGoalState() const;

private:
    static std::vector<double> calculateQuinticPowers(double step);

    static std::vector<robot_state::RobotStatePtr> interpolateQuintic(
        const std::vector<moveit_msgs::JointConstraint>& jointConstraints,
        const robot_state::RobotStatePtr pStartState,
        const robot_state::RobotStatePtr pEndState,
        int steps);

    static std::vector<robot_state::RobotStatePtr> interpolateLinear(
        const std::vector<moveit_msgs::JointConstraint>& jointConstraints,
        const robot_state::RobotStatePtr pStartState,
        const robot_state::RobotStatePtr pEndState,
        int steps);
};

}  // namespace str1ker
