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
    static const size_t QUINTIC_COEFFICIENTS;
    static const char* PLUGIN_NAME;

public:
    PluginContext(const std::string& group);
    ~PluginContext() override;

public:
    bool solve(planning_interface::MotionPlanResponse& res) override;
    bool solve(planning_interface::MotionPlanDetailedResponse& res) override;

    robot_state::RobotStatePtr getStartState() const;
    robot_state::RobotStatePtr getGoalState() const;

    std::vector<robot_state::RobotStatePtr> interpolateQuintic(
        const std::vector<moveit_msgs::JointConstraint>& jointConstraints,
        const robot_state::RobotStatePtr pStartState,
        const robot_state::RobotStatePtr pEndState,
        double discretization,
        int steps);

    bool terminate() override;
    void clear() override;
};

}  // namespace str1ker
