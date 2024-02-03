/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 motionPlanningPlugin.h

 Motion Planning Plugin
 Created 05/01/2023

 Copyright (C) 2023 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <string>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| PlannerPlugin class
\*----------------------------------------------------------*/

class PlannerPlugin: public planning_interface::PlannerManager
{
public:
    PlannerPlugin();
    virtual ~PlannerPlugin();

public:
    bool initialize(
        const robot_model::RobotModelConstPtr& model,
        const std::string& ns) override;
    bool canServiceRequest(const moveit_msgs::MotionPlanRequest& req) const override;
    void getPlanningAlgorithms(std::vector<std::string>& algs) const override;
    std::string getDescription() const override;
    planning_interface::PlanningContextPtr getPlanningContext(
        const planning_scene::PlanningSceneConstPtr& planning_scene,
        const planning_interface::MotionPlanRequest& req,
        moveit_msgs::MoveItErrorCodes& error_code) const;
};

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
