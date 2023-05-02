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
#include <moveit/planning_scene/planning_scene.h>
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
    static const char* PLUGIN_NAME;

public:
    PluginContext(const std::string& group);
    ~PluginContext() override;

public:
    bool solve(planning_interface::MotionPlanResponse& res) override;
    bool solve(planning_interface::MotionPlanDetailedResponse& res) override;
    bool terminate() override;
    void clear() override;

private:
    bool validateRequest();
    Eigen::Isometry3d extractGoalPose()
};

}  // namespace str1ker
