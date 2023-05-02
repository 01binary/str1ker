/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 planner.h

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

}  // namespace str1ker
