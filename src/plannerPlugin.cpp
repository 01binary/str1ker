/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 plannerPlugin.cpp

 Motion Planning Plugin
 Created 05/01/2023

 Copyright (C) 2023 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include "include/planner.h"
#include "include/context.h"
#include <moveit/planning_interface/planning_interface.h>
#include <pluginlib/class_list_macros.h>

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace std;
using namespace moveit::core;
using namespace planning_scene;
using namespace planning_interface;
using namespace robot_trajectory;
using namespace trajectory_msgs;
using namespace robot_state;
using namespace str1ker;

/*----------------------------------------------------------*\
| PlannerPlugin implementation
\*----------------------------------------------------------*/

PlannerPlugin::PlannerPlugin() : PlannerManager()
{
}

PlannerPlugin::~PlannerPlugin()
{
}

bool PlannerPlugin::initialize(const RobotModelConstPtr& model, const string& ns)
{
    return true;
}

bool PlannerPlugin::canServiceRequest(const MotionPlanRequest& req) const
{
    return true;
}

string PlannerPlugin::getDescription() const
{
    return "Str1ker Planner";
}

void PlannerPlugin::getPlanningAlgorithms(vector<string>& algs) const
{
    algs.clear();
    algs.push_back("Quintic Splines");
}

PlanningContextPtr PlannerPlugin::getPlanningContext(
    const PlanningSceneConstPtr& planning_scene,
    const MotionPlanRequest& req,
    moveit_msgs::MoveItErrorCodes& error_code) const
{
    PlanningContextPtr pContext(new PluginContext(req.group_name));
    pContext->setPlanningScene(planning_scene);
    pContext->setMotionPlanRequest(req);
    return pContext;
}

PLUGINLIB_EXPORT_CLASS(str1ker::PlannerPlugin, planning_interface::PlannerManager);
