
/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 reconfigure.h
 ROS Dynamic Reconfigure Interface
 Copyright (C) 2025 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/Reconfigure.h>

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const char DESCRIPTIONS[] = "parameter_descriptions";
const char UPDATES[] = "parameter_updates";
const char SET[] = "set_parameters";

/*----------------------------------------------------------*\
| Definitions
\*----------------------------------------------------------*/

typedef dynamic_reconfigure::Reconfigure::Request ReconfigureReq;
typedef dynamic_reconfigure::Reconfigure::Response ReconfigureRes;
typedef ros::ServiceServer<ReconfigureReq, ReconfigureRes> ReconfigureSrv;

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

ros::Publisher descriptionPublisher;
ros::Publisher updatePublisher;
ReconfigureSrv reconfigureServer;

/*----------------------------------------------------------*\
| Forward Declarations
\*----------------------------------------------------------*/

extern ros::NodeHandle node;

/*----------------------------------------------------------*\
| Functions
\*----------------------------------------------------------*/

void initializeDynamicReconfigure()
{
  descriptionPublisher = node.advertise(DESCRIPTIONS, 1, true);
  updatePublisher = node.advertise(UPDATES, 1, true);
  reconfigureServer = node.advertiseService(SET, &configureCommand);
}

void descriptionFeedback()
{
  dynamic_reconfigure::ConfigDescription descriptionMsg;
}

void updateFeedback()
{
  dynamic_reconfigure::Config updateMsg;
}

bool configureCommand(ReconfigureReq &req, ReconfigureRes& res)
{

}
