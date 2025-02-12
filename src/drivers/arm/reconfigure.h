
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
| Forward Declarations
\*----------------------------------------------------------*/

extern ros::NodeHandle node;
void configureCommand(const ReconfigureReq &req, ReconfigureRes& res);

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

dynamic_reconfigure::ConfigDescription descriptionMsg;
ros::Publisher descriptionPublisher(DESCRIPTIONS, &descriptionMsg);

dynamic_reconfigure::Config updateMsg;
ros::Publisher updatePublisher(UPDATES, &updateMsg);

ReconfigureSrv reconfigureServer(SET, &configureCommand);

/*----------------------------------------------------------*\
| Functions
\*----------------------------------------------------------*/

void initializeDynamicReconfigure()
{
  node.advertise(descriptionPublisher);
  node.advertise(updatePublisher);
  node.advertiseService(reconfigureServer);
}

void descriptionFeedback()
{
}

void updateFeedback()
{
}

void configureCommand(const ReconfigureReq &req, ReconfigureRes& res)
{

}
