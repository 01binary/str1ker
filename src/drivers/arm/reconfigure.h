
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

extern const String NAMESPACE;

const String DESCRIPTIONS = NAMESPACE + "parameter_descriptions";
const String UPDATES = NAMESPACE + "parameter_updates";
const String SET = NAMESPACE + "set_parameters";

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
ros::Publisher descriptionPublisher(DESCRIPTIONS.c_str(), &descriptionMsg);

dynamic_reconfigure::Config updateMsg;
ros::Publisher updatePublisher(UPDATES.c_str(), &updateMsg);

ReconfigureSrv reconfigureServer(SET.c_str(), &configureCommand);

/*----------------------------------------------------------*\
| Functions
\*----------------------------------------------------------*/

void initializeDynamicReconfigure()
{
  node.advertise(descriptionPublisher);
  node.advertise(updatePublisher);
  node.advertiseService(reconfigureServer);
  node.negotiateTopics();

  delay(1000);

  descriptionFeedback();
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
