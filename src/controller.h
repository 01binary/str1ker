/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 arm.h

 Controller Base Class
 Created 1/19/2021

 Copyright (C) 2021 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros/ros.h>
#include <string>

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| Declarations
\*----------------------------------------------------------*/

class robot;

/*----------------------------------------------------------*\
| controller base class
\*----------------------------------------------------------*/

class controller
{
protected:
    // Current node
    ros::NodeHandle m_node;

    // Controller display name
    std::string m_name;

    // Controller control path
    std::string m_path;

    // Whether controller is enabled
    bool m_enable;

public:
    controller(ros::NodeHandle node, const char* path);

public:
    // Get current node
    ros::NodeHandle getNode();

    // Get controller name
    const char* getName();

    // Get controller path
    const char* getPath();

    // Get controller parent name
    std::string getParentName();

    // Get controller parent path
    std::string getParentPath();

    // Get enabled status
    const bool isEnabled();

    // Get controller type display name
    virtual const char* getType() = 0;

    // Load controller settings
    virtual bool configure();

    // Initialize controller
    virtual bool init();

    // Update self and/or children
    virtual void update(ros::Time time, ros::Duration period);

protected:
    // Get child controller path
    std::string getControllerPath(const char* controllerName);
};

} // namespace str1ker
