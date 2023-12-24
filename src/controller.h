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

    // Controller type
    std::string m_type;

    // Controller configuration node name
    std::string m_name;

    // Controller configuration path
    std::string m_path;

    // Whether controller is enabled
    bool m_enable;

public:
    controller(ros::NodeHandle node, std::string type, std::string path);

public:
    // Get current node
    ros::NodeHandle getNode();

    // Get controller type name
    const std::string& getType();

    // Get controller name
    const std::string& getName();

    // Get controller path
    const std::string& getPath();

    // Get controller parent name
    std::string getParentName();

    // Get controller parent path
    std::string getParentPath();

    // Get enabled status
    const bool isEnabled();

    // Load controller settings
    virtual bool configure();

    // Initialize controller
    virtual bool init();

    // Update self and/or children
    virtual void update(ros::Time time, ros::Duration period);

protected:
    // Get child controller path
    std::string getChildPath(const std::string& controllerName);
};

} // namespace str1ker
