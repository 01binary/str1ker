/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 robot.h

 Robot Controller
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
#include <map>
#include "arm.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| Definitions
\*----------------------------------------------------------*/

typedef std::map<std::string, std::shared_ptr<controller>> controllerMap;

/*----------------------------------------------------------*\
| robot class
\*----------------------------------------------------------*/

class robot
{
private:
    static const char PATH[];

private:
    // Current node
    ros::NodeHandle m_node;

    // Loaded controllers
    controllerMap m_controllers;

    // Spin rate
    double m_rate;

public:
    robot(ros::NodeHandle node);
    ~robot();

public:
    // Load controller settings
    robot& deserialize();

    // Initialize controllers
    bool init();

    // Update controllers
    void update();

    // Run ROS loop
    robot& run();

    // Print logo
    robot& logo();

    // Get current node
    ros::NodeHandle getNode();

    // Get controller of type by name
    template <class C> C* getController(const char* name)
    {
        return dynamic_cast<C*>(getController(name).get());
    }

    // Get any controller by name
    std::shared_ptr<controller> getController(const char* name);

public:
    // Get component name
    static const char* getControllerName(const char* path);

    // Get parent name
    static const char* getControllerPath(const char* path, const char* componentType, char* componentPath);
};

} // namespace str1ker
