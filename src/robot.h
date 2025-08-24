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
#include "arm.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| robot class
\*----------------------------------------------------------*/

class robot
{
private:
    // Current node
    ros::NodeHandle m_node;

    // Spin rate
    float m_rate;

public:
    robot(ros::NodeHandle node);

public:
    // Get current node
    ros::NodeHandle getNode();

    // Load controller settings
    robot& configure(const char* nameSpace);

    // Initialize controllers
    bool init();

    // Update controllers
    bool update();

    // Run ROS loop
    robot& run();

    // Print logo
    robot& logo();
};

} // namespace str1ker
