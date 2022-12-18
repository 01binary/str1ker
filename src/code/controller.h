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
    // Robot the controller belongs to
    robot& m_robot;

    // Controller display name
    std::string m_name;

    // Controller control path
    std::string m_path;

    // Whether controller is enabled
    bool m_enable;

public:
    controller(robot& robot, const char* path);

public:
    // Get robot
    robot& getRobot();

    // Get controller display name
    const char* getName();

    // Get controller control path
    const char* getPath();

    // Get enabled status
    const bool isEnabled();

    // Get controller type display name
    virtual const char* getType() = 0;

    // Load controller settings
    virtual void deserialize(ros::NodeHandle node);

    // Initialize controller
    virtual bool init();

    // Publish self or children
    virtual void publish();

protected:
    // Get child controller path
    std::string getControllerPath(const char* controllerName);
};

} // namespace str1ker
