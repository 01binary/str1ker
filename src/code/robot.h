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

#include <string>
#include <map>
#include "arm.h"
#include "adc.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| Definitions
\*----------------------------------------------------------*/

typedef std::map<std::string, controller*> controllerMap;

/*----------------------------------------------------------*\
| robot class
\*----------------------------------------------------------*/

class robot
{
private:
    static const char PATH[];

private:
    controllerMap m_controllers;

public:
    robot();
    ~robot();

public:
    // Load controller settings
    robot& deserialize(ros::NodeHandle node);

    // Initialize controllers
    bool init();

    // Publish controller topics
    void publish();

    // Print logo
    robot& logo();

    // Get controller of type by name
    template <class C> C* getController(const char* name)
    {
        return dynamic_cast<C*>(getController(name));
    }

    // Get any controller by name
    controller* getController(const char* name);

public:
    // Get component name
    static const char* getControllerName(const char* path);

    // Get parent name
    static const char* getControllerPath(const char* path, const char* componentType, char* componentPath);
};

} // namespace str1ker
