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
    // Loaded controllers
    controllerMap m_controllers;

    // GPIO connection
    int m_gpio;

    // Whether GPIO is enabled
    bool m_enableGpio;

public:
    robot();
    ~robot();

public:
    // Load controller settings
    robot& deserialize(ros::NodeHandle node);

    // Initialize controllers
    bool init();

    // Update controllers
    void update();

    // Print logo
    robot& logo();

    // Get GPIO daemon handle
    int getGpio();

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
