/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 controllerFactory.h

 Controller Factory Class
 Created 1/21/2021

 Copyright (C) 2021 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <string>
#include <map>
#include "controller.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| Macros
\*----------------------------------------------------------*/

#define REGISTER_CONTROLLER(a) \
    class register_##a { public: register_##a() { \
    try { controllerFactory::registerType(a::TYPE, a::create, false); } catch (...) {} \
    }}; register_##a reg##a;

#define REGISTER_SINGLETON(a) \
    class register_##a { public: register_##a() { controllerFactory::registerType(a::TYPE, a::create, true);}}; \
    register_##a reg##a;

/*----------------------------------------------------------*\
| Definitions
\*----------------------------------------------------------*/

typedef controller* (*createController)(ros::NodeHandle, std::string);
typedef std::vector<std::shared_ptr<controller>> controllerArray;

/*----------------------------------------------------------*\
| controllerRegistration struct
\*----------------------------------------------------------*/

struct controllerRegistration
{
    // Create handler
    createController create;

    // Whether this controller is shared between all others
    bool shared;

    // Controller singleton if shared
    controller* instance;
};

/*----------------------------------------------------------*\
| controllerFactory class
\*----------------------------------------------------------*/

class controllerFactory
{
private:
    static bool s_initialized;
    static std::map<std::string, controllerRegistration> s_types;

public:
    controllerFactory();

public:
    // Deserialize controller with type cast
    template<class T> static T* deserialize(ros::NodeHandle node, std::string path)
    {
        return dynamic_cast<T*>(fromPath(node, path));
    }

    // Deserialize controller by type with type cast
    template<class T> static T* deserialize(std::string type)
    {
        return dynamic_cast<T*>(fromType(type));
    }

    // Deserialize controller by path
    static controller* fromPath(ros::NodeHandle node, std::string path);

    // Deserialize controller by type
    static controller* fromType(std::string type);

    // Deserialize all controllers in namespace
    static controllerArray fromNamespace(ros::NodeHandle node, std::string controllerNamespace);

    // Register controller type
    static void registerType(std::string type, createController create, bool shared);

    
};

} // namespace str1ker
