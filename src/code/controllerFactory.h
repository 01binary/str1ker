/*
                                                                                     @@@@@@@                  
 @@@@@@@@@@@@  @@@@@@@@@@@@   @@@@@@@@@@@@       @  @@@@@@@@@@@@@  @           @  @@@       @@@  @@@@@@@@@@@@ 
@              @ @           @            @    @ @  @              @        @@@      @@@@@@@    @            @
 @@@@@@@@@@@@  @   @         @@@@@@@@@@@@@   @   @   @             @   @@@@@      @@@       @@@ @@@@@@@@@@@@@ 
             @ @     @       @            @      @    @@           @@@@      @                  @            @
 @@@@@@@@@@@@  @       @     @            @      @      @@@@@@@@@  @          @   @@@       @@@ @            @
                                                                                     @@@@@@@                  
 controllerFactory.h

 Controller Factory class
 Created 1/21/2021

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

typedef controller* (*createController)(const char*);

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
    template<class T> static T* deserialize(
        const char* parentPath,
        const char* controllerName,
        ros::NodeHandle node)
    {
        return dynamic_cast<T*>(deserialize(parentPath, controllerName, node));
    }

    // Deserialize controller by type with type cast
    template<class T> static T* deserialize(const char* type)
    {
        return dynamic_cast<T*>(deserialize(type));
    }

    // Deserialize controller by path
    static controller* deserialize(const char* parentPath, const char* controllerName, ros::NodeHandle node);

    // Deserialize controller by type
    static controller* deserialize(const char* type);

    // Register controller type
    static void registerType(const char* type, createController create, bool shared);
};

} // namespace str1ker
