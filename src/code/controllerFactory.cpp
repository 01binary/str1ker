/*
                                                                                     @@@@@@@                  
 @@@@@@@@@@@@  @@@@@@@@@@@@   @@@@@@@@@@@@       @  @@@@@@@@@@@@@  @           @  @@@       @@@  @@@@@@@@@@@@ 
@              @ @           @            @    @ @  @              @        @@@      @@@@@@@    @            @
 @@@@@@@@@@@@  @   @         @@@@@@@@@@@@@   @   @   @             @   @@@@@      @@@       @@@ @@@@@@@@@@@@@ 
             @ @     @       @            @      @    @@           @@@@      @                  @            @
 @@@@@@@@@@@@  @       @     @            @      @      @@@@@@@@@  @          @   @@@       @@@ @            @
                                                                                     @@@@@@@                  
 controllerFactory.cpp

 Controller factory implementation
 Created 1/21/2021

 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros/ros.h>
#include "controllerFactory.h"
#include "solenoid.h"
#include "pwmServo.h"
#include "dynamixelPro.h"
#include "linear.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace str1ker;
using namespace std;

/*----------------------------------------------------------*\
| controllerFactory implementation
\*----------------------------------------------------------*/

map<string, controllerRegistration> controllerFactory::s_types;

controller* controllerFactory::deserialize(const char* parentPath, const char* controllerName, ros::NodeHandle node)
{
    string componentPath = string(parentPath) + "/" + controllerName;
    string controllerTypePath = componentPath + "/controller";
    string controllerType;

    ros::param::get(controllerTypePath, controllerType);

    if (s_types.find(controllerType.c_str()) == s_types.end())
    {
        ROS_WARN("could not find type '%s'", controllerType.c_str());
        return NULL;
    }

    struct controllerRegistration& reg = s_types[controllerType];
    controller* instance = NULL;

    if (reg.shared)
    {
        if (NULL == reg.instance)
        {
            reg.instance = reg.create(componentPath.c_str());
            reg.instance->deserialize(node);
        }

        instance = reg.instance;
    }
    else
    {
        instance = reg.create(componentPath.c_str());
        instance->deserialize(node);
    }

    return instance;
}

controller* controllerFactory::deserialize(const char* type)
{
    if (s_types.find(type) == s_types.end())
    {
        ROS_ERROR("failed to deserialize type '%s'", type);
        return NULL;
    }

    controller* instance = s_types[type].instance;

    if (NULL == instance)
    {
        ROS_ERROR("type '%s' requested before it was loaded", type);
    }

    return instance;
}

void controllerFactory::registerType(const char* type, createController create, bool shared)
{
    struct controllerRegistration reg;

    reg.create = create;
    reg.shared = shared;
    reg.instance = NULL;

    s_types[type] = reg;

    ROS_INFO("registered %s %s", type, shared ? "singleton" : "scoped");
}
