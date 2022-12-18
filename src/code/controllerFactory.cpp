/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 controllerFactory.cpp

 Controller Factory Implementation
 Created 1/21/2021

 Copyright (C) 2021 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros/ros.h>
#include "robot.h"
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
bool controllerFactory::s_initialized = false;

controllerFactory instance;

controllerFactory::controllerFactory()
{
    s_initialized = true;
}

controller* controllerFactory::deserialize(robot& robot, const char* parentPath, const char* controllerName, ros::NodeHandle node)
{
    try
    {
        if (!s_initialized)
        {
            ROS_ERROR("attempted to deserialize %s/%s before static initialization: ensure controllerFactory.cpp is first in CMakeLists", parentPath, controllerName);
            return NULL;
        }

        string componentPath = string(parentPath) + "/" + controllerName;
        string controllerTypePath = componentPath + "/controller";
        string controllerType;

        ros::param::get(controllerTypePath, controllerType);

        if (s_types.find(controllerType.c_str()) == s_types.end())
        {
            ROS_WARN("could not find type '%s' loading %s from parent %s (no %s defined)",
                controllerType.c_str(), controllerName, parentPath, controllerTypePath.c_str());

            return NULL;
        }

        struct controllerRegistration& reg = s_types[controllerType];
        controller* instance = NULL;

        if (reg.shared)
        {
            if (NULL == reg.instance)
            {
                reg.instance = reg.create(robot, componentPath.c_str());
                reg.instance->deserialize(node);
            }

            instance = reg.instance;
        }
        else
        {
            instance = reg.create(robot, componentPath.c_str());
            instance->deserialize(node);
        }

        return instance;
    }
    catch (...)
    {
        ROS_ERROR("exception deserializing %s/%s", parentPath, controllerName);
        return NULL;
    }
}

controller* controllerFactory::deserialize(const char* type)
{
    try
    {
        if (!s_initialized)
        {
            ROS_ERROR("attempted to deserialize %s before static initialization: ensure controllerFactory.cpp is first in CMakeLists", type);
            return NULL;
        }

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
    catch (...)
    {
        ROS_ERROR("exception deserializing %s", type);
        return NULL;
    }
}

void controllerFactory::registerType(const char* type, createController create, bool shared)
{
    try
    {
        if (!s_initialized)
        {
            ROS_ERROR("attempted to register %s before static initialization: ensure controllerFactory.cpp is first in CMakeLists", type);
            return;
        }

        struct controllerRegistration reg;

        reg.create = create;
        reg.shared = shared;
        reg.instance = NULL;

        s_types[type] = reg;

        ROS_INFO("registered %s %s", type, shared ? "singleton" : "scoped");
    }
    catch (...)
    {
        ROS_ERROR("exception registering %s", type);
    }
}
