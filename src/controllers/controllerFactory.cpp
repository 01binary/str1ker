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
#include "../robot.h"
#include "controllerFactory.h"
#include "controllerUtilities.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace str1ker;
using namespace std;

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

map<string, controllerRegistration> controllerFactory::s_types;
bool controllerFactory::s_initialized = false;
controllerFactory instance;

/*----------------------------------------------------------*\
| controllerFactory implementation
\*----------------------------------------------------------*/

controllerFactory::controllerFactory()
{
    s_initialized = true;
}

controller* controllerFactory::fromPath(ros::NodeHandle node, string path)
{
    try
    {
        if (!s_initialized)
        {
            ROS_ERROR("attempted to deserialize %s before static initialization: ensure controllerFactory.cpp is first in CMakeLists", path.c_str());
            return NULL;
        }

        string controllerType;

        if (!ros::param::get(path + "/controller", controllerType))
            return NULL;

        if (s_types.find(controllerType) == s_types.end())
        {
            ROS_WARN("could not find type '%s' loading %s", controllerType.c_str(), path.c_str());
            return NULL;
        }

        struct controllerRegistration& reg = s_types[controllerType];
        controller* instance = NULL;

        if (reg.shared)
        {
            if (NULL == reg.instance)
            {
                reg.instance = reg.create(node, path);
                reg.instance->configure();
            }

            instance = reg.instance;
        }
        else
        {
            instance = reg.create(node, path);
            instance->configure();
        }

        return instance;
    }
    catch (...)
    {
        ROS_ERROR("exception deserializing %s", path.c_str());
        return NULL;
    }
}

controller* controllerFactory::fromType(string type)
{
    try
    {
        if (!s_initialized)
        {
            ROS_ERROR("attempted to deserialize %s before static initialization: ensure controllerFactory.cpp is first in CMakeLists", type.c_str());
            return NULL;
        }

        if (s_types.find(type) == s_types.end())
        {
            ROS_ERROR("failed to deserialize type '%s'", type.c_str());
            return NULL;
        }

        controller* instance = s_types[type].instance;

        if (NULL == instance)
        {
            ROS_ERROR("type '%s' requested before it was loaded", type.c_str());
        }

        return instance;
    }
    catch (...)
    {
        ROS_ERROR("exception deserializing %s", type.c_str());
        return NULL;
    }
}

controllerArray controllerFactory::fromNamespace(ros::NodeHandle node, string controllerNamespace)
{
    vector<string> params;
    ros::param::getParamNames(params);

    controllerArray controllers;
    set<string> controllerPaths;

    string parentPath = controllerNamespace[0] == '/'
        ? controllerNamespace.c_str() + 1
        : controllerNamespace.c_str();

    for (vector<string>::iterator pos = params.begin();
        pos != params.end();
        pos++)
    {
        auto path = controllerUtilities::getControllerPath(*pos, parentPath);

        if (path.length() && controllerPaths.find(path) == controllerPaths.end())
        {
            controller* instance = fromPath(node, path);

            if (instance)
            {
                controllers.push_back(shared_ptr<controller>(instance));
                controllerPaths.insert(path);
            }
        }
    }

    return controllers;
}

void controllerFactory::registerType(string type, createController create, bool shared)
{
    try
    {
        if (!s_initialized)
        {
            ROS_ERROR("attempted to register %s before static initialization: ensure controllerFactory.cpp is first in CMakeLists", type.c_str());
            return;
        }

        struct controllerRegistration reg;

        reg.create = create;
        reg.shared = shared;
        reg.instance = NULL;

        s_types[type] = reg;

        ROS_INFO("registered %s %s", type.c_str(), shared ? "singleton" : "scoped");
    }
    catch (...)
    {
        ROS_ERROR("exception registering %s", type.c_str());
    }
}

