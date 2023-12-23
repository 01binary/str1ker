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

controller* controllerFactory::deserialize(ros::NodeHandle node, const char* parentPath, const char* controllerName)
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

        if (!ros::param::get(controllerTypePath, controllerType))
            return NULL;

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
                reg.instance = reg.create(node, componentPath.c_str());
                reg.instance->configure();
            }

            instance = reg.instance;
        }
        else
        {
            instance = reg.create(node, componentPath.c_str());
            instance->configure();
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

controllerArray controllerFactory::deserialize(ros::NodeHandle node, const char* controllerNamespace)
{
    char path[255] = {0};
    vector<string> params;
    ros::param::getParamNames(params);

    controllerArray controllers;
    std::set<string> controllerPaths;

    for (vector<string>::iterator pos = params.begin();
        pos != params.end();
        pos++)
    {
        if (getControllerPath(pos->c_str(), controllerNamespace + 1, path) &&
            controllerPaths.find(path) == controllerPaths.end())
        {
            const char* name = getControllerName(path);
            controller* instance = deserialize(node, controllerNamespace, name);

            if (instance)
            {
                controllers.push_back(shared_ptr<controller>(instance));
                controllerPaths.insert(string(path));
            }
        }
    }

    return controllers;
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

const char* controllerFactory::getControllerName(const char* path)
{
    const char* lastSep = strrchr(path, '/');
    if (lastSep == NULL) return path;

    return lastSep + 1;
}

const char* controllerFactory::getControllerPath(
    const char* path, const char* componentType, char* componentPath)
{
    char componentTypePath[64] = {0};
    sprintf(componentTypePath, "/%s/", componentType);

    const char* typeNode = strstr(path, componentTypePath);
    if (typeNode == NULL) return NULL;

    const char* nameNode = typeNode + strlen(componentTypePath);
    const char* nextNode = strchr(nameNode , '/');

    if (nextNode == NULL) nextNode = nameNode + strlen(nameNode);

    strncpy(componentPath, path, nextNode - path);
    return componentPath;
}