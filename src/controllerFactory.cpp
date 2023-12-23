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
    vector<string> params;
    ros::param::getParamNames(params);

    controllerArray controllers;
    std::set<string> controllerPaths;

    auto parentPath = controllerNamespace[0] == '/'
        ? controllerNamespace + 1
        : controllerNamespace;

    for (vector<string>::iterator pos = params.begin();
        pos != params.end();
        pos++)
    {
        auto path = getControllerPath(pos->c_str(), parentPath);

        if (path.size() && controllerPaths.find(path) == controllerPaths.end())
        {
            auto parent = getParentPath(path.c_str());
            auto name = getControllerName(path.c_str());

            controller* instance = deserialize(node, parent.c_str(), name);

            if (instance)
            {
                controllers.push_back(shared_ptr<controller>(instance));
                controllerPaths.insert(path);
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

string controllerFactory::getParentName(const char* path)
{
    const char* parent = path + strlen(path) - 1;
    if (*parent == '/') parent--;

    while (*parent != '/' && parent >= path)
        parent--;

    const char* parentEnd = parent;

    while (*parent != '/' && parent >= path)
        parent--;

    if (parent == path) return string();

    int parentLength = parentEnd - parent;

    string parentName(parentLength + 1, 0);
    strncpy(&parentName[0], parent, parentLength);

    return parentName;
}

string controllerFactory::getParentPath(const char* path)
{
    const char* parent = path + strlen(path) - 1;
    if (*parent == '/') parent--;

    while (*parent != '/' && parent >= path)
        parent--;

    if (parent == path) return string();

    int length = parent - path;

    string parentPath(length + 1, 0);
    strncpy(&parentPath[0], path, length);

    return parentPath;
}

string controllerFactory::getControllerPath(const char* path, const char* parentPath)
{
    char componentTypePath[64] = {0};
    sprintf(componentTypePath, "/%s/", parentPath);

    const char* typeNode = strstr(path, componentTypePath);
    if (typeNode == NULL) return string();

    // Path must end with /controller
    const char* leaf = path + strlen(path) - 1;

    while (*leaf != '/' && leaf >= path)
        leaf--;

    if (strncmp(leaf, "/controller", strlen("/controller")) == 0)
    {
        int length = leaf - path;
        string controllerPath(length, '\0');

        strncpy(&controllerPath[0], path, length);

        return controllerPath;
    }

    return string();
}