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
        auto path = getControllerPath(*pos, parentPath);

        if (path.size() && controllerPaths.find(path) == controllerPaths.end())
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

string controllerFactory::getControllerName(string path)
{
    const char* lastSep = strrchr(path.c_str(), '/');
    if (lastSep == NULL) return path;

    return lastSep + 1;
}

string controllerFactory::getParentName(string path)
{
    const char* parent = path.c_str() + path.length() - 1;
    if (*parent == '/') parent--;

    while (*parent != '/' && parent >= path)
        parent--;

    const char* parentEnd = parent--;

    while (*parent != '/' && parent >= path)
        parent--;

    if (parent == path) return string();

    int parentLength = parentEnd - parent - 1;

    string parentName(parentLength, 0);
    strncpy(&parentName[0], parent + 1, parentLength);

    return parentName;
}

string controllerFactory::getParentPath(string path)
{
    const char* parent = path.c_str() + path.length() - 1;
    if (*parent == '/') parent--;

    while (*parent != '/' && parent >= path)
        parent--;

    if (parent == path) return string();

    int length = parent - path.c_str();

    string parentPath(length + 1, 0);
    strncpy(&parentPath[0], path.c_str(), length);

    return parentPath;
}

string controllerFactory::getControllerPath(string path, string parentPath)
{
    string typePath = "/" + parentPath + "/";

    const char* typeNode = strstr(path.c_str(), typePath.c_str());
    if (typeNode == NULL) return string();

    // Path must end with /controller
    const char* leaf = path.c_str() + path.length() - 1;

    while (*leaf != '/' && leaf >= path)
        leaf--;

    if (strncmp(leaf, "/controller", strlen("/controller")) == 0)
    {
        int length = leaf - path.c_str();
        string controllerPath(length, '\0');

        strncpy(&controllerPath[0], path.c_str(), length);

        return controllerPath;
    }

    return string();
}
