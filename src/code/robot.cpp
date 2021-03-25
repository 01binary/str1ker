/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 robot.cpp

 Robot Controller Implementation
 Created 11/28/2020

 Copyright (C) 2021 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <vector>
#include <set>
#include <pigpio.h>
#include <ros/ros.h>
#include "robot.h"
#include "controllerFactory.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace str1ker;
using namespace std;

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const char robot::PATH[] = "/robot";

/*----------------------------------------------------------*\
| robot implementation
\*----------------------------------------------------------*/

robot::robot()
{
}

robot::~robot()
{
    for(controllerMap::iterator pos = m_controllers.begin();
        pos != m_controllers.end();
        ++pos)
    {
        delete pos->second;
    }
}

controller* robot::getController(const char* name)
{
    return m_controllers[name];
}

bool robot::init()
{
    ROS_INFO("initializing controllers...");

    if (gpioInitialise() < 0)
    {
        ROS_ERROR("failed to initialize GPIO");
        return false;
    }

    for(controllerMap::iterator pos = m_controllers.begin();
        pos != m_controllers.end();
        ++pos)
    {
        if (!pos->second->init()) return false;
    }

    ROS_INFO("initialization completed");

    return true;
}

void robot::publish()
{
    for(controllerMap::iterator pos = m_controllers.begin();
        pos != m_controllers.end();
        ++pos)
    {
        if (pos->second) pos->second->publish();
    }
}

robot& robot::deserialize(ros::NodeHandle node)
{
    ROS_INFO("loading controllers...");

    char path[255] = {0};
    vector<string> params;
    ros::param::getParamNames(params);

    for (vector<string>::iterator pos = params.begin();
        pos != params.end();
        pos++)
    {
        if (getControllerPath(pos->c_str(), PATH + 1, path) &&
            m_controllers.find(path) == m_controllers.end())
        {
            const char* name = getControllerName(path);
            controller* instance = controllerFactory::deserialize(PATH, name, node);

            if (instance) m_controllers[path] = instance;
        }
    }

    ROS_INFO("loaded successfully");

    return *this;
}

const char* robot::getControllerName(const char* path)
{
    const char* lastSep = strrchr(path, '/');

    if (lastSep == NULL) return path;

    return lastSep + 1;
}

const char* robot::getControllerPath(const char* path, const char* componentType, char* componentPath)
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

robot& robot::logo()
{
    puts("                                                                                     ███████                  ");
    puts(" ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ ");
    puts("█              █ █           █            █    █ █  █              █        ███      ███████    █            █");
    puts(" ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ ");
    puts("             █ █     █       █            █      █    █            ████      █                  █            █");
    puts(" ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █");
    puts("                                                                                     ███████                  ");

    return *this;
}