/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 controller.cpp

 Controller Base Class Implementation
 Created 1/19/2021

 Copyright (C) 2021 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <algorithm>
#include <ros/ros.h>
#include "controllerFactory.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace str1ker;
using namespace std;

/*----------------------------------------------------------*\
| controller implementation
\*----------------------------------------------------------*/

controller::controller(ros::NodeHandle node, const char* path)
    : m_node(node)
    , m_name(controllerFactory::getControllerName(path))
    , m_path(path)
    , m_enable(true)
{
}

ros::NodeHandle controller::getNode()
{
    return m_node;
}

const char* controller::getName()
{
    return m_name.c_str();
}

const char* controller::getPath()
{
    return m_path.c_str();
}

const bool controller::isEnabled()
{
    return m_enable;
}

bool controller::configure()
{
    string indent;
    indent.resize(
        (max((int)count(m_path.begin(), m_path.end(), '/') - 2, 1)) * 2,
        ' '
    );

    ROS_INFO("%sloading %s %s", indent.c_str(), getType(), getPath());

    ros::param::get(getControllerPath("enable"), m_enable);

    return true;
}

bool controller::init()
{
    return true;
}

void controller::update(ros::Time time, ros::Duration period)
{
}

string controller::getControllerPath(const char* controllerName)
{
    return m_path + "/" + controllerName;
}
