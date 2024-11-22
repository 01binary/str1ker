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
#include "controllerUtilities.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace str1ker;
using namespace std;

/*----------------------------------------------------------*\
| controller implementation
\*----------------------------------------------------------*/

controller::controller(ros::NodeHandle node, string type, string path)
    : m_node(node)
    , m_type(type)
    , m_name(controllerUtilities::getControllerName(path))
    , m_path(path)
    , m_enable(true)
{
}

ros::NodeHandle controller::getNode()
{
    return m_node;
}

const string& controller::getType()
{
    return m_type;
}

const string& controller::getName()
{
    return m_name;
}

const string& controller::getPath()
{
    return m_path;
}

std::string controller::getParentName()
{
    return controllerUtilities::getParentName(m_path.c_str());
}

std::string controller::getParentPath()
{
    return controllerUtilities::getParentPath(m_path.c_str());
}

const bool controller::isEnabled()
{
    return m_enable;
}

bool controller::configure()
{
    ROS_INFO("  loading %s %s", getType().c_str(), getPath().c_str());

    ros::param::get(getChildPath("enable"), m_enable);

    return true;
}

bool controller::init()
{
    return true;
}

void controller::update(ros::Time time, ros::Duration period)
{
}

string controller::getChildPath(const string& controllerName)
{
    return m_path + "/" + controllerName;
}
