/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 potentiometer.cpp

 Potentiometer Controller Implementation
 Created 1/27/2021

 Copyright (C) 2021 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <numeric>
#include <algorithm>
#include <std_msgs/Float32.h>
#include "potentiometer.h"
#include "adc.h"
#include "controllerFactory.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace std;
using namespace str1ker;

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const char potentiometer::TYPE[] = "potentiometer";

/*----------------------------------------------------------*\
| potentiometer implementation
\*----------------------------------------------------------*/

REGISTER_CONTROLLER(potentiometer)

potentiometer::potentiometer(class robot& robot, const char* path) :
    controller(robot, path),
    m_adc(NULL),
    m_channel(0),
    m_lastSample(0.0)
{
}

const char* potentiometer::getType()
{
    return potentiometer::TYPE;
}

bool potentiometer::init()
{
    if (!m_enable) return true;

    if (!m_adc)
        ROS_WARN("  %s does not have assigned adc to read from", getPath());

    return true;
}

double potentiometer::getPos()
{
    return m_lastSample;
}

void potentiometer::deserialize(ros::NodeHandle node)
{
    controller::deserialize(node);

    ros::param::get(getControllerPath("channel"), m_channel);

    string adcName;
    ros::param::get(getControllerPath("adc"), adcName);
    m_adc = controllerFactory::deserialize<adc>(adcName.c_str());

    m_pub = node.advertise<std_msgs::Float32>(getPath(), 256);
}

void potentiometer::publish()
{
    if (m_enable && m_adc)
    {
        m_lastSample = double(m_adc->getValue(m_channel)) / double(m_adc->getMaxValue());

        std_msgs::Float32 msg;
        msg.data = m_lastSample;
        m_pub.publish(msg);
    }
}

controller* potentiometer::create(robot& robot, const char* path)
{
    return new potentiometer(robot, path);
}
