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
#include <math.h>
#include <algorithm>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <angles/angles.h>
#include "potentiometer.h"
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
    m_channel(0),
    m_reading(0),
    m_invert(false),
    m_minReading(0),
    m_maxReading(0),
    m_pos(0.0),
    m_min(0.0),
    m_max(0.0)
{
}

const char* potentiometer::getType()
{
    return potentiometer::TYPE;
}

double potentiometer::getPos()
{
    return m_pos;
}

void potentiometer::deserialize(ros::NodeHandle node)
{
    controller::deserialize(node);

    ros::param::get(getControllerPath("channel"), m_channel);

    ros::param::get(getControllerPath("min"), m_minReading);
    ros::param::get(getControllerPath("max"), m_maxReading);

    if (m_minReading > m_maxReading) {
        // Support inverting value during normalization
        m_invert = true;
        int swap = m_maxReading;
        m_maxReading = m_minReading;
        m_minReading = swap;
    }

    if (ros::param::get(getControllerPath("minAngle"), m_min))
        m_min = angles::from_degrees(m_min);

    if (ros::param::get(getControllerPath("maxAngle"), m_max))
        m_max = angles::from_degrees(m_max);

    string adcTopicName;
    ros::param::get(getControllerPath("adc"), adcTopicName);

    m_sub = node.subscribe<Adc>(
        adcTopicName,
        SUBSCRIBE_QUEUE_SIZE,
        readingCallback,
        this
    );
}

void potentiometer::readingCallback(const Adc::ConstPtr& msg)
{
    if (!m_enable) return;

    // Get the raw reading
    m_reading = int(((const uint16_t*)&msg->adc0)[m_channel]);

    // Calculate normalized value
    double norm = normalize(m_reading, m_minReading, m_maxReading, m_invert);

    // Map output position from normalized value
    m_pos = scale(norm, m_min, m_max);
}

controller* potentiometer::create(robot& robot, const char* path)
{
    return new potentiometer(robot, path);
}

double potentiometer::normalize(int value, int min, int max, bool invert)
{
    if (value < min) return 0.0;
    if (value > max) return 1.0;

    double norm = double(value - min) / double(max - min);

    if (invert) return 1.0 - norm;
    return norm;
}

double potentiometer::scale(double value, double min, double max)
{
    return value * (max - min) + min;
}
