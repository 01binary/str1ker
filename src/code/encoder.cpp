/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 encoder.cpp

 encoder Controller Implementation
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
#include "encoder.h"
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

const char encoder::TYPE[] = "encoder";

/*----------------------------------------------------------*\
| encoder implementation
\*----------------------------------------------------------*/

REGISTER_CONTROLLER(encoder)

encoder::encoder(class robot& robot, const char* path) :
    controller(robot, path),
    m_adc(NULL),
    m_channel(0),
    m_reading(0),
    m_invert(false),
    m_minReading(0),
    m_maxReading(0),
    m_pos(0.0),
    m_angle(0.0),
    m_minAngle(0.0),
    m_maxAngle(0.0)
{
}

const char* encoder::getType()
{
    return encoder::TYPE;
}

bool encoder::init()
{
    if (!m_enable) return true;

    if (!m_adc)
        ROS_WARN("  %s does not have assigned adc to read from", getPath());

    return true;
}

double encoder::getPos()
{
    return m_pos;
}

double encoder::getAngle()
{
    return m_angle;
}

double encoder::getPos(double angle)
{
    if (angle < m_minAngle) angle = m_minAngle;
    if (angle > m_maxAngle) angle = m_maxAngle;

    return (angle - m_minAngle) / (m_maxAngle - m_minAngle);
}

void encoder::deserialize(ros::NodeHandle node)
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

    ros::param::get(getControllerPath("minAngle"), m_minAngle);
    m_minAngle = angles::from_degrees(m_minAngle);

    ros::param::get(getControllerPath("maxAngle"), m_maxAngle);
    m_maxAngle = angles::from_degrees(m_maxAngle);

    string adcName;
    ros::param::get(getControllerPath("adc"), adcName);
    m_adc = controllerFactory::deserialize<adc>(adcName.c_str());

    if (m_adc) {
        m_reading = m_adc->getMaxValue();
        m_pos = scale(m_reading, m_minReading, m_maxReading);
        m_angle = m_maxAngle;
    }

    m_pub = node.advertise<geometry_msgs::QuaternionStamped>(
        getPath(),
        PUBLISH_QUEUE_SIZE
    );
}

void encoder::publish()
{
    if (!m_enable || !m_adc || !m_pub) return;

    // Get the raw reading
    m_reading = m_adc->getValue();

    // Calculate normalized position
    m_pos = normalize(m_reading, m_minReading, m_maxReading, m_invert);

    // Map angle from normalized position
    m_angle = scale(m_pos, m_minAngle, m_maxAngle);

    // Publish angle
    tf2::Stamped<tf2::Quaternion> q;
    q.setRPY(m_angle, 0.0, 0.0);
    q.normalize();

    tf2::Stamped<tf2::Quaternion> payload(q, ros::Time::now(), "world");
    geometry_msgs::QuaternionStamped msg = tf2::toMsg(payload);

    m_pub.publish(msg);
}

controller* encoder::create(robot& robot, const char* path)
{
    return new encoder(robot, path);
}

double encoder::normalize(int value, int min, int max, bool invert)
{
    if (value < min) return 0.0;
    if (value > max) return 1.0;

    double norm = double(value - min) / double(max - min);

    if (invert) return 1.0 - norm;
    return norm;
}

double encoder::scale(double value, double min, double max)
{
    return value * (max - min) + min;
}
