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
    m_rotation(0.0),
    m_offset(0.0),
    m_range(360.0)
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
    return m_rotation;
}

void potentiometer::deserialize(ros::NodeHandle node)
{
    controller::deserialize(node);

    ros::param::get(getControllerPath("channel"), m_channel);

    ros::param::get(getControllerPath("offset"), m_offset);
    m_offset = angles::to_degrees(m_offset);

    ros::param::get(getControllerPath("range"), m_range);
    m_range = angles::to_degrees(m_range);

    string adcName;
    ros::param::get(getControllerPath("adc"), adcName);
    m_adc = controllerFactory::deserialize<adc>(adcName.c_str());

    m_pub = node.advertise<geometry_msgs::QuaternionStamped>(
        getPath(),
        PUBLISH_QUEUE_SIZE
    );
}

void potentiometer::publish()
{
    if (!m_enable || !m_adc || !m_pub) return;

    m_rotation =
        m_offset +
        double(m_adc->getValue(m_channel)) /
        double(m_adc->getMaxValue()) *
        m_range;

    tf2::Stamped<tf2::Quaternion> q;
    q.setRPY(m_rotation, 0, 0);
    q.normalize();

    tf2::Stamped<tf2::Quaternion> payload(q, ros::Time::now(), "world");
    geometry_msgs::QuaternionStamped msg = tf2::toMsg(payload);

    m_pub.publish(msg);
}

controller* potentiometer::create(robot& robot, const char* path)
{
    return new potentiometer(robot, path);
}
