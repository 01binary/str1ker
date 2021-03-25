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

#include <ros/ros.h>
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

potentiometer::potentiometer(const char* path) :
    controller(path),
    m_adc(NULL),
    m_id(0),
    m_sampleId(0),
    m_avgId(0),
    m_samples(SAMPLE_COUNT),
    m_avg(AVG_COUNT),
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

    if (m_adc)
    {
        if (!m_adc->init())
        {
            ROS_ERROR("  %s cannot read from adc", getPath());
            return false;
        }

        m_lastSample = round2(m_adc->getValue(m_id) / (double)m_adc->getMaxValue());
    }

    return true;
}

double potentiometer::getPos()
{
    if (!m_enable || !m_adc) return m_lastSample;

    double sample = round2(m_adc->getValue(m_id) / (double)m_adc->getMaxValue());

    m_samples[m_sampleId++] = sample;

    if (m_sampleId >= SAMPLE_COUNT)
    {
        m_sampleId = 0;

        m_avg[m_avgId++] = round2(
            accumulate(m_samples.begin(), m_samples.end(), 0.0) / SAMPLE_COUNT);

        if (m_avgId >= AVG_COUNT)
        {
            m_avgId = 0;

            double avg  = round2(accumulate(m_avg.begin(), m_avg.end(), 0.0) / AVG_COUNT);
            double min = *std::min_element(m_avg.begin(), m_avg.end());
            double max = *std::max_element(m_avg.begin(), m_avg.end());

            if (abs(avg - min) <= AVG_THRESHOLD)
                avg = min;
            else
                avg = max;

            m_lastSample = avg;
        }
    }

    return m_lastSample;
}

void potentiometer::deserialize(ros::NodeHandle node)
{
    controller::deserialize(node);

    ros::param::get(getControllerPath("channel"), m_id);

    string adcName;
    ros::param::get(getControllerPath("adc"), adcName);
    m_adc = controllerFactory::deserialize<adc>(adcName.c_str());

    m_pub = node.advertise<std_msgs::Float32>(getPath(), 256);
}

void potentiometer::publish()
{
    std_msgs::Float32 msg;
    msg.data = getPos();
    m_pub.publish(msg);
}

controller* potentiometer::create(const char* path)
{
    return new potentiometer(path);
}

double potentiometer::round2(double num)
{
    return std::floor((num * 100) + 0.5) / 100.0;
}
