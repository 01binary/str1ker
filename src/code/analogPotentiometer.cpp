/*
                                                                                     @@@@@@@                  
 @@@@@@@@@@@@  @@@@@@@@@@@@   @@@@@@@@@@@@       @  @@@@@@@@@@@@@  @           @  @@@       @@@  @@@@@@@@@@@@ 
@              @ @           @            @    @ @  @              @        @@@      @@@@@@@    @            @
 @@@@@@@@@@@@  @   @         @@@@@@@@@@@@@   @   @   @             @   @@@@@      @@@       @@@ @@@@@@@@@@@@@ 
             @ @     @       @            @      @    @@           @@@@      @                  @            @
 @@@@@@@@@@@@  @       @     @            @      @      @@@@@@@@@  @          @   @@@       @@@ @            @
                                                                                     @@@@@@@                  
 analogPotentiometer.cpp

 Analog Potentiometer Controller implementation
 Created 1/19/2021

 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros/ros.h>
#include <numeric>
#include <algorithm>
#include "analogPotentiometer.h"
#include "controllerFactory.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace std;
using namespace str1ker;

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const char analogPotentiometer::TYPE[] = "potentiometer";

/*----------------------------------------------------------*\
| analogPotentiometer implementation
\*----------------------------------------------------------*/

REGISTER_CONTROLLER(analogPotentiometer)

analogPotentiometer::analogPotentiometer(const char* path) :
    potentiometer(path),
    m_adc(NULL),
    m_id(0),
    m_sampleId(0),
    m_avgId(0),
    m_samples(SAMPLE_COUNT),
    m_avg(AVG_COUNT),
    m_lastSample(0.0)
{
}

const char* analogPotentiometer::getType()
{
    return analogPotentiometer::TYPE;
}

bool analogPotentiometer::init()
{
    if (!m_enable) return true;

    if (m_adc && !m_adc->init())
    {
        ROS_ERROR("  %s cannot read from adc", getPath());
        return false;
    }

    return true;
}

double round2(double num)
{
    return std::floor((num * 100) + 0.5) / 100.0;
}

double analogPotentiometer::getPos()
{
    double sample = m_adc
        ? round2(m_adc->getValue(m_id) / (double)m_adc->getMaxValue())
        : 0.0;

    if (m_lastSample == 0) m_lastSample = sample;

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

void analogPotentiometer::deserialize()
{
    potentiometer::deserialize();

    ros::param::get(getControllerPath("id"), m_id);

    string source;
    ros::param::get(getControllerPath("source"), source);
    m_adc = controllerFactory::deserialize<adc>(source.c_str());
}

controller* analogPotentiometer::create(const char* path)
{
    return new analogPotentiometer(path);
}
