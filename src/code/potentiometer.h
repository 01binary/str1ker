/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 potentiometer.h

 Potentiometer Controller
 Created 1/27/2021

 Copyright (C) 2021 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include "adc.h"
#include "controller.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| potentiometer class
\*----------------------------------------------------------*/

class potentiometer : public controller
{
public:
    // Controller type
    static const char TYPE[];

private:
    // Number of samples to average for de-noising
    const int SAMPLE_COUNT = 8;

    // Number of averages to analyze
    const int AVG_COUNT = 4;

    // Threshold for picking stable average
    const double AVG_THRESHOLD = 0.2;

    // Analog to digital converter (ADC) for reading measurements
    adc* m_adc;

    // Channel index to use when reading from ADC
    int m_id;

    // Samples collected
    std::vector<double> m_samples;

    // Current sample index
    int m_sampleId;

    // Averages collected
    std::vector<double> m_avg;

    // Current average index
    int m_avgId;

    // Last reading
    double m_lastSample;

    // Publisher
    ros::Publisher m_pub;

public:
    potentiometer(const char* path);

public:
    // Get display type
    virtual const char* getType();

    // Initialize potentiometer controller
    virtual bool init();

    // Get absolute position
    virtual double getPos();

    // Deserialize from settings
    virtual void deserialize(ros::NodeHandle node);

    // Publish current position
    virtual void publish();

public:
    // Create instance
    static controller* create(const char* path);

private:
    // Round number to two decimal places
    static double round2(double num);
};

} // namespace str1ker
