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

#include <ros/ros.h>
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
    // Message subscription queue size
    const int SUBSCRIBE_QUEUE_SIZE = 8;

    // Channel index to use when reading from ADC
    int m_channel;

    // Last reading
    int m_reading;

    // Min reading
    int m_minReading;

    // Max reading
    int m_maxReading;

    // Invert readings (min > max) if installed backwards
    bool m_invert;

    // Last position
    double m_pos;

    // Min position
    double m_min;

    // Max position
    double m_max;

    // Analog reading subscriber
    ros::Subscriber m_sub;

public:
    potentiometer(class robot& robot, const char* path);

public:
    // Get display type
    virtual const char* getType();

    // Get absolute position
    double getPos();

    // Deserialize from settings
    virtual void deserialize(ros::NodeHandle node);

    // Subscribe callback for ADC readings
    void readingCallback(const msgs::Adc::ConstPtr& msg);

public:
    // Create instance
    static controller* create(class robot& robot, const char* path);

private:
    // Normalize reading
    static double normalize(int value, int min, int max, bool invert);

    // Scale reading
    static double scale(double value, double min, double max);
};

} // namespace str1ker
