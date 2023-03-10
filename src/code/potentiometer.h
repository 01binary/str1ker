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
    // Message publishing queue size
    const int PUBLISH_QUEUE_SIZE = 16;

    // Analog to digital converter (ADC) for reading measurements
    adc* m_adc;

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

    // Normalized reading (between min and max)
    double m_pos;

    // Last reading as rotation angle in radians
    double m_angle;

    // Min rotation angle
    double m_minAngle;

    // Max rotation angle
    double m_maxAngle;

    // Rotation angle publisher
    ros::Publisher m_pub;

public:
    potentiometer(class robot& robot, const char* path);

public:
    // Get display type
    virtual const char* getType();

    // Initialize potentiometer controller
    virtual bool init();

    // Get absolute position
    double getPos();

    // Get absolute angle
    double getAngle();

    // Map normalized position from angle
    double getPos(double angle);

    // Deserialize from settings
    virtual void deserialize(ros::NodeHandle node);

    // Publish current position from ADC
    virtual void update();

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
