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

    // Last reading as rotation angle in radians
    double m_rotation;

    // Offset angle for advertised rotation in degrees
    double m_offset;

    // Range for advertised rotation in degrees
    double m_range;

    // Rotation angle publisher
    ros::Publisher m_anglePublisher;

public:
    potentiometer(class robot& robot, const char* path);

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
    static controller* create(class robot& robot, const char* path);
};

} // namespace str1ker
