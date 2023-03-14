/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 solenoid.h

 Solenoid Controller
 Created 1/19/2021

 Copyright (C) 2021 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <string>
#include "controller.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| solenoid class
\*----------------------------------------------------------*/

class solenoid : public controller
{
public:
    // Controller type
    static const char TYPE[];

private:
    // Output topic
    std::string m_topic;

    // Output channel
    int m_channel;

public:
    solenoid(class robot& robot, const char* path);

public:
    // Get display type
    virtual const char* getType();

    // Initialize
    bool init(ros::NodeHandle node);

    // Momentary trigger
    void trigger(double durationSeconds);

    // Deserialize from settings
    void deserialize(ros::NodeHandle node);

public:
    // Create instance
    static controller* create(robot& robot, const char* path);
};

} // namespace str1ker
