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
#include <str1ker/Pwm.h>
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
    // Default trigger duration
    const double DEFAULT_TRIGGER_DURATION_SEC = 0.023;

private:
    // Publishing queue size
    const int QUEUE_SIZE = 4;

    // Publish topic
    std::string m_topic = "pwm";

    // Publisher trigger channel
    int m_channel;

    // Trigger duration in seconds
    double m_triggerDurationSec;

    // Publisher to solenoid driver
    ros::Publisher m_pub;

    // Triggered status
    bool m_triggered;

    // Reset time if triggered
    ros::Time m_resetTime;

public:
    solenoid(ros::NodeHandle node, std::string path);

public:

    // Deserialize from settings
    virtual bool configure();

    // Initialize
    virtual bool init();

    // Update
    virtual void update(ros::Time time, ros::Duration period);

    // Momentary trigger
    void trigger();
    bool isTriggered();

public:
    // Create instance
    static controller* create(ros::NodeHandle node, std::string path);
};

} // namespace str1ker
