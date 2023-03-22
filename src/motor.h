/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 motor.h

 PWM Motor Controller
 Created 1/19/2021

 Copyright (C) 2021 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <str1ker/Pwm.h>
#include "potentiometer.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| motor class
\*----------------------------------------------------------*/

class motor: public controller
{
public:
    // Controller type
    static const char TYPE[];

private:
    // PWM publishing queue size
    const int QUEUE_SIZE = 4;

    // Max PWM duty cycle
    const uint8_t DUTY_CYCLE = 0xFF;

private:
    // PWM topic
    std::string m_topic;

    // LPWM channel
    int m_lpwm;

    // RPWM channel
    int m_rpwm;

    // Current velocity
    double m_velocity;

    // Potentiometer as absolute encoder
    std::shared_ptr<potentiometer> m_encoder;

    // Publisher to PWM node that runs motors
    ros::Publisher m_pub;

public:
    motor(class robot& robot, const char* path);

public:
    // Get display type
    virtual const char* getType();

    // Initialize
    virtual bool init(ros::NodeHandle node);

    // Get position from associated encoder
    double getPos();

    // Get velocity
    double getVelocity();

    // Set velocity
    bool setVelocity(double velocity);

    // Deserialize from settings
    virtual void configure(ros::NodeHandle node);

public:
    // Create instance
    static controller* create(class robot& robot, const char* path);
};

} // namespace str1ker
