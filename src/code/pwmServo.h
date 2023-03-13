/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 pwmServo.h

 PWM Servo Controller
 Created 1/19/2021

 Copyright (C) 2021 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <str1ker/Pwm.h>
#include "servo.h"
#include "potentiometer.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| pwmServo class
\*----------------------------------------------------------*/

class pwmServo : public servo
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

    // Minimum speed
    double m_min;

    // Maximum speed
    double m_max;

    // Current velocity
    double m_velocity;

    // Potentiometer as absolute encoder
    std::shared_ptr<potentiometer> m_encoder;

    // PWM publisher
    ros::Publisher m_pub;

public:
    pwmServo(class robot& robot, const char* path);

public:
    // Get display type
    virtual const char* getType();

    // Initialize
    virtual bool init(ros::NodeHandle node);

    // Get position from encoder
    virtual double getPos();

    // Move to absolute position
    virtual void setPos(double target);

    // Get current velocity
    virtual double getVelocity();

    // Set speed and direction directly (+/-)
    virtual bool setVelocity(double velocity);

    // Get minimum speed for ramping up
    double getMinSpeed();

    // Get maximum speed for ramping up
    double getMaxSpeed();

    // Calculate speed between min and max based on ramp position
    double rampSpeed(double ramp);

    // Deserialize from settings
    virtual void deserialize(ros::NodeHandle node);

public:
    // Create instance
    static controller* create(class robot& robot, const char* path);
};

} // namespace str1ker
