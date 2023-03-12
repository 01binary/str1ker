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

#include "servo.h"
#include "potentiometer.h"
#include <actionlib/client/simple_action_client.h>
#include <str1ker/PwmAction.h>

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
    // Max PWM duty cycle
    const uint8_t DUTY_CYCLE = 0xFF;

private:
    // PWM action topic
    std::string m_topic;

    // PWM channel
    int m_channel;

    // Minimum speed
    double m_min;

    // Maximum speed
    double m_max;

    // Current velocity
    double m_velocity;

    // Potentiometer as absolute encoder
    std::shared_ptr<potentiometer> m_encoder;

    // Action client for PWM
    actionlib::SimpleActionClient<str1ker::PwmAction> m_pwm("robot/pwm");

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
    virtual bool setVelocity(double speed);

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
