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
    const int DUTY_CYCLE = 0xFF;

private:
    // Left PWM pin
    int m_gpioLPWM;

    // Right PWM pin
    int m_gpioRPWM;

    // Ramp min (defaults to 1.0)
    double m_minSpeed;

    // Ramp max (defaults to 1.0)
    double m_maxSpeed;

    // Potentiometer as absolute encoder
    std::shared_ptr<potentiometer> m_encoder;

public:
    pwmServo(class robot& robot, const char* path);

public:
    // Get display type
    virtual const char* getType();

    // Initialize
    virtual bool init();

    // Get normalized absolute position from encoder
    virtual double getPos();

    // Move to absolute position by tracking encoder
    virtual void setPos(double target);

    // Get absolute angle from encoder in radians
    virtual double getAngle();

    // Move to absolute angle in radians
    virtual void setAngle(double angle);

    // Set speed and direction directly (+/-)
    bool setVelocity(double speed);

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

protected:
    void handlePwmError(int error);
};

} // namespace str1ker
