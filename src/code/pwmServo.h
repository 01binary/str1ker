/*
                                                                                     @@@@@@@                  
 @@@@@@@@@@@@  @@@@@@@@@@@@   @@@@@@@@@@@@       @  @@@@@@@@@@@@@  @           @  @@@       @@@  @@@@@@@@@@@@ 
@              @ @           @            @    @ @  @              @        @@@      @@@@@@@    @            @
 @@@@@@@@@@@@  @   @         @@@@@@@@@@@@@   @   @   @             @   @@@@@      @@@       @@@ @@@@@@@@@@@@@ 
             @ @     @       @            @      @    @@           @@@@      @                  @            @
 @@@@@@@@@@@@  @       @     @            @      @      @@@@@@@@@  @          @   @@@       @@@ @            @
                                                                                     @@@@@@@                  
 pwmServo.h

 PWM Servo Controller
 Created 1/19/2021

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
    // Max speed for PWM pulse
    const int MAX_SPEED = 255;

private:
    // Left PWM pin
    int m_lpwm;

    // Right PWM pin
    int m_rpwm;

    // Potentiometer measuring absolute position
    potentiometer* m_pot;

public:
    pwmServo(const char* path);

public:
    // Get display type
    virtual const char* getType();

    // Initialize
    virtual bool init();

    // Get absolute position using related potentiometer
    virtual double getPos();

    // Move to absolute position by tracking potentiometer
    virtual void setPos(double pos);

    // Move by delta
    virtual void deltaPos(double delta);

    // Deserialize from settings
    virtual void deserialize();

public:
    // Create instance
    static controller* create(const char* path);
};

} // namespace str1ker
