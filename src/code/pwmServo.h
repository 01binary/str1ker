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

#ifndef STR1KER_BASIC_SERVO_H
#define STR1KER_BASIC_SERVO_H

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include "servo.h"

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

    // Current position
    double m_pos;

    // Time it takes to reach limit in seconds
    int m_time;

public:
    pwmServo(const char* path);
    pwmServo(const char* path, int lwpm, int rpwm, int time);

public:
    // Get display type
    virtual const char* getType();

    // Initialize
    virtual bool init();

    // Get current position
    virtual double getPos();

    // Rotate by delta
    virtual void rotate(double delta);

    // Deserialize from settings
    virtual void deserialize();

public:
    // Create instance
    static controller* create(const char* path);
};

} // namespace str1ker

#endif // STR1KER_BASIC_SERVO_H