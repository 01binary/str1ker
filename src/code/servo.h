/*
                                                                                     @@@@@@@                  
 @@@@@@@@@@@@  @@@@@@@@@@@@   @@@@@@@@@@@@       @  @@@@@@@@@@@@@  @           @  @@@       @@@  @@@@@@@@@@@@ 
@              @ @           @            @    @ @  @              @        @@@      @@@@@@@    @            @
 @@@@@@@@@@@@  @   @         @@@@@@@@@@@@@   @   @   @             @   @@@@@      @@@       @@@ @@@@@@@@@@@@@ 
             @ @     @       @            @      @    @@           @@@@      @                  @            @
 @@@@@@@@@@@@  @       @     @            @      @      @@@@@@@@@  @          @   @@@       @@@ @            @
                                                                                     @@@@@@@                  
 servo.h

 Servo base class
 Created 1/19/2021

 This software is licensed under GNU GPLv3
*/

#ifndef STR1KER_SERVO_H
#define STR1KER_SERVO_H

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
| pwmServo class
\*----------------------------------------------------------*/

class servo : public controller
{
public:
    servo(const char* path) : controller(path) {}

public:
    // Initialize servo controller
    virtual bool init() = 0;

    // Get absolute servo position
    virtual double getPos() = 0;

    // Set absolute servo position
    virtual void setPos(double pos) = 0;

    // Rotate servo
    virtual void deltaPos(double delta) = 0;
};

} // namespace str1ker

#endif // STR1KER_SERVO_H
