/*
                                                                                     @@@@@@@                  
 @@@@@@@@@@@@  @@@@@@@@@@@@   @@@@@@@@@@@@       @  @@@@@@@@@@@@@  @           @  @@@       @@@  @@@@@@@@@@@@ 
@              @ @           @            @    @ @  @              @        @@@      @@@@@@@    @            @
 @@@@@@@@@@@@  @   @         @@@@@@@@@@@@@   @   @   @             @   @@@@@      @@@       @@@ @@@@@@@@@@@@@ 
             @ @     @       @            @      @    @@           @@@@      @                  @            @
 @@@@@@@@@@@@  @       @     @            @      @      @@@@@@@@@  @          @   @@@       @@@ @            @
                                                                                     @@@@@@@                  
 linear.h

 PWM linear actuator controller
 Created 1/21/2021

 This software is licensed under GNU GPLv3
*/

#ifndef STR1KER_LINEAR_H
#define STR1KER_LINEAR_H

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include "pwmServo.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| linear class
\*----------------------------------------------------------*/

class linear : public pwmServo
{
public:
    // Actuator type
    static const char TYPE[];

public:
    linear(const char* path);
    linear(const char* path, int lwpm, int rpwm, int time);

public:
    // Get display type
    virtual const char* getType();

    // Extend linear actuator by amount
    void extend(double amount = 1.0);

    // Contract linear actuator by amount
    void contract(double amount = 1.0);

public:
    // Create instance
    static actuator* create(const char* path);
};

} // namespace str1ker

#endif // STR1KER_LINEAR_H