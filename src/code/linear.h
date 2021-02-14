/*
                                                                                     @@@@@@@                  
 @@@@@@@@@@@@  @@@@@@@@@@@@   @@@@@@@@@@@@       @  @@@@@@@@@@@@@  @           @  @@@       @@@  @@@@@@@@@@@@ 
@              @ @           @            @    @ @  @              @        @@@      @@@@@@@    @            @
 @@@@@@@@@@@@  @   @         @@@@@@@@@@@@@   @   @   @             @   @@@@@      @@@       @@@ @@@@@@@@@@@@@ 
             @ @     @       @            @      @    @@           @@@@      @                  @            @
 @@@@@@@@@@@@  @       @     @            @      @      @@@@@@@@@  @          @   @@@       @@@ @            @
                                                                                     @@@@@@@                  
 linear.h

 PWM Linear Actuator Controller
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
    // Controller type
    static const char TYPE[];

public:
    linear(const char* path);

public:
    // Get display type
    virtual const char* getType();

    // Extend linear actuator
    void extend(double delta = 1.0);

    // Contract linear actuator
    void contract(double delta = 1.0);

public:
    // Create instance
    static controller* create(const char* path);
};

} // namespace str1ker

#endif // STR1KER_LINEAR_H