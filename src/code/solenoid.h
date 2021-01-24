/*
                                                                                     @@@@@@@                  
 @@@@@@@@@@@@  @@@@@@@@@@@@   @@@@@@@@@@@@       @  @@@@@@@@@@@@@  @           @  @@@       @@@  @@@@@@@@@@@@ 
@              @ @           @            @    @ @  @              @        @@@      @@@@@@@    @            @
 @@@@@@@@@@@@  @   @         @@@@@@@@@@@@@   @   @   @             @   @@@@@      @@@       @@@ @@@@@@@@@@@@@ 
             @ @     @       @            @      @    @@           @@@@      @                  @            @
 @@@@@@@@@@@@  @       @     @            @      @      @@@@@@@@@  @          @   @@@       @@@ @            @
                                                                                     @@@@@@@                  
 solenoid.h

 Solenoid class
 Created 1/19/2021

 This software is licensed under GNU GPLv3
*/

#ifndef STR1KER_SOLENOID_H
#define STR1KER_SOLENOID_H

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <string>
#include "actuator.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| solenoid class
\*----------------------------------------------------------*/

class solenoid : public actuator
{
public:
    // Actuator type
    static const char TYPE[];

private:
    // Output pin
    int m_output;

public:
    solenoid(const char* path);
    solenoid(const char* path, int output);

public:
    // Get display type
    virtual const char* getType();

    // Initialize
    bool init();

    // Momentary trigger
    void trigger(double durationSeconds);

    // Deserialize from settings
    void deserialize();

public:
    // Create instance
    static actuator* create(const char* path);
};

} // namespace str1ker

#endif // STR1KER_SOLENOID_H
