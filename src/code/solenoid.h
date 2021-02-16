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

#pragma once

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
| solenoid class
\*----------------------------------------------------------*/

class solenoid : public controller
{
public:
    // Controller type
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
    static controller* create(const char* path);
};

} // namespace str1ker
