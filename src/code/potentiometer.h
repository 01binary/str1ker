/*
                                                                                     @@@@@@@                  
 @@@@@@@@@@@@  @@@@@@@@@@@@   @@@@@@@@@@@@       @  @@@@@@@@@@@@@  @           @  @@@       @@@  @@@@@@@@@@@@ 
@              @ @           @            @    @ @  @              @        @@@      @@@@@@@    @            @
 @@@@@@@@@@@@  @   @         @@@@@@@@@@@@@   @   @   @             @   @@@@@      @@@       @@@ @@@@@@@@@@@@@ 
             @ @     @       @            @      @    @@           @@@@      @                  @            @
 @@@@@@@@@@@@  @       @     @            @      @      @@@@@@@@@  @          @   @@@       @@@ @            @
                                                                                     @@@@@@@                  
 potentiometer.h

 Potentiometer Controller
 Created 1/27/2021

 This software is licensed under GNU GPLv3
*/

#ifndef STR1KER_POTENTIOMETER_H
#define STR1KER_POTENTIOMETER_H

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
| potentiometer class
\*----------------------------------------------------------*/

class potentiometer : public controller
{
public:
    potentiometer(const char* path) : controller(path) {}

public:
    // Initialize potentiometer controller
    virtual bool init() = 0;

    // Get absolute position
    virtual double getPos() = 0;
};

} // namespace str1ker

#endif // STR1KER_POTENTIOMETER_H
