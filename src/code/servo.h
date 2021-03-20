/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 servo.h

 Servo Controller Base Class
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
| servo class
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
