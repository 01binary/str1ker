/*
                                                                                     @@@@@@@                  
 @@@@@@@@@@@@  @@@@@@@@@@@@   @@@@@@@@@@@@       @  @@@@@@@@@@@@@  @           @  @@@       @@@  @@@@@@@@@@@@ 
@              @ @           @            @    @ @  @              @        @@@      @@@@@@@    @            @
 @@@@@@@@@@@@  @   @         @@@@@@@@@@@@@   @   @   @             @   @@@@@      @@@       @@@ @@@@@@@@@@@@@ 
             @ @     @       @            @      @    @@           @@@@      @                  @            @
 @@@@@@@@@@@@  @       @     @            @      @      @@@@@@@@@  @          @   @@@       @@@ @            @
                                                                                     @@@@@@@                  
 dynamixelPro.h

 dynamixelPro servo controller
 Created 1/19/2021

 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include "servo.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| dynamixelPro class
\*----------------------------------------------------------*/

class dynamixelPro : public servo
{
public:
    // Controller type
    static const char TYPE[];

private:
    // dynamixelPro interface
    static DynamixelWorkbench* s_wb;

    // Servo id on serial bus
    int m_id;

    // Current position control
    const ControlItem* m_pos;

    // Goal position control
    const ControlItem* m_goal;

public:
    dynamixelPro(const char* path);
    dynamixelPro(const char* path, int id);

public:
    // Get display type
    virtual const char* getType();

    // Initialize the servo
    virtual bool init();

    // Get servo position
    virtual double getPos();

    // Set servo position
    virtual void setPos(double pos);

    // Rotate servo given delta in radians
    virtual void deltaPos(double delta);

    // Load from settings
    virtual void deserialize();

public:
    // Create instance
    static controller* create(const char* path);
};

} // namespace str1ker
