/*
                                                                                     @@@@@@@                  
 @@@@@@@@@@@@  @@@@@@@@@@@@   @@@@@@@@@@@@       @  @@@@@@@@@@@@@  @           @  @@@       @@@  @@@@@@@@@@@@ 
@              @ @           @            @    @ @  @              @        @@@      @@@@@@@    @            @
 @@@@@@@@@@@@  @   @         @@@@@@@@@@@@@   @   @   @             @   @@@@@      @@@       @@@ @@@@@@@@@@@@@ 
             @ @     @       @            @      @    @@           @@@@      @                  @            @
 @@@@@@@@@@@@  @       @     @            @      @      @@@@@@@@@  @          @   @@@       @@@ @            @
                                                                                     @@@@@@@                  
 dynamixel.h

 Dynamixel servo controller
 Created 1/19/2021

 This software is licensed under GNU GPLv3
*/

#ifndef STR1KER_DYNAMIXEL_SERVO_H
#define STR1KER_DYNAMIXEL_SERVO_H

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
| dynamixel class
\*----------------------------------------------------------*/

class dynamixel : public servo
{
public:
    // Controller type
    static const char TYPE[];

private:
    // Dynamixel interface
    static DynamixelWorkbench* s_wb;

    // Servo id on serial bus
    int m_id;

    // Current position control
    const ControlItem* m_pos;

    // Goal position control
    const ControlItem* m_goal;

public:
    dynamixel(const char* path);
    dynamixel(const char* path, int id);

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

#endif // STR1KER_DYNAMIXEL_SERVO_H