/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 dynamixelPro.h

 Dynamixel Pro Servo Controller
 Created 1/19/2021

 Copyright (C) 2021 Valeriy Novytskyy
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

    // Publisher
    ros::Publisher m_pub;

public:
    dynamixelPro(class robot& robot, const char* path);
    dynamixelPro(class robot& robot, const char* path, int id);

public:
    // Get display type
    virtual const char* getType();

    // Initialize the servo
    virtual bool init();

    // Get servo position
    virtual double getPos();

    // Set servo position
    virtual void setPos(double target);

    // Rotate servo given delta in radians
    virtual void deltaPos(double delta);

    // Get absolute angle from encoder in radians
    virtual double getAngle();

    // Move to absolute angle in radians
    virtual void setAngle(double angle);

    // Load from settings
    virtual void deserialize(ros::NodeHandle node);

    // Publish current position
    virtual void publish();

public:
    // Create instance
    static controller* create(class robot& robot, const char* path);
};

} // namespace str1ker
