/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 arm.h

 Robot Arm Controller
 Created 1/19/2021

 Copyright (C) 2021 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include "controller.h"
#include "servo.h"
#include "linear.h"
#include "solenoid.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| arm class
\*----------------------------------------------------------*/

class arm : public controller
{
public:
    // Controller type
    static const char TYPE[];

private:
    // Rotation servo
    servo* m_shoulder;

    // Lift servo
    linear* m_upperarm;

    // Extend servo
    linear* m_forearm;

    // Trigger solenoid
    solenoid* m_trigger;

public:
    arm(const char* path);
    ~arm();

public:
    // Get display type
    virtual const char* getType();

    // Rotate arm by delta in possible range
    void rotate(double delta);

    // Raise arm all the way
    void raise(double amount = 1.0);

    // Lower arm all the way
    void lower(double amount = 1.0);

    // Extend arm all the way
    void extend(double amount = 1.0);

    // Contract arm all the way
    void contract(double amount = 1.0);

    // Trigger arm
    void trigger(double durationSeconds = 0.023);

    // Load arm controller settings
    virtual void deserialize(ros::NodeHandle node);

    // Initialize arm controllers
    virtual bool init();

    // Publish topic
    virtual void publish();

public:
    // Create instance
    static controller* create(const char* path);
};

} // namespace str1ker
