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
    // Message publishing queue size
    const int PUBLISH_QUEUE_SIZE = 16;

    // Names of joints to publish
    static const char* JOINT_NAMES[];

    // Rotation servo
    std::shared_ptr<servo> m_shoulder;

    // Lift servo
    std::shared_ptr<servo> m_upperarm;

    // Extend servo
    std::shared_ptr<servo> m_forearm;

    // Trigger solenoid
    std::shared_ptr<solenoid> m_trigger;

    // Joint states publisher
    ros::Publisher m_pub;

public:
    arm(class robot& robot, const char* path);

public:
    // Get display type
    virtual const char* getType();

    // Trigger arm
    void trigger(double durationSeconds = 0.023);

    // Load arm controller settings
    virtual void deserialize(ros::NodeHandle node);

    // Initialize arm controllers
    virtual bool init();

    // Update joints
    virtual void update();

public:
    // Create instance
    static controller* create(class robot& robot, const char* path);
};

} // namespace str1ker
