/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 linear.cpp

 PWM Linear Actuator Controller Implementation
 Created 1/19/2021

 Copyright (C) 2021 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros/ros.h>
#include "robot.h"
#include "linear.h"
#include "controllerFactory.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace std;
using namespace str1ker;

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const char linear::TYPE[] = "linear";

/*----------------------------------------------------------*\
| linear implementation
\*----------------------------------------------------------*/

REGISTER_CONTROLLER(linear)

linear::linear(robot& robot, const char* path) :
    pwmServo(robot, path)
{
}

const char* linear::getType()
{
    return linear::TYPE;
}

void linear::extend(double delta)
{
    deltaPos(delta);
}

void linear::contract(double delta)
{
    deltaPos(-delta);
}

controller* linear::create(robot& robot, const char* path)
{
    return new linear(robot, path);
}
