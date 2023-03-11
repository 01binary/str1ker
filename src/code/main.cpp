/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 main.cpp

 Str1ker Drumming Robot Main Module
 Created 01/20/2021

 Copyright (C) 2021 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros/ros.h>
#include "robot.h"
#include "arm.h"

/*----------------------------------------------------------*\
| Namespaces
\*----------------------------------------------------------*/

using namespace str1ker;

/*----------------------------------------------------------*\
| Module
\*----------------------------------------------------------*/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot");

    ros::NodeHandle node;
    robot robot(node);

    if (!robot
        .logo()
        .deserialize()
        .init())
    {
        return 1;
    }

    robot.run();

    return 0;
}
