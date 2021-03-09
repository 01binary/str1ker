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

    robot robot;

    if (!robot
            .logo()
            .deserialize(node)
            .init())
    {
        return 1;
    }

    ros::Rate rate(4);

    arm* arm1 = robot.getController<arm>("arm1");

    while(node.ok())
    {
        robot.publish();

        if (arm1) arm1->rotate(-0.25);

        rate.sleep();
    }

    return 0;
}