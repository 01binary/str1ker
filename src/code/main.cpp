/*
                                                                                     @@@@@@@                  
 @@@@@@@@@@@@  @@@@@@@@@@@@   @@@@@@@@@@@@       @  @@@@@@@@@@@@@  @           @  @@@       @@@  @@@@@@@@@@@@ 
@              @ @           @            @    @ @  @              @        @@@      @@@@@@@    @            @
 @@@@@@@@@@@@  @   @         @@@@@@@@@@@@@   @   @   @             @   @@@@@      @@@       @@@ @@@@@@@@@@@@@ 
             @ @     @       @            @      @    @@           @@@@      @                  @            @
 @@@@@@@@@@@@  @       @     @            @      @      @@@@@@@@@  @          @   @@@       @@@ @            @
                                                                                     @@@@@@@                  
 main.cpp

 Str1ker drumming robot module
 Created 01/20/2021

 This software is licensed under GNU GPLv3
*/

#define DEBUG

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros/ros.h>
#include "robot.h"

/*----------------------------------------------------------*\
| Module
\*----------------------------------------------------------*/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot");
    ros::NodeHandle node;

    str1ker::robot robot;

    if (!robot
            .logo()
            .deserialize(node)
            .init())
    {
        return 1;
    }

    ros::Rate rate(1000);

    while(node.ok())
    {
        robot.publish();

        robot.getArm(0)->rotate(-0.25);

        rate.sleep();
    }

    return 0;
}