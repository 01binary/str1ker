/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 robot.cpp

 Robot Controller Implementation
 Created 11/28/2020

 Copyright (C) 2021 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <vector>
#include <set>
#include <ros/ros.h>
#include "robot.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace str1ker;
using namespace std;

/*----------------------------------------------------------*\
| robot implementation
\*----------------------------------------------------------*/

robot::robot(ros::NodeHandle node):
    m_node(node),
    m_rate(1.0)
{
}

ros::NodeHandle robot::getNode()
{
    return m_node;
}

bool robot::init()
{
    return true;
}

bool robot::update()
{
    return true;
}

robot& robot::run()
{
    ros::Rate rate(m_rate);

    while(m_node.ok())
    {
        update();
        ros::spinOnce();
        rate.sleep();
    }

    return *this;
}

robot& robot::configure(const char* name)
{
    ROS_INFO("loading robot...");

    ros::param::get(string(name) + "/rate", m_rate);

    return *this;
}

robot& robot::logo()
{
    puts("                                                                                     ███████                  ");
    puts(" ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ ");
    puts("█              █ █           █            █    █ █  █              █        ███      ███████    █            █");
    puts(" ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ ");
    puts("             █ █     █       █            █      █    █            ████      █                  █            █");
    puts(" ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █");
    puts("                                                                                     ███████                  ");

    return *this;
}

/*----------------------------------------------------------*\
| Module entry point
\*----------------------------------------------------------*/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot");

    ros::NodeHandle node;
    robot robot(node);

    if (!robot
        .logo()
        .configure("robot")
        .init())
    {
        return 1;
    }

    robot.run();

    return 0;
}
