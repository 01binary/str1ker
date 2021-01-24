/*
                                                                                     @@@@@@@                  
 @@@@@@@@@@@@  @@@@@@@@@@@@   @@@@@@@@@@@@       @  @@@@@@@@@@@@@  @           @  @@@       @@@  @@@@@@@@@@@@ 
@              @ @           @            @    @ @  @              @        @@@      @@@@@@@    @            @
 @@@@@@@@@@@@  @   @         @@@@@@@@@@@@@   @   @   @             @   @@@@@      @@@       @@@ @@@@@@@@@@@@@ 
             @ @     @       @            @      @    @@           @@@@      @                  @            @
 @@@@@@@@@@@@  @       @     @            @      @      @@@@@@@@@  @          @   @@@       @@@ @            @
                                                                                     @@@@@@@                  
 robot.cpp

 Robot controller implementation
 Created 11/28/2020

 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <vector>
#include <set>
#include <pigpio.h>
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

robot::robot()
{
}

robot::robot(armMap arms) : m_arms(arms)
{
}

robot::~robot()
{
    for(armMap::iterator pos = m_arms.begin();
        pos != m_arms.end();
        ++pos)
    {
        delete pos->second;
    }
}

arm* robot::getArm(const char* name)
{
    return m_arms[name];
}

bool robot::init()
{
    ROS_INFO("initializing controllers...");

    if (gpioInitialise() < 0)
    {
        ROS_ERROR("failed to initialize GPIO");
        return false;
    }

    for (armMap::iterator pos = m_arms.begin();
        pos != m_arms.end();
        pos++)
    {
        if (!pos->second->init())
            return false;
    }

    ROS_INFO("initialization completed");

    return true;
}

robot& robot::deserialize()
{
    ROS_INFO("loading actuators...");

    deserializeArms();

    ROS_INFO("loaded successfully");

    return *this;
}

void robot::deserializeArms()
{
    vector<string> params;
    set<string> unique;
    char armPath[255] = {0};

    ros::param::getParamNames(params);

    for (vector<string>::iterator pos = params.begin();
        pos != params.end();
        pos++)
    {
        const char* path = getComponentPath(pos->c_str(), "arms", armPath);

        if (path && unique.find(path) == unique.end())
        {
            unique.insert(armPath);

            arm* robotArm = new arm(path);
            robotArm->deserialize();

            m_arms[getComponentName(path)] = robotArm;
        }
    }
}

const char* robot::getComponentName(const char* path)
{
    const char* lastSep = strrchr(path, '/');

    if (lastSep == NULL) return path;

    return lastSep + 1;
}

const char* robot::getComponentPath(const char* path, const char* componentType, char* componentPath)
{
    char componentTypePath[64] = {0};
    sprintf(componentTypePath, "/%s/", componentType);

    const char* typeNode = strstr(path, componentTypePath);
    if (typeNode == NULL) return NULL;

    const char* nameNode = typeNode + strlen(componentTypePath);
    const char* nextNode = strchr(nameNode , '/');

    if (nextNode == NULL) nextNode = nameNode + strlen(nameNode);

    strncpy(componentPath, path, nextNode - path);
    return componentPath;
}

robot& robot::logo()
{
    puts("                                                                                     @@@@@@@                  ");
    puts(" @@@@@@@@@@@@  @@@@@@@@@@@@   @@@@@@@@@@@@       @  @@@@@@@@@@@@@  @           @  @@@       @@@  @@@@@@@@@@@@ ");
    puts("@              @ @           @            @    @ @  @              @        @@@      @@@@@@@    @            @");
    puts(" @@@@@@@@@@@@  @   @         @@@@@@@@@@@@@   @   @   @             @   @@@@@      @@@       @@@ @@@@@@@@@@@@@ ");
    puts("             @ @     @       @            @      @    @@           @@@@      @                  @            @");
    puts(" @@@@@@@@@@@@  @       @     @            @      @      @@@@@@@@@  @          @   @@@       @@@ @            @");
    puts("                                                                                     @@@@@@@                  ");

    return *this;
}