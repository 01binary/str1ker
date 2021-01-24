/*
                                                                                     @@@@@@@                  
 @@@@@@@@@@@@  @@@@@@@@@@@@   @@@@@@@@@@@@       @  @@@@@@@@@@@@@  @           @  @@@       @@@  @@@@@@@@@@@@ 
@              @ @           @            @    @ @  @              @        @@@      @@@@@@@    @            @
 @@@@@@@@@@@@  @   @         @@@@@@@@@@@@@   @   @   @             @   @@@@@      @@@       @@@ @@@@@@@@@@@@@ 
             @ @     @       @            @      @    @@           @@@@      @                  @            @
 @@@@@@@@@@@@  @       @     @            @      @      @@@@@@@@@  @          @   @@@       @@@ @            @
                                                                                     @@@@@@@                  
 actuatorFactory.cpp

 Actuator factory implementation
 Created 1/21/2021

 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros/ros.h>
#include "actuatorFactory.h"
#include "solenoid.h"
#include "pwmServo.h"
#include "dynamixelServo.h"
#include "linear.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace str1ker;
using namespace std;

/*----------------------------------------------------------*\
| actuatorFactory implementation
\*----------------------------------------------------------*/

map<string, createActuator> actuatorFactory::s_types;

actuator* actuatorFactory::deserialize(const char* parentPath, const char* componentName)
{
    string componentPath = string(parentPath) + "/" + componentName;
    string actuatorTypePath = componentPath + "/actuator";
    string actuatorType;

    ros::param::get(actuatorTypePath, actuatorType);

    if (s_types.find(actuatorType.c_str()) == s_types.end())
        return NULL;

    actuator* created = s_types[actuatorType](componentPath.c_str());
    created->deserialize();

    return created;
}

void actuatorFactory::registerType(const char* type, createActuator create)
{
    s_types[type] = create;
}
