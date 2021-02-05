/*
                                                                                     @@@@@@@                  
 @@@@@@@@@@@@  @@@@@@@@@@@@   @@@@@@@@@@@@       @  @@@@@@@@@@@@@  @           @  @@@       @@@  @@@@@@@@@@@@ 
@              @ @           @            @    @ @  @              @        @@@      @@@@@@@    @            @
 @@@@@@@@@@@@  @   @         @@@@@@@@@@@@@   @   @   @             @   @@@@@      @@@       @@@ @@@@@@@@@@@@@ 
             @ @     @       @            @      @    @@           @@@@      @                  @            @
 @@@@@@@@@@@@  @       @     @            @      @      @@@@@@@@@  @          @   @@@       @@@ @            @
                                                                                     @@@@@@@                  
 controllerFactory.cpp

 Controller factory implementation
 Created 1/21/2021

 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros/ros.h>
#include "controllerFactory.h"
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
| controllerFactory implementation
\*----------------------------------------------------------*/

map<string, createController> controllerFactory::s_types;

controller* controllerFactory::deserialize(const char* parentPath, const char* controllerName)
{
    string componentPath = string(parentPath) + "/" + controllerName;
    string controllerTypePath = componentPath + "/controller";
    string controllerType;

    ros::param::get(controllerTypePath, controllerType);

    if (s_types.find(controllerType.c_str()) == s_types.end())
        return NULL;

    controller* created = s_types[controllerType](componentPath.c_str());
    created->deserialize();

    return created;
}

void controllerFactory::registerType(const char* type, createController create)
{
    s_types[type] = create;
}
