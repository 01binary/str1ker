/*
                                                                                     @@@@@@@                  
 @@@@@@@@@@@@  @@@@@@@@@@@@   @@@@@@@@@@@@       @  @@@@@@@@@@@@@  @           @  @@@       @@@  @@@@@@@@@@@@ 
@              @ @           @            @    @ @  @              @        @@@      @@@@@@@    @            @
 @@@@@@@@@@@@  @   @         @@@@@@@@@@@@@   @   @   @             @   @@@@@      @@@       @@@ @@@@@@@@@@@@@ 
             @ @     @       @            @      @    @@           @@@@      @                  @            @
 @@@@@@@@@@@@  @       @     @            @      @      @@@@@@@@@  @          @   @@@       @@@ @            @
                                                                                     @@@@@@@                  
 robot.h

 Robot Controller
 Created 1/19/2021

 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <string>
#include <map>
#include "arm.h"
#include "adc.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| Definitions
\*----------------------------------------------------------*/

typedef std::vector<arm*> armArray;
typedef std::map<std::string, arm*> armMap;

/*----------------------------------------------------------*\
| robot class
\*----------------------------------------------------------*/

class robot
{
private:
    static const char PATH[];

    // Robot arms
    armArray m_arms;
    armMap m_armNames;
    adc* m_adc;

public:
    robot();
    ~robot();

public:
    // Initialize controllers
    bool init();

    // Load controller settings
    robot& deserialize();

    // Print logo
    robot& logo();

    // Get robot arm by index
    arm* getArm(int index);

    // Get robot arm by name
    arm* getArm(const char* name);

public:
    // Get component name
    static const char* getControllerName(const char* path);

    // Get parent name
    static const char* getControllerPath(const char* path, const char* componentType, char* componentPath);

private:
    void deserializeArms();
};

} // namespace str1ker
