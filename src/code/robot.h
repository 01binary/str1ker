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

#ifndef STR1KER_ROBOT_H
#define STR1KER_ROBOT_H

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <string>
#include <map>
#include "servo.h"
#include "arm.h"

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
    // Robot arms
    armArray m_arms;
    armMap m_armNames;

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

#endif // STR1KER_ROBOT_H