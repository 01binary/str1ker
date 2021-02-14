/*
                                                                                     @@@@@@@                  
 @@@@@@@@@@@@  @@@@@@@@@@@@   @@@@@@@@@@@@       @  @@@@@@@@@@@@@  @           @  @@@       @@@  @@@@@@@@@@@@ 
@              @ @           @            @    @ @  @              @        @@@      @@@@@@@    @            @
 @@@@@@@@@@@@  @   @         @@@@@@@@@@@@@   @   @   @             @   @@@@@      @@@       @@@ @@@@@@@@@@@@@ 
             @ @     @       @            @      @    @@           @@@@      @                  @            @
 @@@@@@@@@@@@  @       @     @            @      @      @@@@@@@@@  @          @   @@@       @@@ @            @
                                                                                     @@@@@@@                  
 arm.h

 Robot arm controller
 Created 1/19/2021

 This software is licensed under GNU GPLv3
*/

#ifndef STR1KER_ARM_H
#define STR1KER_ARM_H

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <string>
#include "servo.h"
#include "linear.h"
#include "solenoid.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| arm class
\*----------------------------------------------------------*/

class arm
{
private:
    // Robot arm path
    std::string m_path;

    // Robot arm name
    std::string m_name;

    // Rotation servo
    servo* m_shoulder;

    // Lift servo
    linear* m_upperarm;

    // Extend servo
    linear* m_forearm;

    // Trigger solenoid
    solenoid* m_trigger;

public:
    arm(const char* path, servo* shoulder, linear* upperarm, linear* forearm, solenoid* trigger);
    arm(const char* path);
    ~arm();

public:
    // Initialize arm controllers
    bool init();

    // Rotate arm by delta in possible range
    void rotate(double delta);

    // Raise arm all the way
    void raise(double amount = 1.0);

    // Lower arm all the way
    void lower(double amount = 1.0);

    // Extend arm all the way
    void extend(double amount = 1.0);

    // Contract arm all the way
    void contract(double amount = 1.0);

    // Trigger arm
    void trigger(double durationSeconds = 0.023);

    // Load arm controller settings
    void deserialize();
};

} // namespace str1ker

#endif // STR1KER_ARM_H
