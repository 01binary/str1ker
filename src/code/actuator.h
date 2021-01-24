/*
                                                                                     @@@@@@@                  
 @@@@@@@@@@@@  @@@@@@@@@@@@   @@@@@@@@@@@@       @  @@@@@@@@@@@@@  @           @  @@@       @@@  @@@@@@@@@@@@ 
@              @ @           @            @    @ @  @              @        @@@      @@@@@@@    @            @
 @@@@@@@@@@@@  @   @         @@@@@@@@@@@@@   @   @   @             @   @@@@@      @@@       @@@ @@@@@@@@@@@@@ 
             @ @     @       @            @      @    @@           @@@@      @                  @            @
 @@@@@@@@@@@@  @       @     @            @      @      @@@@@@@@@  @          @   @@@       @@@ @            @
                                                                                     @@@@@@@                  
 arm.h

 Actuator base class
 Created 1/19/2021

 This software is licensed under GNU GPLv3
*/

#ifndef STR1KER_ACTUATOR_H
#define STR1KER_ACTUATOR_H

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <string>

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| actuator base class
\*----------------------------------------------------------*/

class actuator
{
protected:
    // Actuator display name
    std::string m_name;

    // Actuator control path
    std::string m_path;

    // Whether actuator is enabled
    bool m_enable;

public:
    actuator(const char* path);

public:
    // Get actuator display name
    const char* getName();

    // Get actuator control path
    const char* getPath();

    // Get enabled status
    const bool isEnabled();

    // Get actuator type display name
    virtual const char* getType() = 0;

    // Deserialize from settings
    virtual void deserialize();

protected:
    // Get child component control path
    std::string getComponentPath(const char* componentName);
};

} // namespace str1ker

#endif // STR1KER_ACTUATOR_H
