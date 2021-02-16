/*
                                                                                     @@@@@@@                  
 @@@@@@@@@@@@  @@@@@@@@@@@@   @@@@@@@@@@@@       @  @@@@@@@@@@@@@  @           @  @@@       @@@  @@@@@@@@@@@@ 
@              @ @           @            @    @ @  @              @        @@@      @@@@@@@    @            @
 @@@@@@@@@@@@  @   @         @@@@@@@@@@@@@   @   @   @             @   @@@@@      @@@       @@@ @@@@@@@@@@@@@ 
             @ @     @       @            @      @    @@           @@@@      @                  @            @
 @@@@@@@@@@@@  @       @     @            @      @      @@@@@@@@@  @          @   @@@       @@@ @            @
                                                                                     @@@@@@@                  
 arm.h

 Controller base class
 Created 1/19/2021

 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <string>

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| controller base class
\*----------------------------------------------------------*/

class controller
{
protected:
    // Controller display name
    std::string m_name;

    // Controller control path
    std::string m_path;

    // Whether controller is enabled
    bool m_enable;

public:
    controller(const char* path);

public:
    // Get controller display name
    const char* getName();

    // Get controller control path
    const char* getPath();

    // Get enabled status
    const bool isEnabled();

    // Get controller type display name
    virtual const char* getType() = 0;

    // Deserialize from settings
    virtual void deserialize();

protected:
    // Get child controller path
    std::string getControllerPath(const char* controllerName);
};

} // namespace str1ker
