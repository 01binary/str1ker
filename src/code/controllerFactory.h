/*
                                                                                     @@@@@@@                  
 @@@@@@@@@@@@  @@@@@@@@@@@@   @@@@@@@@@@@@       @  @@@@@@@@@@@@@  @           @  @@@       @@@  @@@@@@@@@@@@ 
@              @ @           @            @    @ @  @              @        @@@      @@@@@@@    @            @
 @@@@@@@@@@@@  @   @         @@@@@@@@@@@@@   @   @   @             @   @@@@@      @@@       @@@ @@@@@@@@@@@@@ 
             @ @     @       @            @      @    @@           @@@@      @                  @            @
 @@@@@@@@@@@@  @       @     @            @      @      @@@@@@@@@  @          @   @@@       @@@ @            @
                                                                                     @@@@@@@                  
 controllerFactory.h

 Controller Factory class
 Created 1/21/2021

 This software is licensed under GNU GPLv3
*/

#ifndef STR1KER_CONTROLLER_FACTORY_H
#define STR1KER_CONTROLLER_FACTORY_H

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <string>
#include <map>
#include "controller.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| Macros
\*----------------------------------------------------------*/

#define REGISTER_CONTROLLER(a) \
    class register_##a { public: register_##a() { controllerFactory::registerType(a::TYPE, a::create);}}; \
    register_##a reg##a;

/*----------------------------------------------------------*\
| Definitions
\*----------------------------------------------------------*/

typedef controller* (*createController)(const char*);

/*----------------------------------------------------------*\
| controllerFactory class
\*----------------------------------------------------------*/

class controllerFactory
{
private:
    // Controller types
    static std::map<std::string, createController> s_types;

public:
    // Deserialize controller with type
    template<class T> static T* deserialize(
        const char* parentPath,
        const char* controllerName)
    {
        return (T*)deserialize(parentPath, controllerName);
    }

    // Deserialize controller
    static controller* deserialize(const char* parentPath, const char* controllerName);

    // Register controller type
    static void registerType(const char* type, createController create);
};

} // namespace str1ker

#endif // STR1KER_CONTROLLER_FACTORY_H
