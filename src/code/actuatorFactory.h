/*
                                                                                     @@@@@@@                  
 @@@@@@@@@@@@  @@@@@@@@@@@@   @@@@@@@@@@@@       @  @@@@@@@@@@@@@  @           @  @@@       @@@  @@@@@@@@@@@@ 
@              @ @           @            @    @ @  @              @        @@@      @@@@@@@    @            @
 @@@@@@@@@@@@  @   @         @@@@@@@@@@@@@   @   @   @             @   @@@@@      @@@       @@@ @@@@@@@@@@@@@ 
             @ @     @       @            @      @    @@           @@@@      @                  @            @
 @@@@@@@@@@@@  @       @     @            @      @      @@@@@@@@@  @          @   @@@       @@@ @            @
                                                                                     @@@@@@@                  
 arm.h

 Actuator factory class
 Created 1/21/2021

 This software is licensed under GNU GPLv3
*/

#ifndef STR1KER_ACTUATOR_FACTORY_H
#define STR1KER_ACTUATOR_FACTORY_H

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <string>
#include <map>
#include "actuator.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| Macros
\*----------------------------------------------------------*/

#define REGISTER_ACTUATOR(a) \
    class register_##a { public: register_##a() { actuatorFactory::registerType(a::TYPE, a::create);}}; \
    register_##a reg##a;

/*----------------------------------------------------------*\
| Definitions
\*----------------------------------------------------------*/

typedef actuator* (*createActuator)(const char*);

/*----------------------------------------------------------*\
| actuatorFactory class
\*----------------------------------------------------------*/

class actuatorFactory
{
private:
    // Actuator types
    static std::map<std::string, createActuator> s_types;

public:
    // Deserialize actuator with type
    template<class T> static T* deserialize(
        const char* parentPath,
        const char* componentName)
    {
        return (T*)deserialize(parentPath, componentName);
    }

    // Deserialize actuator
    static actuator* deserialize(const char* parentPath, const char* componentName);

    // Register actuator type
    static void registerType(const char* type, createActuator create);
};

} // namespace str1ker

#endif // STR1KER_ACTUATOR_FACTORY_H
