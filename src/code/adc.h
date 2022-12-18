/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 adc.h

 Analog to Digital Converter Controller Base Class
 Created 1/27/2021

 Copyright (C) 2021 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include "controller.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| adc class
\*----------------------------------------------------------*/

class adc : public controller
{
public:
    adc(class robot& robot, const char* path): controller(robot, path) {}

public:
    // Get value on channel
    virtual int getValue(int channel = 0) = 0;

    // Get number of channels
    virtual int getChannels() = 0;

    // Get max possible value
    virtual int getMaxValue() = 0;
};

} // namespace str1ker
