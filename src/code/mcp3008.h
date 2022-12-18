/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 mcp3008.h

 MCP3008 Analog to Digital Converter
 Created 02/22/2021

 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <string>
#include "adc.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| mcp3008 class
\*----------------------------------------------------------*/

/*

 Wiring MCP3008 on Raspberri PI 4B SPI0:

 Vdd  - 3V3
 Vref - 3V3
 Agnd - GND
 CLK  - CLK
 Dout - MISO
 Din  - MOSI
 CS   - CS
 Dgnd - GND

*/

class mcp3008 : public adc
{
public:
    // Controller type
    static const char TYPE[];

private:
    // SPI rate for MCP3008 (100 kbps)
    const int SPI_RATE = 100000;

    // SPI mode for MCP3008
    const int SPI_MODE = 0;

     // SPI bus ID where MCP3008 is attached
    int m_spiBus;

    // SPI bus handle
    int m_spi;

public:
    mcp3008(class robot& robot, const char* path);

public:
    // Get display type
    virtual const char* getType();

    // Initialize
    virtual bool init();

    // Get value on channel
    virtual int getValue(int channel);

    // Get number of channels
    virtual int getChannels();

    // Get max possible value
    virtual int getMaxValue();

    // Load controller settings
    virtual void deserialize(ros::NodeHandle node);

public:
    // Create instance
    static controller* create(class robot& robot, const char* path);
};

} // namespace str1ker
