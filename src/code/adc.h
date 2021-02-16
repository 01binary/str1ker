/*
                                                                                     @@@@@@@                  
 @@@@@@@@@@@@  @@@@@@@@@@@@   @@@@@@@@@@@@       @  @@@@@@@@@@@@@  @           @  @@@       @@@  @@@@@@@@@@@@ 
@              @ @           @            @    @ @  @              @        @@@      @@@@@@@    @            @
 @@@@@@@@@@@@  @   @         @@@@@@@@@@@@@   @   @   @             @   @@@@@      @@@       @@@ @@@@@@@@@@@@@ 
             @ @     @       @            @      @    @@           @@@@      @                  @            @
 @@@@@@@@@@@@  @       @     @            @      @      @@@@@@@@@  @          @   @@@       @@@ @            @
                                                                                     @@@@@@@                  
 adc.h

 Analog to Digital Converter using MCP3008
 Created 1/27/2021

 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <string>
#include "controller.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| adc class
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

class adc : public controller
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
    adc(const char* path);

public:
    // Get display type
    virtual const char* getType();

    // Initialize
    virtual bool init();

    // Get value on channel 0-8
    int getValue(int channel);

    // Get max possible value
    int getMaxValue();

    // Load controller settings
    virtual void deserialize();

public:
    // Create instance
    static controller* create(const char* path);
};

} // namespace str1ker
