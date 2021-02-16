/*
                                                                                     @@@@@@@                  
 @@@@@@@@@@@@  @@@@@@@@@@@@   @@@@@@@@@@@@       @  @@@@@@@@@@@@@  @           @  @@@       @@@  @@@@@@@@@@@@ 
@              @ @           @            @    @ @  @              @        @@@      @@@@@@@    @            @
 @@@@@@@@@@@@  @   @         @@@@@@@@@@@@@   @   @   @             @   @@@@@      @@@       @@@ @@@@@@@@@@@@@ 
             @ @     @       @            @      @    @@           @@@@      @                  @            @
 @@@@@@@@@@@@  @       @     @            @      @      @@@@@@@@@  @          @   @@@       @@@ @            @
                                                                                     @@@@@@@                  
 adc.cpp

 Analog to Digital Converter using MCP3008 implementation
 Created 1/27/2021

 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros/ros.h>
#include <pigpio.h>
#include "adc.h"
#include "controllerFactory.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace str1ker;
using namespace std;

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const char adc::TYPE[] = "adc";

/*----------------------------------------------------------*\
| adc implementation
\*----------------------------------------------------------*/

REGISTER_SINGLETON(adc)

adc::adc(const char* path) :
    controller(path),
    m_spiBus(-1),
    m_spi(-1)
{
}

const char* adc::getType()
{
    return adc::TYPE;
}

bool adc::init()
{
    if (!m_enable || m_spi >= 0) return true;

    m_spi = spiOpen(m_spiBus, SPI_RATE, SPI_MODE);

    if (m_spi < 0)
    {
        ROS_ERROR("failed to initialize ADC on SPI %d: 0x%x", m_spiBus, m_spi);
        return false;
    }

    ROS_INFO("  initialized %s %s on SPI %d", getPath(), getType(), m_spiBus);

    return true;
}

int adc::getValue(int channel)
{
    if (!m_enable) return 0;

    char buffer[3] = { 1, char((8 + channel) << 4), 0 };
    spiXfer(m_spi, buffer, buffer, sizeof(buffer));

    return ((buffer[1] & 3) << 8) | buffer[2];
}

int adc::getMaxValue()
{
    // MCP3008 is 10-bit (0 to 1024)
    return 1 << 10;
}

void adc::deserialize(ros::NodeHandle node)
{
    controller::deserialize(node);
    ros::param::get(getControllerPath("spi"), m_spiBus);
}

controller* adc::create(const char* path)
{
    return new adc(path);
}
