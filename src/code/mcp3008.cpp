/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 mcp3008.cpp

 MCP3008 Analog to Digital Converter
 Created 02/22/2021

 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros/ros.h>
#include <pigpio.h>
#include "mcp3008.h"
#include "controllerFactory.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace str1ker;
using namespace std;

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const char mcp3008::TYPE[] = "mcp3008";

/*----------------------------------------------------------*\
| mcp3008 implementation
\*----------------------------------------------------------*/

REGISTER_SINGLETON(mcp3008)

mcp3008::mcp3008(const char* path) :
    adc(path),
    m_spiBus(-1),
    m_spi(-1)
{
}

const char* mcp3008::getType()
{
    return mcp3008::TYPE;
}

bool mcp3008::init()
{
    if (!m_enable || m_spi >= 0) return true;

    m_spi = spiOpen(m_spiBus, SPI_RATE, SPI_MODE);

    if (m_spi < 0)
    {
        ROS_ERROR("failed to initialize MCP3008 ADC on SPI %d: 0x%x", m_spiBus, m_spi);
        return false;
    }

    ROS_INFO("  initialized %s %s on SPI %d", getPath(), getType(), m_spiBus);

    return true;
}

int mcp3008::getValue(int channel)
{
    if (!m_enable) return 0;

    char buffer[3] = { 1, char((8 + channel) << 4), 0 };
    spiXfer(m_spi, buffer, buffer, sizeof(buffer));

    return ((buffer[1] & 3) << 8) | buffer[2];
}

int mcp3008::getMaxValue()
{
    // MCP3008 is 10-bit
    return 1 << 10;
}

int mcp3008::getChannels()
{
    // MCP3008 has 8 channels
    return 8;
}

void mcp3008::deserialize(ros::NodeHandle node)
{
    controller::deserialize(node);
    ros::param::get(getControllerPath("spi"), m_spiBus);
}

controller* mcp3008::create(const char* path)
{
    return new mcp3008(path);
}
