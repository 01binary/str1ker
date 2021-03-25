/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 pi16adc.cpp

 PI16ADC Analog to Digital Converter
 Created 03/20/2021

 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros/ros.h>
#include "pi16adc.h"
#include "controllerFactory.h"
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace str1ker;
using namespace std;

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const char pi16adc::TYPE[] = "pi16adc";

/*
    If the first three bits shifted into the device are 101 ("preamble"), then
    the next five bits select the input channel for the next conversion cycle.

    The first input bit (SGL) following the 101 sequence determines
    if the input selection is differential (SGL = 0) or single-ended (SGL = 1)
*/

const uint8_t pi16adc::CHANNEL_COMMANDS[] =
{           // PREAMBLE SGL CHANNEL
    0xB0,   // 101      1   0000
    0xB8,   // 101      1   1000
    0xB1,   // 101      1   0001
    0xB9,   // 101      1   1001
    0xB2,   // 101      1   0010
    0xBA,   // 101      1   1010
    0xB3,   // 101      1   0011
    0xBB,   // 101      1   1011
    0xB4,   // 101      1   0100
    0xBC,   // 101      1   1100
    0xB5,   // 101      1   0101
    0xBD,   // 101      1   1101
    0xB6,   // 101      1   0110
    0xBE,   // 101      1   1110
    0xB7,   // 101      1   0111
    0xBF    // 101      1   1111
};

/*----------------------------------------------------------*\
| pi16adc implementation
\*----------------------------------------------------------*/

REGISTER_SINGLETON(pi16adc)

pi16adc::pi16adc(const char* path) :
    adc(path),
    m_initialized(false),
    m_i2cBus(-1),
    m_i2cHandle(0),
    m_i2cAddress(DEFAULT_ADDRESS),
    m_delayUs(DEFAULT_DELAY_US),
    m_channels(1)
{
    memset(m_samples, 0, sizeof(m_samples));
}

const char* pi16adc::getType()
{
    return pi16adc::TYPE;
}

int pi16adc::getValue(int channel)
{
    if (!m_enable || !m_initialized || channel >= m_channels) return 0;
    return m_samples[channel];
}

int pi16adc::getMaxValue()
{
    return MAX_VALUE;
}

int pi16adc::getChannels()
{
    return m_channels;
}

bool pi16adc::init()
{
    if (!m_enable || m_initialized) return true;

    char busName[16] = {0};
    sprintf(busName, "/dev/i2c-%d", m_i2cBus);

    m_i2cHandle = open(busName, O_RDWR);
    
    if (m_i2cHandle < 0)
    {
        ROS_ERROR("  failed to initialize %s: could not open %s", getName(), busName);
        return false;
    }

    if (ioctl(m_i2cHandle, I2C_SLAVE, m_i2cAddress) == -1)
    {
        ROS_ERROR("  failed to initialize %s: could not select i2c address 0x%x", getName(), m_i2cAddress);
        return false;
    }

    if (!configure(RESET_COMMAND))
    {
        ROS_ERROR("  failed to initialize %s: could not write reset command", getName());
        return false;
    }

    ROS_INFO("  initialized %s on i2c bus %d address 0x%x", getPath(), m_i2cBus, m_i2cAddress);

    m_initialized = true;

    return true;
}

bool pi16adc::configure(uint8_t config)
{
    int result = 0;

    for (int attempt = 0; attempt < MAX_RETRY; attempt++)
    {
        result = write(m_i2cHandle, &config, 1);
        usleep(m_delayUs);

        if (result == 1) break;
    }

    if (result != 1)
    {
        ROS_ERROR("  failed to configure %s with 0x%x", getName(), config);
        return false;
    }

    return true;
}

void pi16adc::publish()
{
    if (!m_enable || !m_initialized) return;

    unsigned char buffer[VALUE_SIZE] = {0};

    for (int n = 0; n < m_channels; n++)
    {
        // Switch channel and wait
        configure(CHANNEL_COMMANDS[n]);

        // Read conversion
        if (read(m_i2cHandle, buffer, VALUE_SIZE) != VALUE_SIZE)
        {
            ROS_ERROR("  failed to read %s channel %d: chip may not respond faster than .15s", getName(), n);
            return;
        }

        // Validate conversion
        if ((buffer[0] & 0b11000000) == 0b11000000)
        {
            ROS_WARN("  %s channel %d voltage open or > 2.5V", getName(), n);
            m_samples[n] = MAX_VALUE;
        }
        else
        {
            // Decode conversion
            m_samples[n] = ((buffer[0] & 0x3F) << 16) + (buffer[1] << 8) + (buffer[2] & 0xE0);
        }
    }

    ROS_INFO("[%.2g][%.2g]",
        double(m_samples[0]) / double(MAX_VALUE),
        double(m_samples[1]) / double(MAX_VALUE));
}

void pi16adc::deserialize(ros::NodeHandle node)
{
    controller::deserialize(node);

    ros::param::get(getControllerPath("i2c"), m_i2cBus);
    ros::param::get(getControllerPath("address"), m_i2cAddress);
    ros::param::get(getControllerPath("channels"), m_channels);

    double delay;
    if (ros::param::get(getControllerPath("delay"), delay))
    {
        // Read delay in fractions of a second and convert to microseconds.
        m_delayUs = int(delay * 1000000.0);
    }
}

controller* pi16adc::create(const char* path)
{
    return new pi16adc(path);
}
