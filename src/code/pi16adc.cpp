/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 pi16adc.cpp

 Analog to Digital Converter Using Alchemy PI-16ADC
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
#ifndef I2C_M_RD
#include <linux/i2c.h>
#endif

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace str1ker;
using namespace std;

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const char pi16adc::TYPE[] = "pi16adc";

/*----------------------------------------------------------*\
| pi16adc implementation
\*----------------------------------------------------------*/

REGISTER_SINGLETON(pi16adc)

pi16adc::pi16adc(const char* path) :
    adc(path),
    m_initialized(false),
    m_i2cBus(-1),
    m_i2cHandle(0),
    m_i2cAddress(DEFAULT_ADDRESS)
{
    memset(m_samples, 0, sizeof(m_samples));
}

const char* pi16adc::getType()
{
    return pi16adc::TYPE;
}

bool pi16adc::init()
{
    if (!m_enable || m_initialized) return true;

    char deviceName[16] = {0};
    sprintf(deviceName, "/dev/i2c-%d", m_i2cBus);

    m_i2cHandle = open(deviceName, O_RDWR);
    
    if (m_i2cHandle < 0)
    {
        ROS_ERROR("  failed to initialize %s: could not open %s", getName(), deviceName);
        return false;
    }

    ROS_INFO("  initialized %s on i2c bus %d address 0x%x", getPath(), m_i2cBus, m_i2cAddress);

    m_initialized = true;

    return true;
}

bool pi16adc::selectChannel(channels channel)
{
    uint8_t buffer[] = { channel };

    if (write(m_i2cHandle, buffer, 1) != 1)
    {
        ROS_ERROR("  failed to publish %s: could not select channel %d",
            getName(), channel - channels::channel0);
        return false;
    }

    return true;
}

bool pi16adc::readRegister(int reg, uint8_t *value, int size)
{
    uint8_t command[] = { reg };
    i2c_msg msgs[2];
    i2c_rdwr_ioctl_data msgset[1];

    msgs[0].addr = m_i2cAddress;
    msgs[0].flags = 0;
    msgs[0].len = sizeof(command);
    msgs[0].buf = command;

    msgs[1].addr = m_i2cAddress;
    msgs[1].flags = I2C_M_RD | I2C_M_NOSTART;
    msgs[1].len = size;
    msgs[1].buf = value;

    msgset[0].msgs = msgs;
    msgset[0].nmsgs = 2;

    if (ioctl(m_i2cHandle, I2C_RDWR, &msgset) < 0)
    {
        ROS_ERROR("  failed to read %s on i2c bus %d address 0x%x",
            getName(), m_i2cBus, m_i2cAddress);
        return false;
    }

    return true;
}

bool pi16adc::writeRegister(int reg, uint8_t* value, int size)
{
    uint8_t* buffer = new uint8_t[size + 1];

    buffer[0] = reg;
    memcpy(buffer + 1, value, size);

    struct i2c_msg msgs[1];
    struct i2c_rdwr_ioctl_data msgset[1];

    msgs[0].addr = m_i2cAddress;
    msgs[0].flags = 0;
    msgs[0].len = size + 1;
    msgs[0].buf = buffer;

    msgset[0].msgs = msgs;
    msgset[0].nmsgs = 1;

    if (ioctl(m_i2cHandle, I2C_RDWR, &msgset) < 0)
    {
        ROS_ERROR("  failed to write %s on i2c bus %d address 0x%x",
            getName(), m_i2cBus, m_i2cAddress);
        return false;
    }

    return true;
}

int pi16adc::getValue(int channel)
{
    if (!m_enable || !m_initialized || channel >= CHANNELS) return 0;
    return m_samples[channel];
}

int pi16adc::getMaxValue()
{
    return MAX_VALUE;
}

int pi16adc::getChannels()
{
    return CHANNELS;
}

void pi16adc::publish()
{
    if (!m_enable || !m_initialized) return;

    unsigned char buffer[BLOCK_SIZE] = {0};

    for (int n = 0; n < CHANNELS; n++)
    {
        usleep(SLEEP_TIME);

        // Switch channel
        channels channel = (channels)(channels::channel0 + n);
        if (!selectChannel(channel)) continue;

        // Read conversion
        if (!readRegister(channel, buffer, BLOCK_SIZE)) continue;

        // Validate conversion
        if ((buffer[0] & 0b11000000) == 0b11000000)
        {
            ROS_WARN("  %s channel %d voltage open or > 2.5V", getName(), n);
        }

        // Decode conversion
        int sample =
            ((buffer[0] & 0x3F) << 16) +
            (buffer[1] << 8) +
            (buffer[2] & 0xE0);

        m_samples[n] = sample;
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
}

controller* pi16adc::create(const char* path)
{
    return new pi16adc(path);
}
