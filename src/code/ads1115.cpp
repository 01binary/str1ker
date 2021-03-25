/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 ads1115.cpp

 ADS1115 Analog to Digital Converter Implementation
 Created 02/22/2021

 Copyright (C) 2021 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <inttypes.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include "ads1115.h"
#include "controllerFactory.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace str1ker;
using namespace std;

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const char ads1115::TYPE[] = "ads1115";

const uint8_t ads1115::DEVICE_ADDRESS[] =
{
    0x48,                       // Device address when ADDR pin tied to GND
    0x49,                       // Device address when ADDR pin tied to VDD
    0x4A,                       // Device address when ADDR pin tied to SDA
    0x4B                        // Device address when ADDR pin tied to SCL
};

const uint16_t ads1115::SELECT_CHANNEL[] =
{
    multiplexer::single_0,      // Select single-ended channel 0 in multiplexer
    multiplexer::single_1,      // Select single-ended channel 1 in multiplexer
    multiplexer::single_2,      // Select single-ended channel 2 in multiplexer
    multiplexer::single_3       // Select single-ended channel 3 in multiplexer
};

const int ads1115::DELAYS[] =
{
    1000 + 1000 * 1000 / 8,     // Device response delay for 8 samples/sec rate
    1000 + 1000 * 1000 / 16,    // Device response delay for 16 samples/sec rate
    1000 + 1000 * 1000 / 32,    // Device response delay for 32 samples/sec rate
    1000 + 1000 * 1000 / 64,    // Device response delay for 64 samples/sec rate
    1000 + 1000 * 1000 / 128,   // Device response delay for 128 samples/sec rate
    1000 + 1000 * 1000 / 250,   // Device response delay for 250 samples/sec rate
    1000 + 1000 * 1000 / 475,   // Device response delay for 475 samples/sec rate
    1000 + 1000 * 1000 / 860    // Device response delay for 860 samples/sec rate
};

const char* ads1115::GAIN_NAMES[] =
{
    "6.144V",                   // Expect analog voltage between 0 and 6.144 volts when taking readings
    "4.096V",                   // Expect analog voltage between 0 and 4.096 volts when taking readings
    "2.048V",                   // Expect analog voltage between 0 and 2.048 volts when taking readings
    "1.024V",                   // Expect analog voltage between 0 and 1.024 volts when taking readings
    "0.512V",                   // Expect analog voltage between 0 and 0.512 volts when taking readings
    "0.256V",                   // Expect analog voltage between 0 and 0.256 volts when taking readings
    "0.256V2",                  // This value is not used
    "0.256V3"                   // This value is not used
};

const uint16_t ads1115::GAIN_VALUES[] =
{
    gain::pga_6_144V,
    gain::pga_4_096V,
    gain::pga_2_048V,
    gain::pga_1_024V,
    gain::pga_0_512V,
    gain::pga_0_256V,
    gain::pga_0_256V2,
    gain::pga_0_256V3
};

const int ads1115::RATES[] =
{
    8, 16, 32, 64, 128, 250, 475, 860
};

const char* ads1115::RATE_NAMES[] =
{
    "sps8",
    "sps16",
    "sps32",
    "sps64",
    "sps128",
    "sps250",
    "sps475",
    "sps860"
};

const uint16_t ads1115::RATE_VALUES[] =
{
    ads1115::sps8,
    ads1115::sps16,
    ads1115::sps32,
    ads1115::sps64,
    ads1115::sps128,
    ads1115::sps250,
    ads1115::sps475,
    ads1115::sps860
};

/*----------------------------------------------------------*\
| ads1115 implementation
\*----------------------------------------------------------*/

REGISTER_SINGLETON(ads1115)

ads1115::ads1115(const char* path) :
    adc(path),
    m_i2cBus(-1),
    m_devices(1),
    m_lastDevice(-1),
    m_lastDeviceChannel(-1),
    m_gain(gain::pga_2_048V),
    m_rate(sampleRate::sps128),
    m_i2cHandle(-1)
{
    memset(m_lastSample, 0, sizeof(m_lastSample));
}

controller* ads1115::create(const char* path)
{
    return new ads1115(path);
}

const char* ads1115::getType()
{
    return ads1115::TYPE;
}

bool ads1115::init()
{
    if (!m_enable) return true;

    char busName[16] = {0};
    sprintf(busName, "/dev/i2c-%d", m_i2cBus);

    m_i2cHandle = open(busName, O_RDWR);
    
    if (m_i2cHandle < 0)
    {
        ROS_ERROR("  failed to initialize %s: could not open %s", getName(), busName);
        return false;
    }

    return true;
}

bool ads1115::configureDevice(int deviceIndex, int deviceChannelIndex, uint16_t config)
{
    // Validate
    if (m_i2cHandle == -1)
    {
        ROS_WARN("  attempted to configure %s before initializing i2c bus", getName());
        return false;
    }

    // Select device to talk to on I2C bus
    if (ioctl(m_i2cHandle, I2C_SLAVE, DEVICE_ADDRESS[deviceIndex]) < 0)
    {
        ROS_ERROR("  failed to select %s at 0x%x on i2c %d", getName(), DEVICE_ADDRESS[deviceIndex], m_i2cBus);
        return false;
    }

    // Switch device channel as part of configuration
    config |= SELECT_CHANNEL[deviceChannelIndex];

    // Configure device
    uint8_t command[] = {
        // Select configuration register
        deviceRegister::configuration,
        // Configuration register value MSB
        ((uint8_t*)&config)[1],
        // Configuration register value LSB
        ((uint8_t*)&config)[0]
    };

    if (write(m_i2cHandle, command, sizeof(command)) != sizeof(command))
    {
        ROS_ERROR("  failed to configure %s at 0x%x on i2c %d", getName(), DEVICE_ADDRESS[deviceIndex], m_i2cBus);
        return false;
    }

    m_lastDevice = deviceIndex;
    m_lastDeviceChannel = deviceChannelIndex;

    return true;
}

bool ads1115::pollDevice()
{
    uint8_t buffer[2] = {0};
    uint16_t config = 0;
    int retries = 0;

    // Validate
    if (m_i2cHandle == -1)
    {
        ROS_WARN("  attempted to poll %s before initializing i2c bus", getName());
        return false;
    }

    if (m_lastDevice == -1 || m_lastDeviceChannel == -1)
    {
        ROS_WARN("  attempted to poll %s before selecting and configuring device", getName());
        return false;
    }

    do
    {
        // Wait for configuration change to take effect
        usleep(DELAYS[m_rate >> 5]);

        // Read configuration register selected when configuring device
        if (read(m_i2cHandle, &buffer, sizeof(buffer)) != sizeof(buffer))
        {
            ROS_ERROR("  failed to poll %s channel %d at 0x%x on i2c %d",
                getName(), m_lastDeviceChannel, DEVICE_ADDRESS[m_lastDevice], m_i2cBus);
            return false;
        }

        // Decode configuration register value from two's complement
        config = buffer[0] << 8 | buffer[1];
    }
    while
    (
        // Wait until conversion is ready for the expected channel
        config & readOperation::ready == 0 &&
        ((config >> 12) & 0b111) != (SELECT_CHANNEL[m_lastDeviceChannel] >> 12) &&
        retries++ < MAX_RETRY
    );

    return retries < MAX_RETRY;
}

bool ads1115::readConversion(uint16_t& value)
{
    // Validate
    if (m_i2cHandle == -1)
    {
        ROS_WARN("  attempted to read %s conversion before initializing i2c bus", getName());
        return false;
    }

    if (m_lastDevice == -1 || m_lastDeviceChannel == -1)
    {
        ROS_WARN("  attempted to read %s conversion before selecting and configuring", getName());
        return false;
    }

    // Select conversion register
    uint8_t command[] = { deviceRegister::conversion };

    if (write(m_i2cHandle, command, sizeof(command)) != sizeof(command))
    {
        ROS_ERROR("  failed to select conversion register for %s channel %d at 0x%x on i2c %d",
            getName(), m_lastDeviceChannel, DEVICE_ADDRESS[m_lastDevice], m_i2cBus);
        return false;
    }

    // Read conversion register
    uint8_t conversion[2] = {0};

    if (read(m_i2cHandle, &conversion, sizeof(conversion)) != sizeof(conversion))
    {
        ROS_ERROR("  failed to read conversion for %s channel %d at 0x%x on i2c %d",
            getName(), m_lastDeviceChannel, DEVICE_ADDRESS[m_lastDevice], m_i2cBus);
        return false;
    }

    // Decode conversion from two's complement
    value = conversion[0] << 8 | conversion[1];

    return true;
}

void ads1115::publish()
{
    if (!m_enable) return;

    uint16_t value = 0;
    uint16_t config =
        // Trigger conversion
        writeOperation::convert |
        // Set gain
        m_gain |
        // Set single-shot conversion mode
        sampleMode::single |
        // Set sampling rate
        m_rate |
        // Not using comparator
        comparatorMode::traditional |
        // Not using latching
        latchMode::nonLatching |
        // Not using alerts
        alertMode::none |
        alertPolarity::activeLow;

    for (int deviceIndex = 0; deviceIndex < m_devices; deviceIndex++)
    {
        for (int deviceChannelIndex = 0;
            deviceChannelIndex < DEVICE_CHANNELS;
            deviceChannelIndex++)
        {
            if (configureDevice(deviceIndex, deviceChannelIndex, config) &&
                pollDevice() &&
                readConversion(value))
            {
                m_lastSample[deviceIndex * DEVICE_CHANNELS + deviceChannelIndex] = value;
            }
        }
    }
}

int ads1115::getValue(int channel)
{
    if (!m_enable || channel < 0 || channel >= m_devices * DEVICE_CHANNELS) return 0;
    return m_lastSample[channel];
}

int ads1115::getMaxValue()
{
    switch (m_gain)
    {
    case gain::pga_6_144V:
        return 0x44FF;
    case gain::pga_4_096V:
        return 0x66FF;
    default:
        return 0x7FFF;
    }
}

int ads1115::getChannels()
{
    return m_devices * DEVICE_CHANNELS;
}

void ads1115::deserialize(ros::NodeHandle node)
{
    controller::deserialize(node);

    ros::param::get(getControllerPath("i2c"), m_i2cBus);
    ros::param::get(getControllerPath("devices"), m_devices);

    string gainSetting;
    if (ros::param::get(getControllerPath("gain"), gainSetting))
    {
        for (int n = 0; n < sizeof(GAIN_VALUES) / sizeof(uint16_t); n++)
        {
            if (strcmp(gainSetting.c_str(), GAIN_NAMES[n]) == 0)
            {
                m_gain = (gain)GAIN_VALUES[n];
                break;
            }
        }
    }

    int rateSetting;
    if (ros::param::get(getControllerPath("rate"), rateSetting))
    {
        for (int n = 0; n <= sizeof(RATES) / sizeof(int); n++)
        {
            if (rateSetting == RATES[n])
            {
                m_rate = (sampleRate)RATE_VALUES[n];
                break;
            }
        }
    }
}
