/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 pi16adc.h

 Analog to Digital Converter Using Alchemy PI-16ADC
 Created 03/20/2021

 This software is licensed under GNU GPLv3
*/

/*
    Default I2C bus on Raspberry Pi 4: 1
    Default Address: 0x76

    A2    A1     A0      Address
    LOW   LOW    LOW     0x14
    LOW   LOW    HIGH    0x16
    LOW   LOW    FLOAT   0x15
    LOW   HIGH   LOW     0x26
    LOW   HIGH   HIGH    0x34
    LOW   HIGH   FLOAT   0x27
    LOW   FLOAT  LOW     0x17
    LOW   FLOAT  HIGH    0x25
    LOW   FLOAT  FLOAT   0x24
    HIGH  LOW    LOW     0x56
    HIGH  LOW    HIGH    0x64
    HIGH  LOW    FLOAT   0x57
    HIGH  HIGH   LOW     0x74
    HIGH  HIGH   HIGH    0x76 default
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
| pi16adc class
\*----------------------------------------------------------*/

class pi16adc : public adc
{
public:
    const int DEFAULT_ADDRESS = 0x76;
    const int MAX_CHANNELS = 16;
    const int MAX_VALUE = 0x800000;
    const int VALUE_SIZE = 3;
    const int SLEEP_TIME_US = 150 * 1000;
    const uint16_t RESET_COMMAND = 0xA0;
    static const uint8_t CHANNEL_COMMANDS[];

public:
    // Controller type
    static const char TYPE[];

private:
     // I2C bus ID
    int m_i2cBus;

    // I2C bus handle
    int m_i2cHandle;

    // I2C address 
    int m_i2cAddress;

    // Number of channels to read
    int m_channels;

    // Last published values for all channels
    int m_samples[16];

    // Whether ADS was initialized
    bool m_initialized;

public:
    pi16adc(const char* path);

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

    // Publish channel values
    virtual void publish();

    // Load controller settings
    virtual void deserialize(ros::NodeHandle node);

private:
    bool configure(uint8_t config);

public:
    // Create instance
    static controller* create(const char* path);
};

} // namespace str1ker
