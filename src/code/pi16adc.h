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
    const int CHANNELS = 16;
    const int MAX_VALUE = 0x800000;
    const int BLOCK_SIZE = 6;
    const int SLEEP_TIME = 100 * 1000;

    enum channels
    {
        channel0 =  0xB0,
        channel1 =  0xB8,
        channel2 =  0xB1,
        channel3 =  0xB9,
        channel4 =  0xB2,
        channel5 =  0xBA,
        channel6 =  0xB3,
        channel7 =  0xBB,
        channel8 =  0xB4,
        channel9 =  0xBC,
        channel10 = 0xB5,
        channel11 = 0xBD,
        channel12 = 0xB6,
        channel13 = 0xBE,
        channel14 = 0xB7,
        channel15 = 0xBF
    };

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
    bool selectChannel(channels channel);
    bool readRegister(int reg, uint8_t *value, int size);
    bool writeRegister(int reg, uint8_t* value, int size);


public:
    // Create instance
    static controller* create(const char* path);
};

} // namespace str1ker
