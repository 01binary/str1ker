/*
                                                                                     @@@@@@@                  
 @@@@@@@@@@@@  @@@@@@@@@@@@   @@@@@@@@@@@@       @  @@@@@@@@@@@@@  @           @  @@@       @@@  @@@@@@@@@@@@ 
@              @ @           @            @    @ @  @              @        @@@      @@@@@@@    @            @
 @@@@@@@@@@@@  @   @         @@@@@@@@@@@@@   @   @   @             @   @@@@@      @@@       @@@ @@@@@@@@@@@@@ 
             @ @     @       @            @      @    @@           @@@@      @                  @            @
 @@@@@@@@@@@@  @       @     @            @      @      @@@@@@@@@  @          @   @@@       @@@ @            @
                                                                                     @@@@@@@                  
 ads1115.cpp

 Analog to Digital Converter ADS1115 implementation
 Created 02/22/2021

 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros/ros.h>
#include <pigpio.h>
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
const unsigned char ads1115::DEVICE_IDS[] = { 0x48, 0x49, 0x4A, 0x4B };

/*----------------------------------------------------------*\
| ads1115 implementation
\*----------------------------------------------------------*/

REGISTER_SINGLETON(ads1115)

ads1115::ads1115(const char* path) :
    adc(path),
    m_initialized(false),
    m_devices(1),
    m_i2cBus(-1),
    m_gain(gainMultiplier::PGA_2_048V),
    m_coefficient(0.0625),
    m_differential(false),
    m_sampleMode(sampleMode::SINGLE),
    m_sampleRate(sampleRate::SPS_128)
{
    memset(m_i2c, -1, sizeof(m_i2c));
}

const char* ads1115::getType()
{
    return ads1115::TYPE;
}

bool ads1115::init()
{
    if (!m_enable || m_initialized) return true;

    for (int n = 0; n < m_devices; n++)
    {
        m_i2c[n] = i2cOpen(m_i2cBus, DEVICE_IDS[n], 0);

        if (m_i2c[n] < 0)
        {
            ROS_ERROR("  failed to initialize ADS1115 ADC on I2C bus %d at 0x%x: 0x%x",
                m_i2cBus, DEVICE_IDS[n], m_i2c[n]);

            return false;
        }

        for (int channel = 0; channel < MAX_CHANNELS; channel++)
        {
            if (!configure(n, channel)) return false;
        }

        ROS_INFO("  initialized %s %s on i2c %d 0x%x", getPath(), getType(), m_i2cBus, DEVICE_IDS[n]);
    }

    m_initialized = true;

    return true;
}

int ads1115::getValue(int channel)
{
    int device = channel >> 2;

    if (!m_enable || !m_initialized || device >= m_devices || !configure(device, channel)) return 0;

    unsigned short data = i2cReadWordData(m_i2c[device], registers::CONVERT);

    if (data > 32767) data -= 65535;

    int ret = int(data * m_coefficient);

    if (channel == 0)
    {
        ROS_INFO("-- read %d on device %d channel %d", ret, device, channel);
    }

    return ret;
}

int ads1115::getMaxValue()
{
    // ADS1115 is 16-bit
    return 1 << 16;
}

int ads1115::getChannels()
{
    // A single ADS1115 has 4 channels
    return m_devices * 4;
}

void ads1115::deserialize(ros::NodeHandle node)
{
    controller::deserialize(node);

    ros::param::get(getControllerPath("i2c"), m_i2cBus);
    ros::param::get(getControllerPath("devices"), m_devices);

    string gainEnum;
    if (ros::param::get(getControllerPath("gain"), gainEnum))
    {
        if (gainEnum == "6.144V")
        {
            m_gain = gainMultiplier::PGA_6_144V;
        }
        else if (gainEnum == "4.096V")
        {
            m_gain = gainMultiplier::PGA_4_096V;
        }
        else if (gainEnum == "2.048V")
        {
            m_gain = gainMultiplier::PGA_2_048V;
        }
        else if (gainEnum == "1.024V")
        {
            m_gain = gainMultiplier::PGA_1_024V;
        }
        else if (gainEnum == "0.512V")
        {
            m_gain = gainMultiplier::PGA_0_512V;
        }
        else if (gainEnum == "0.256V")
        {
            m_gain = gainMultiplier::PGA_0_256V;
        }

        m_coefficient = getCoefficient();
    }

    ros::param::get(getControllerPath("differential"), m_differential);

    bool continuous = false;

    if (ros::param::get(getControllerPath("continuous"), continuous))
    {
        m_sampleMode = continuous ? sampleMode::CONTINUOUS : sampleMode::SINGLE;
    }

    int sampleRateNumber;

    if (ros::param::get(getControllerPath("rate"), sampleRateNumber))
    {
        switch(sampleRateNumber)
        {
        case 8:
            m_sampleRate = sampleRate::SPS_8;
            break;
        case 16:
            m_sampleRate = sampleRate::SPS_16;
            break;
        case 32:
            m_sampleRate = sampleRate::SPS_32;
            break;
        case 64:
            m_sampleRate = sampleRate::SPS_64;
            break;
        case 128:
            m_sampleRate = sampleRate::SPS_128;
            break;
        case 250:
            m_sampleRate = sampleRate::SPS_250;
            break;
        case 475:
            m_sampleRate = sampleRate::SPS_475;
            break;
        case 860:
            m_sampleRate = sampleRate::SPS_860;
            break;
        }
    }
}

ads1115::gainMultiplier ads1115::getGain()
{
    return m_gain;
}

double ads1115::getCoefficient()
{
    switch(m_gain)
    {
    case PGA_6_144V:
        return 0.1875;
    case PGA_4_096V:
        return 0.125;
    case PGA_2_048V:
        return 0.0625;
    case PGA_1_024V:
        return 0.03125;
    case PGA_0_512V:
        return 0.015625;
    case PGA_0_256V:
        return 0.0078125;
    default:
        return 0.125;
    }
}

void ads1115::setGain(gainMultiplier gain)
{
    m_gain = gain;
    m_coefficient = getCoefficient();
}

bool ads1115::getDifferential()
{
    return m_differential;
}

void ads1115::setDifferential(bool differential)
{
    m_differential = differential;
}

ads1115::sampleMode ads1115::getSampleMode()
{
    return m_sampleMode;
}

void ads1115::setSampleMode(sampleMode sampleMode)
{
    m_sampleMode = sampleMode;
}

ads1115::sampleRate ads1115::getSampleRate()
{
    return m_sampleRate;
}

void ads1115::setSampleRate(sampleRate sampleRate)
{
    m_sampleRate = sampleRate;
}

bool ads1115::configure(int device, int channel)
{
    if (!m_enable || m_i2c[device] < 0) return true;

    unsigned char ref = m_differential
        ? referenceMode::DIFF_0_1 + channel
        : referenceMode::SINGLE_0 + channel;

    unsigned char reg[2] =
    {
        (unsigned char)(command::CONV | ref | m_gain | m_sampleMode),
        (unsigned char)(m_sampleRate | alertMode::NONE)
    };

    unsigned short* word = (unsigned short*)reg;
    int result = i2cWriteWordData(m_i2c[device], registers::CONFIG, *word);
    const char* error = NULL;

    switch (result)
    {
    case PI_BAD_HANDLE:
        error = "bad handle";
        break;
    case PI_BAD_PARAM:
        error = "bad param";
        break;
    case PI_I2C_WRITE_FAILED:
        error = "I2C write failed";
        break;
    }

    if (result != 0)
    {
        ROS_ERROR("  failed to configure ADS1115 at 0x%x %s",
            DEVICE_IDS[device], error);

        return false;
    }
    else
    {
#if DEBUG
        ROS_INFO("  wrote 0x%x to register 0x%x on 0x%x",
            *word, registers::CONFIG, DEVICE_IDS[device]);
#endif
    }

    return true;
}

controller* ads1115::create(const char* path)
{
    return new ads1115(path);
}
