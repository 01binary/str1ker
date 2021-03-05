/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 ads1115.cpp

 Analog to Digital Converter Using ADS1115 Implementation
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

const char* ads1115::OP[] =
{
    "idle",
    "converting"
};

const char* ads1115::MULTIPLEXER[] =
{
    "diff_0_1",
    "diff_0_3",
    "diff_1_3",
    "diff_2_3",
    "single_0",
    "single_1",
    "single_2",
    "single_3"
};

const char* ads1115::GAIN[] =
{
    "6.144V",
    "4.096V",
    "2.048V",
    "1.024V",
    "0.512V",
    "0.256V"
};

const char* ads1115::MODE[] =
{
    "continuous",
    "single"
};

const int ads1115::RATE[] =
{
    8, 16, 32, 64, 128, 250, 475, 860
};

const char* ads1115::COMP[] =
{
    "traditional",
    "window"
};

const char* ads1115::LATCH[] =
{
    "non-latching",
    "latching"
};

const char* ads1115::ALERT[] =
{
    "alert1",
    "alert2",
    "alert4",
    "none"
};

const char* ads1115::POLARITY[] = {
    "active low",
    "active high"
};

/*----------------------------------------------------------*\
| ads1115 implementation
\*----------------------------------------------------------*/

REGISTER_SINGLETON(ads1115)

ads1115::ads1115(const char* path) :
    adc(path),
    m_initialized(false),
    m_devices(1),
    m_i2cBus(-1)
{
    setGain(gainMultiplier::pga_2_048V);
    setDifferential(false);
    setSampleRate(sampleRate::sps128);
    memset(m_i2c, -1, sizeof(m_i2c));
}

const char* ads1115::getType()
{
    return ads1115::TYPE;
}

bool ads1115::init()
{
    if (!m_enable || m_initialized) return true;

    for (int device = 0; device < m_devices; device++)
    {
        if (!openDevice(device) ||
            !configureDevice(device, 0, state::idle, true) ||
            !testDevice(device))
            return false;
    }

    m_initialized = true;

    return true;
}

bool ads1115::openDevice(int device)
{
    m_i2c[device] = i2cOpen(m_i2cBus, DEVICE_IDS[device], 0);

    if (m_i2c[device] < 0)
    {
        ROS_ERROR("  failed to access ADS1115 ADC on I2C%d at 0x%x: %s",
            m_i2cBus, DEVICE_IDS[device], getError(m_i2c[device]));

        return false;
    }

    return true;
}

bool ads1115::testDevice(int device)
{
    if (!m_enable || m_i2c[device] < 0) return true;

    usleep(m_sampleTimeMilliseconds * 1000);

    int result = i2cReadWordData(m_i2c[device], deviceRegister::conversion);

    gettimeofday(&m_last[device], NULL);

    if (result < 0)
    {
        ROS_ERROR("  failed to test ADS1115 conversion at 0x%x: %s", DEVICE_IDS[device], getError(result));
        return false;
    }

    ROS_INFO("  initialized %s %s on I2C%d at 0x%x", getPath(), getType(), m_i2cBus, DEVICE_IDS[device]);

    unsigned short word = i2cReadWordData(m_i2c[device], deviceRegister::configuration);
    config* conf = (config*)&word;
    dump(conf);

    if (m_gain != conf->gain)
    {
        ROS_WARN("    synced gain from %s to %s", GAIN[m_gain], GAIN[conf->gain]);
        setGain(conf->gain);
    }

    if (m_sampleRate != conf->rate)
    {
        ROS_WARN("    synced sample rate from %d sps to %d sps", RATE[m_sampleRate], RATE[conf->rate]);
        setSampleRate(conf->rate);
    }

    return true;
}

void ads1115::dump(config* conf)
{
    ROS_INFO("    operation   = %s", OP[conf->operation]);
    ROS_INFO("    multiplexer = %s", MULTIPLEXER[conf->multiplexer]);
    ROS_INFO("    gain        = %s", GAIN[conf->gain]);
    ROS_INFO("    mode        = %s", MODE[conf->mode]);
    ROS_INFO("    rate        = %d s/sec (%d ms per sample)", RATE[conf->rate], 1000 / RATE[conf->rate]);
    ROS_INFO("    comparator  = %s", COMP[conf->comparator]);
    ROS_INFO("    polarity    = %s", POLARITY[conf->polarity]);
    ROS_INFO("    latch       = %s", LATCH[conf->latch]);
    ROS_INFO("    alert       = %s", ALERT[conf->alert]);
}

bool ads1115::configureDevice(int device, int deviceChannel, state operation, bool reset)
{
    if (!m_enable || m_i2c[device] < 0) return true;

    config conf;
    conf.operation = operation;
    conf.multiplexer = m_differential
        ? (referenceMode)(referenceMode::diff_0_1 + deviceChannel)
        : (referenceMode)(referenceMode::single_0 + deviceChannel);
    conf.gain = m_gain;
    conf.mode = reset ? sampleMode::continuous : sampleMode::single;
    conf.rate = m_sampleRate;
    conf.comparator = comparatorMode::traditional;
    conf.polarity = alertPolarity::activeLow;
    conf.latch = latchMode::nonLatching;
    conf.alert = alertMode::none;

    int result = i2cWriteWordData(m_i2c[device], deviceRegister::configuration, (unsigned int)conf);
 
    if (result != 0)
    {
        ROS_ERROR("  failed to configure ADS1115 at 0x%x: %s", DEVICE_IDS[device], getError(result));
        return false;
    }

    if (reset)
    {
        result = i2cReadWordData(m_i2c[device], deviceRegister::conversion);

        conf.mode = sampleMode::single;

        result = i2cWriteWordData(m_i2c[device], deviceRegister::configuration, (unsigned int)conf);
 
        if (result != 0)
        {
            ROS_ERROR("  failed to reset ADS1115 at 0x%x: %s", DEVICE_IDS[device], getError(result));
            return false;
        }
    }

    return true;
}

bool ads1115::trigger(int channel)
{
    return configure(channel, state::convert);
}

bool ads1115::poll(int channel)
{
    int device = getDevice(channel);

    if (!m_enable || device >= m_devices || m_i2c[device] < 0) return false;

    for (int n = 0; n < 8; n++)
    {
        unsigned short word = i2cReadWordData(m_i2c[device], deviceRegister::configuration);
        config* conf = (config*)&word;

        if (conf->operation == state::convert) return true;
    }

    return false;
}

void ads1115::wait(int device)
{
    timeval now;
    gettimeofday(&now, NULL);

    time_t diffSeconds = now.tv_sec - m_last[device].tv_sec;
    time_t diffMicroseconds = now.tv_usec - m_last[device].tv_usec;
    unsigned int diffMilliseconds = diffSeconds * 1000 + diffMicroseconds / 1000;

    if (diffMilliseconds < m_sampleTimeMilliseconds)
    {
        unsigned int diff = m_sampleTimeMilliseconds - diffMilliseconds;
        usleep(diff * 1000 + 20);
    }

    gettimeofday(&m_last[device], NULL);
}

int ads1115::getValue(int channel)
{
    int device = getDevice(channel);

    if (!m_enable || !m_initialized || device >= m_devices) return 0;

    if (!trigger(channel) || !poll(channel)) return 0;

    wait(device);

    int bigEndian = i2cReadWordData(m_i2c[device], deviceRegister::conversion);
    char* bigEndianBytes = (char*)&bigEndian;
    char littleEndianBytes[] = {bigEndianBytes[1], bigEndianBytes[0]};
    unsigned short littleEndian = *((unsigned short*)littleEndianBytes);

    int result = littleEndian;

    ROS_INFO(
        "-- read %d (<= %d 0x%x) [0x%x 0x%x] max %d on device %d channel %d",
        result,
        littleEndian,
        littleEndian,
        bigEndianBytes[0], bigEndianBytes[1],
        getMaxValue(),
        device,
        channel);

    return result;
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

    string gain;
    if (ros::param::get(getControllerPath("gain"), gain))
    {
        for (int n = 0; n <= gainMultiplier::pga_0_256V; n++)
        {
            if (gain == GAIN[n])
            {
                setGain((gainMultiplier)n);
                break;
            }
        }
    }

    bool diff = false;
    if (ros::param::get(getControllerPath("differential"), diff))
        setDifferential(diff);

    int rate;
    if (ros::param::get(getControllerPath("rate"), rate))
    {
        for (int n = 0; n <= sampleRate::sps860; n++)
        {
            if (rate == RATE[n])
            {
                setSampleRate((sampleRate)n);
                break;
            }
        }
    }
}

ads1115::gainMultiplier ads1115::getGain()
{
    return m_gain;
}

void ads1115::setGain(gainMultiplier gain)
{
    m_gain = gain;
}

bool ads1115::getDifferential()
{
    return m_differential;
}

void ads1115::setDifferential(bool differential)
{
    m_differential = differential;
}

ads1115::sampleRate ads1115::getSampleRate()
{
    return m_sampleRate;
}

void ads1115::setSampleRate(sampleRate sampleRate)
{
    m_sampleRate = sampleRate;
    m_sampleTimeMilliseconds = 1000 / RATE[m_sampleRate];
}

bool ads1115::configure(int channel, state operation)
{
    int device = getDevice(channel);
    int deviceChannel = channel - device * 4;

    return configureDevice(device, deviceChannel, operation, false);
}

const char* ads1115::getError(int result)
{
    static char buffer[256] = {0};

    switch (result)
    {
    case PI_BAD_I2C_BUS:
        return "bad I2C bus";
    case PI_BAD_I2C_ADDR:
        return "bad I2C address";
    case PI_BAD_FLAGS:
        return "bad I2C flags";
    case PI_NO_HANDLE:
        return "no handle";
    case PI_I2C_OPEN_FAILED:
        return "I2C open failed";
    case PI_BAD_HANDLE:
        return "bad handle";
    case PI_BAD_PARAM:
        return "bad param";
    case PI_I2C_WRITE_FAILED:
        return "I2C write failed";
    case PI_I2C_READ_FAILED:
        return "I2C read failed";
    default:
        {
            sprintf(buffer, "unknown error %d (0x%x)", result, result);
            return buffer;
        }
    }
}

controller* ads1115::create(const char* path)
{
    return new ads1115(path);
}
