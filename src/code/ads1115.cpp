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
const uint8_t ads1115::DEVICE_IDS[] = { 0x48, 0x49, 0x4A, 0x4B };

const char* ads1115::OP_NAMES_READ[] =
{
    "busy",
    "ready"
};

const char* ads1115::OP_NAMES_WRITE[] =
{
    "idle",
    "convert"
};

const uint16_t ads1115::OP_VALUES[] = {
    operationStatus::read_busy,
    operationStatus::read_ready
};

const char* ads1115::MULTIPLEXER_NAMES[] =
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

const uint16_t ads1115::MULTIPLEXER_VALUES[] =
{
    referenceMode::diff_0_1,
    referenceMode::diff_0_3,
    referenceMode::diff_1_3,
    referenceMode::diff_2_3,
    referenceMode::single_0,
    referenceMode::single_1,
    referenceMode::single_2,
    referenceMode::single_3
};

const char* ads1115::GAIN_NAMES[] =
{
    "6.144V",
    "4.096V",
    "2.048V",
    "1.024V",
    "0.512V",
    "0.256V",
    "0.256V2",
    "0.256V3"
};

const uint16_t ads1115::GAIN_VALUES[] =
{
    gainMultiplier::pga_6_144V,
    gainMultiplier::pga_4_096V,
    gainMultiplier::pga_2_048V,
    gainMultiplier::pga_1_024V,
    gainMultiplier::pga_0_512V,
    gainMultiplier::pga_0_256V,
    gainMultiplier::pga_0_256V2,
    gainMultiplier::pga_0_256V3
};

const char* ads1115::MODE_NAMES[] =
{
    "continuous",
    "single"
};

const uint16_t ads1115::MODE_VALUES[] =
{
    sampleMode::continuous,
    sampleMode::single
};

const int ads1115::RATE_NAMES[] =
{
    8, 16, 32, 64, 128, 250, 475, 860
};

const char* ads1115::RATE_NAMES_PRINT[] =
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

const char* ads1115::COMP_NAMES[] =
{
    "traditional",
    "window"
};

const uint16_t ads1115::COMP_VALUES[] =
{
    comparatorMode::traditional,
    comparatorMode::window
};

const char* ads1115::LATCH_NAMES[] =
{
    "non-latching",
    "latching"
};

const uint16_t ads1115::LATCH_VALUES[] =
{
    latchMode::nonLatching,
    latchMode::latching
};

const char* ads1115::ALERT_NAMES[] =
{
    "alert1",
    "alert2",
    "alert4",
    "none"
};

const uint16_t ads1115::ALERT_VALUES[] =
{
    alertMode::alert1,
    alertMode::alert2,
    alertMode::alert4,
    alertMode::none
};

const char* ads1115::POLARITY_NAMES[] =
{
    "active low",
    "active high"
};

const uint16_t ads1115::POLARITY_VALUES[] =
{
    alertPolarity::activeLow,
    alertPolarity::activeHigh
};

const uint16_t ads1115::MUX_SINGLE[] =
{
    ads1115::single_0,
    ads1115::single_1,
    ads1115::single_2,
    ads1115::single_3
};

const uint16_t ads1115::MUX_DIFF[] =
{
    ads1115::diff_0_1,
    ads1115::diff_0_3,
    ads1115::diff_1_3,
    ads1115::diff_2_3
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
    setSampleRate(sampleRate::sps128);
    memset(m_i2c, -1, sizeof(m_i2c));
    memset(m_samples, 0, sizeof(m_samples));
    memset(m_lastConfChannel, -1, sizeof(m_lastConfChannel));
    memset(m_lastReadChannel, 0, sizeof(m_lastReadChannel));
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
            !configureDevice(device, 0) ||
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

    int conversion = i2cReadWordData(m_i2c[device], deviceRegister::conversion);

    if (conversion < 0)
    {
        ROS_ERROR("  failed to test %s conversion at 0x%x: %s", getName(), DEVICE_IDS[device], getError(conversion));
        return false;
    }

    m_samples[device * 4] = convertSample((uint8_t*)&conversion);
    m_lastReadChannel[device] = 0;

    ROS_INFO("  initialized %s %s on I2C%d at 0x%x", getPath(), getType(), m_i2cBus, DEVICE_IDS[device]);
    return true;
}

bool ads1115::configureDevice(int device, int deviceChannel)
{
    if (!m_enable || m_i2c[device] < 0 || m_lastConfChannel[device] == deviceChannel) return true;

    uint16_t conf =
        operationStatus::write_convert |
        MUX_SINGLE[deviceChannel] |
        m_gain |
        sampleMode::single |
        m_sampleRate |
        comparatorMode::traditional |
        alertPolarity::activeLow |
        latchMode::nonLatching |
        alertMode::none;

    int result = i2cWriteWordData(m_i2c[device], deviceRegister::configuration, conf);

    if (result != 0)
    {
        ROS_ERROR("  failed to configure ADS1115 channel %d at 0x%x: %s", deviceChannel, DEVICE_IDS[device], getError(result));
        dump(conf, false);
        return false;
    }

    m_lastConfChannel[device] = deviceChannel;

    if (!m_initialized)
    {
        dump(conf, false);
    }

    return true;
}

bool ads1115::poll(int device)
{
    if (!m_enable || device >= m_devices || m_i2c[device] < 0) return false;

    for (int n = 0; n < MAX_ATTEMPTS; n++)
    {
        uint16_t conf = i2cReadWordData(m_i2c[device], deviceRegister::configuration);
        if (conf & operationStatus::read_ready) return true;
    }

#ifdef DEBUG
    ROS_WARN("  failed to poll ADS1115 at 0x%x after %d attempts", DEVICE_IDS[device], MAX_ATTEMPTS);
#endif

    return false;
}

int ads1115::getValue(int channel)
{
    if (!m_enable || !m_initialized || channel >= m_devices * MAX_CHANNELS) return 0;
    return m_samples[channel];
}

int ads1115::getMaxValue()
{
    // Single-ended measurements read from 0 to 7FFFh
    return 0x7FFF;
}

int ads1115::getChannels()
{
    // A single ADS1115 has 4 channels
    return m_devices * MAX_CHANNELS;
}

void ads1115::publish()
{
    if (!m_enable || !m_initialized) return;

    int channel = 0;

    for (int device = 0; device < m_devices; device++)
    {
        for (int deviceChannel = 0; deviceChannel < MAX_CHANNELS; deviceChannel++, channel++)
        {
            if (m_lastReadChannel[device] == deviceChannel) continue;

            if (configure(channel) && poll(device))
            {
                int conversion = i2cReadWordData(m_i2c[device], deviceRegister::conversion);

                if (conversion < 0)
                {
                    ROS_ERROR("  failed to read ADS1115 channel %d conversion at 0x%x: %s",
                        deviceChannel, DEVICE_IDS[device], getError(conversion));

                    continue;
                }

                m_samples[channel] = convertSample((uint8_t*)&conversion);

                m_lastReadChannel[device] = deviceChannel;
            }
        }
    }

#ifdef DEBUG
    ROS_INFO("ADS 1115 [%s] %d [%s] %d [%s] %d [%s] %d",
        dumpValue(m_samples[0]).c_str(), m_samples[0],
        dumpValue(m_samples[1]).c_str(), m_samples[1],
        dumpValue(m_samples[2]).c_str(), m_samples[2],
        dumpValue(m_samples[3]).c_str(), m_samples[3]);
#endif
}

int ads1115::convertSample(uint8_t* data)
{
    return (data[1] | data[0] << 8);
}

void ads1115::deserialize(ros::NodeHandle node)
{
    controller::deserialize(node);

    ros::param::get(getControllerPath("i2c"), m_i2cBus);
    ros::param::get(getControllerPath("devices"), m_devices);

    string gain;
    if (ros::param::get(getControllerPath("gain"), gain))
    {
        m_gain = (gainMultiplier)getFlagsValue(
            gain.c_str(), GAIN_NAMES, GAIN_VALUES, sizeof(GAIN_VALUES) / sizeof(uint16_t));
    }

    int rate;
    if (ros::param::get(getControllerPath("rate"), rate))
    {
        for (int n = 0; n <= sizeof(RATE_NAMES) / sizeof(int); n++)
        {
            if (rate == RATE_NAMES[n])
            {
                m_sampleRate = (sampleRate)RATE_VALUES[n];
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

ads1115::sampleRate ads1115::getSampleRate()
{
    return m_sampleRate;
}

void ads1115::setSampleRate(sampleRate sampleRate)
{
    m_sampleRate = sampleRate;
}

bool ads1115::configure(int channel)
{
    int device = getDevice(channel);
    return configureDevice(device, channel - device * 4);
}

const char* ads1115::getField(uint16_t value, const char** names, const uint16_t* values, int count, int field)
{
    int defaultIndex = -1;
    int max = 0;

    value = value >> field;

    for (int n = 0; n < count; n++)
    {
        int shifted = values[n] >> field;
        if (shifted > max) max = shifted;
        if (values[n] == 0) defaultIndex = n;
    }

    if (max == 0b111)
    {
        // Mask out 3 bits because value is up to 3 bits long
        value &= 0x7;
    }
    else if (max == 0b11)
    {
        // Mask out 2 bits because value is up to 2 bits long
        value &= 0x3;
    }

    for (int n = 0; n < count; n++)
    {
        if (values[n] >> field == value) return names[n];
    }

    if (defaultIndex != -1) return names[defaultIndex];
    return "unknown";
}

const char* ads1115::getFlags(uint16_t value, const char** names, const uint16_t* values, int count)
{
    int defaultIndex = -1;
    
    for (int n = 0; n < count; n++)
    {
        if (values[n] == 0) defaultIndex = n;
        if (value & values[n]) return names[n];
    }

    if (defaultIndex != -1) return names[defaultIndex];
    return "unknown";
}

uint16_t ads1115::getFlagsValue(const char* name, const char** names, const uint16_t* values, int count)
{
    for (int n = 0; n < count; n++)
    {
        if (strcmp(name, names[n]) == 0) return values[n];
    }

    return 0;
}

void ads1115::dump(uint16_t conf, bool read)
{
    ROS_INFO("    operation   = %s",
        getFlags(conf, read ? OP_NAMES_READ : OP_NAMES_WRITE, OP_VALUES, sizeof(OP_VALUES) / sizeof(uint16_t)));

    ROS_INFO("    multiplexer = %s",
        getField(conf, MULTIPLEXER_NAMES, MULTIPLEXER_VALUES, sizeof(MULTIPLEXER_VALUES) / sizeof(uint16_t), 12));

    ROS_INFO("    gain        = %s",
        getField(conf, GAIN_NAMES, GAIN_VALUES, sizeof(GAIN_VALUES) / sizeof(uint16_t), 9));

    ROS_INFO("    mode        = %s",
        getFlags(conf, MODE_NAMES, MODE_VALUES, sizeof(MODE_VALUES) / sizeof(uint16_t)));

    ROS_INFO("    rate        = %s",
        getField(conf, RATE_NAMES_PRINT, RATE_VALUES, sizeof(RATE_VALUES) / sizeof(uint16_t), 5));

    ROS_INFO("    comparator  = %s",
        getFlags(conf, COMP_NAMES, COMP_VALUES, sizeof(COMP_VALUES) / sizeof(uint16_t)));

    ROS_INFO("    polarity    = %s",
        getFlags(conf, POLARITY_NAMES, POLARITY_VALUES, sizeof(POLARITY_VALUES) / sizeof(uint16_t)));

    ROS_INFO("    latch       = %s",
        getFlags(conf, LATCH_NAMES, LATCH_VALUES, sizeof(LATCH_VALUES) / sizeof(uint16_t)));

    ROS_INFO("    alert       = %s",
        getField(conf, ALERT_NAMES, ALERT_VALUES, sizeof(ALERT_VALUES) / sizeof(uint16_t), 0));
}

string ads1115::dumpValue(int value)
{
    const int MIN = 0;
    const int MAX = 0x7FFF;
    const int RANGE = MAX - MIN;

    if (value < MIN) value = MIN;
    if (value > MAX) value = MAX;

    int scaled = int(double(value - MIN) / double(RANGE) * RES);
    char scale[RES + 1] = {0};

    for (int n = 0; n < RES; n++)
    {
        if (n < scaled)
            scale[n] = '|';
        else
            scale[n] = ' ';
    }

    scale[RES] = 0;

    return scale;
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
