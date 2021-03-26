/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 ads1115.h

 ADS1115 Analog to Digital Converter
 Created 02/22/2021

 Copyright (C) 2021 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include "adc.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| ads1115 class
\*----------------------------------------------------------*/

class ads1115 : public adc
{
public:
    // Max ads1115 devices on a single I2C bus
    static const int MAX_DEVICES = 4;

    // Max channels available on each ads1115 device
    static const int DEVICE_CHANNELS = 4;

    // How many times to retry polling for conversion result
    static const int MAX_RETRY = 4;

    // Publish queue size
    static const int PUBLISH_QUEUE = 16;

    // ads1115 registers
    enum deviceRegister
    {
        conversion    = 0b00,           // Conversion register
        configuration = 0b01,           // Configuration register
        loThreshold   = 0b10,           // Lo_thresh register
        hiThreshold   = 0b11            // Hi_thresh register
    };

    // configuration register bit 15 (1 bit long) meaning when reading
    enum readOperation
    {
        busy          = 0b0 << 15,      // Conversion not ready
        ready         = 0b1 << 15       // Conversion ready to read
    };

    // configuration register bit 15 (1 bit long) meaning when writing
    enum writeOperation
    {
        noop          = 0b0 << 15,      // Not requesting anything
        convert       = 0b1 << 15,      // Requesting conversion
    };

    // configuration register bits 12-14 (3 bits long)
    enum multiplexer
    {
        diff_0_1      = 0b000 << 12,    // Differential P = AIN0, N = AIN1 (default)
        diff_0_3      = 0b001 << 12,    // Differential P = AIN0, N = AIN3
        diff_1_3      = 0b010 << 12,    // Differential P = AIN1, N = AIN3
        diff_2_3      = 0b011 << 12,    // Differential P = AIN2, N = AIN3
        single_0      = 0b100 << 12,    // Single-ended P = AIN0, N = GND
        single_1      = 0b101 << 12,    // Single-ended P = AIN1, N = GND
        single_2      = 0b110 << 12,    // Single-ended P = AIN2, N = GND
        single_3      = 0b111 << 12     // Single-ended P = AIN3, N = GND
    };

    // configuration register bits 9-11 (3 bits long)
    enum gain
    {
        pga_6_144V    = 0b000 << 9,     // +/-6.144V range = Gain 2/3
        pga_4_096V    = 0b001 << 9,     // +/-4.096V range = Gain 1
        pga_2_048V    = 0b010 << 9,     // +/-2.048V range = Gain 2 (default)
        pga_1_024V    = 0b011 << 9,     // +/-1.024V range = Gain 4
        pga_0_512V    = 0b100 << 9,     // +/-0.512V range = Gain 8
        pga_0_256V    = 0b101 << 9,     // +/-0.256V range = Gain 16
        pga_0_256V2   = 0b110 << 9,     // not used
        pga_0_256V3   = 0b111 << 9      // not used
    };

    // configuration register bit 8 (1 bit long)
    enum sampleMode
    {
        continuous    = 0b0 << 8,       // Continuous conversion mode
        single        = 0b1 << 8        // Power-down single-shot mode (default)
    };

    // configuration register bits 5-7 (3 bits long)
    enum sampleRate
    {
        sps8          = 0b000 << 5,     // 8 samples per second
        sps16         = 0b001 << 5,     // 16 samples per second
        sps32         = 0b010 << 5,     // 32 samples per second
        sps64         = 0b011 << 5,     // 64 samples per second
        sps128        = 0b100 << 5,     // 128 samples per second (default)
        sps250        = 0b101 << 5,     // 250 samples per second
        sps475        = 0b110 << 5,     // 475 samples per second
        sps860        = 0b111 << 5,     // 860 samples per second
    };

    // configuration register bit 4 (1 bit long)
    enum comparatorMode
    {
        traditional   = 0b0 << 4,       // Traditional comparator (default)
        window        = 0b1 << 4        // Window comparator
    };

    // configuration register bit 3 (1 bit long)
    enum alertPolarity
    {
        activeLow     = 0b0 << 3,       // Polarity of the ALERT/RDY pin active low (default)
        activeHigh    = 0b1 << 3        // Polarity of the ALERT/RDY pin active high
    };

    // configuration register bit 2 (1 bit long)
    enum latchMode
    {
        nonLatching   = 0b0 << 2,       // The ALERT/RDY pin does not latch when asserted (default)
        latching      = 0b1 << 2        // The asserted ALERT/RDY pin remains latched until data is read
    };

    // configuration register bits 0-1 (2 bits long)
    enum alertMode
    {
        alert1        = 0b00 << 0,      // Assert ALERT/RDY after one conversions
        alert2        = 0b01 << 0,      // Assert ALERT/RDY after two conversions
        alert4        = 0b10 << 0,      // Assert ALERT/RDY after four conversions
        none          = 0b11 << 0       // Disable the comparator and put ALERT/RDY in high state (default)
    };

    // Controller type
    static const char TYPE[];

private:
    // I2C bus where ADS1115 device(s) are attached
    int m_i2cBus;

    // Number of chained ADS1115 devices up to MAX_DEVICES
    int m_devices;

    // Last device that was selected on I2C bus and configured
    int m_lastDevice;

    // Last device channel that was configured on last device
    int m_lastDeviceChannel;

    // Gain mode (configurable before initialization)
    gain m_gain;

    // Sample rate (configurable before initialization)
    sampleRate m_rate;

    // I2C device handle
    int m_i2cHandle;

    // Publisher for last sample on each channel
    ros::Publisher m_pub;

    // Last sample recorded for each channel
    uint16_t m_lastSample[MAX_DEVICES * DEVICE_CHANNELS];

public:
    ads1115(const char* path);

public:
    // Create instance
    static controller* create(const char* path);

    // Get display type
    virtual const char* getType();

    // Initialize
    virtual bool init();

    // Get last published channel value after calling publish()
    virtual int getValue(int channel);

    // Get max possible value
    virtual int getMaxValue();

    // Get number of channels available across all devices on I2C bus
    virtual int getChannels();

    // Publish channel values
    virtual void publish();

    // Load controller settings
    virtual void deserialize(ros::NodeHandle node);

private:
    // Configure device settings and switch device channel
    bool configureDevice(int deviceIndex, int deviceChannelIndex, uint16_t config);

    // Poll last configured device until conversion is ready
    bool pollDevice();

    // Read conversion for last configured device and channel
    bool readConversion(uint16_t& value);

private:
    // i2c addresses for up to four ads1115 units on the same bus
    static const uint8_t DEVICE_ADDRESS[];

    // select channel commands (lookup table indexed by channel)
    static const uint16_t SELECT_CHANNEL[];

    // how long to wait after sending commands (lookup table indexed by sample rate)
    static const int DELAYS[];

    // gain configuration value names
    static const char* GAIN_NAMES[];

    // gain configuration values
    static const uint16_t GAIN_VALUES[];

    // sample rates
    static const int RATES[];

    // sample rate names
    static const char* RATE_NAMES[];

    // sample rate values
    static const uint16_t RATE_VALUES[];
};

} // namespace str1ker
