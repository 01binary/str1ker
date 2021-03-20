/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 ads1115.h

 Analog to Digital Converter Using ADS1115
 Created 02/22/2021

 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Definitions
\*----------------------------------------------------------*/

#define MAX_DEVICES 4
#define MAX_CHANNELS 4
#define TOTAL_CHANNELS MAX_DEVICES * MAX_CHANNELS
#define MAX_ATTEMPTS 8
#define RES 16

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <sys/time.h>
#include <string>
#include "adc.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| ads1115 class
\*----------------------------------------------------------*/

/*

 Wiring ADS1115 on Raspberri PI 4B I2C1:

 VDD - 3V3
 GND - GND
 SCL - SCL
 SDA - SDA

 Chaining ADS1115 devices on I2C bus:

 Channels 0 - 3:    tie ADDR to GND, I2C address 0x48 (1 device)
 Channels 4 - 7:    tie ADDR to VDD, I2C address 0x49 (2 devices)
 Channels 8 - 11:   tie ADDR to SDA, I2C address 0x4A (3 devices)
 Channels 12 - 15:  tie ADDR to SCL, I2C address 0x4B (4 devices)

 This class supports only single conversions

*/

class ads1115 : public adc
{
public:
    enum deviceRegister
    {
        conversion    = 0x0000, // Conversion register
        configuration = 0x0001, // Configuration register
        loThreshold   = 0x0002, // Lo_thresh register
        hiThreshold   = 0x0003  // Hi_thresh register
    };

    enum operationStatus
    {
        write_idle    = 0x0000, // Write: not requesting conversion
        write_convert = 0x8000, // Write: requesting conversion
        read_busy     = 0x0000, // Read: conversion not ready
        read_ready    = 0x8000  // Read 
    };

    enum gainMultiplier
    {
        pga_6_144V    = 0x0000, // +/-6.144V range = Gain 2/3
        pga_4_096V    = 0x0200, // +/-4.096V range = Gain 1
        pga_2_048V    = 0x0400, // +/-2.048V range = Gain 2 (default)
        pga_1_024V    = 0x0600, // +/-1.024V range = Gain 4
        pga_0_512V    = 0x0800, // +/-0.512V range = Gain 8
        pga_0_256V    = 0x0A00, // +/-0.256V range = Gain 16
        pga_0_256V2   = 0x0C00, // dummy 1
        pga_0_256V3   = 0x0E00  // dummy 2
    };

    enum referenceMode
    {
        diff_0_1      = 0x0000, // 000 Differential P = AIN0, N = AIN1 (default)
        diff_0_3      = 0x1000, // 001 Differential P = AIN0, N = AIN3
        diff_1_3      = 0x2000, // 010 Differential P = AIN1, N = AIN3
        diff_2_3      = 0x3000, // 011 Differential P = AIN2, N = AIN3
        single_0      = 0x4000, // 100 Single-ended P = AIN0, N = GND
        single_1      = 0x5000, // 101 Single-ended P = AIN1, N = GND
        single_2      = 0x6000, // 110 Single-ended P = AIN2, N = GND
        single_3      = 0x7000  // 111 Single-ended P = AIN3, N = GND
    };

    enum sampleMode
    {
        continuous    = 0x000,  // Continuous conversion mode
        single        = 0x0100  // Power-down single-shot mode (default)
    };

    enum sampleRate
    {
        sps8          = 0x0000, // 8 samples per second
        sps16         = 0x0020, // 16 samples per second
        sps32         = 0x0040, // 32 samples per second
        sps64         = 0x0060, // 64 samples per second
        sps128        = 0x0080, // 128 samples per second (default)
        sps250        = 0x00A0, // 250 samples per second
        sps475        = 0x00C0, // 475 samples per second
        sps860        = 0x00E0  // 860 samples per second
    };

    enum comparatorMode
    {
        traditional   = 0x0000, // Traditional comparator (default)
        window        = 0x0010  // Window comparator
    };

    enum latchMode
    {
        nonLatching   = 0x0000, // The ALERT/RDY pin does not latch when asserted (default)
        latching      = 0x0004  // The asserted ALERT/RDY pin remains latched until data is read
    };

    enum alertMode
    {
        alert1        = 0x0000, // Assert ALERT/RDY after one conversions
        alert2        = 0x0001, // Assert ALERT/RDY after two conversions
        alert4        = 0x0002, // Assert ALERT/RDY after four conversions
        none          = 0x0003  // Disable the comparator and put ALERT/RDY in high state (default)
    };

    enum alertPolarity
    {
        activeLow     = 0x0000, // Polarity of the ALERT/RDY pin active low (default)
        activeHigh    = 0x0008  // Polarity of the ALERT/RDY pin active high
    };

public:
    // Controller type
    static const char TYPE[];

private:
     // I2C bus ID where ADS1115 is attached
    int m_i2cBus;

    // I2C device handles
    int m_i2c[MAX_DEVICES];

    // Last published values for all channels
    int m_samples[TOTAL_CHANNELS];

    // Whether ADS was initialized
    bool m_initialized;

    // Number of chained ADS1115 devices up to MAX_DEVICES
    int m_devices;

    // Gain mode
    gainMultiplier m_gain;

    // Sample rate
    sampleRate m_sampleRate;

public:
    ads1115(const char* path);

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

    // Gain multiplier
    gainMultiplier getGain();
    void setGain(gainMultiplier gain);

    // Sample rate
    sampleRate getSampleRate();
    void setSampleRate(sampleRate sampleRate);

private:
    static const uint8_t DEVICE_IDS[];
    static const char* OP_NAMES_READ[];
    static const char* OP_NAMES_WRITE[];
    static const uint16_t OP_VALUES[];
    static const char* MULTIPLEXER_NAMES[];
    static const uint16_t MULTIPLEXER_VALUES[];
    static const char* GAIN_NAMES[];
    static const uint16_t GAIN_VALUES[];
    static const char* MODE_NAMES[];
    static const uint16_t MODE_VALUES[];
    static const int RATE_NAMES[];
    static const char* RATE_NAMES_PRINT[];
    static const uint16_t RATE_VALUES[];
    static const char* COMP_NAMES[];
    static const uint16_t COMP_VALUES[];
    static const char* LATCH_NAMES[];
    static const uint16_t LATCH_VALUES[];
    static const char* ALERT_NAMES[];
    static const uint16_t ALERT_VALUES[];
    static const char* POLARITY_NAMES[];
    static const uint16_t POLARITY_VALUES[];
    static const uint16_t MUX_SINGLE[];
    static const uint16_t MUX_DIFF[];

private:
    bool openDevice(int device);
    bool testDevice(int device);
    bool configureDevice(int device, int deviceChannel);
    inline int getDevice(int channel) { return channel >> 2;}
    bool configure(int channel);
    bool poll(int device);
    void dump(uint16_t conf, bool read);
    double getCoefficient();
    std::string dumpValue(int value);
    const char* getFlagsName(uint16_t value, const char** names, const uint16_t* values, int count, int shift = 0);
    uint16_t getFlagsValue(const char* name, const char** names, const uint16_t* values, int count);
    const char* getError(int result);

public:
    // Create instance
    static controller* create(const char* path);
};

} // namespace str1ker
