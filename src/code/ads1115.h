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

*/

class ads1115 : public adc
{
public:
    enum deviceRegister
    {
        conversion,         // Conversion register
        configuration,      // Configuration register
        loThreshold,        // Lo_thresh register
        hiThreshold         // Hi_thresh register
    };

    enum state
    {
        idle,               // Not performing conversion
        convert             // Perform(ing) conversion
    };

    enum gainMultiplier
    {
        pga_6_144V,         // +/-6.144V range = Gain 2/3
        pga_4_096V,         // +/-4.096V range = Gain 1
        pga_2_048V,         // +/-2.048V range = Gain 2 (default)
        pga_1_024V,         // +/-1.024V range = Gain 4
        pga_0_512V,         // +/-0.512V range = Gain 8
        pga_0_256V         // +/-0.256V range = Gain 16
    };

    enum referenceMode
    {
        diff_0_1,           // 000 Differential P = AIN0, N = AIN1 (default)
        diff_0_3,           // 001 Differential P = AIN0, N = AIN3
        diff_1_3,           // 010 Differential P = AIN1, N = AIN3
        diff_2_3,           // 011 Differential P = AIN2, N = AIN3
        single_0,           // 100 Single-ended P = AIN0, N = GND
        single_1,           // 101 Single-ended P = AIN1, N = GND
        single_2,           // 110 Single-ended P = AIN2, N = GND
        single_3            // 111 Single-ended P = AIN3, N = GND
    };

    enum sampleMode
    {
        continuous,         // Continuous conversion mode
        single              // Power-down single-shot mode (default)
    };

    enum sampleRate
    {
        sps8,               // 8 samples per second
        sps16,              // 16 samples per second
        sps32,              // 32 samples per second
        sps64,              // 64 samples per second
        sps128,             // 128 samples per second (default)
        sps250,             // 250 samples per second
        sps475,             // 475 samples per second
        sps860              // 860 samples per second
    };

    enum comparatorMode
    {
        traditional,        // Traditional comparator (default)
        window              // Window comparator
    };

    enum latchMode
    {
        nonLatching,        // The ALERT/RDY pin does not latch when asserted (default)
        latching            // The asserted ALERT/RDY pin remains latched until data is read
    };

    enum alertMode
    {
        alert1,             // Assert ALERT/RDY after one conversions
        alert2,             // Assert ALERT/RDY after two conversions
        alert4,             // Assert ALERT/RDY after four conversions
        none                // Disable the comparator and put ALERT/RDY in high state (default)
    };

    enum alertPolarity
    {
        activeLow,          // Polarity of the ALERT/RDY pin active low (default)
        activeHigh          // Polarity of the ALERT/RDY pin active high
    };

public:
    // Controller type
    static const char TYPE[];

private:
     // I2C bus ID where ADS1115 is attached
    int m_i2cBus;

    // I2C device handles
    int m_i2c[MAX_DEVICES];

    // Whether ADS was initialized
    bool m_initialized;

    // Number of chained ADS1115 devices up to MAX_DEVICES
    int m_devices;

    // Last time each device was sampled
    timeval m_last[MAX_DEVICES];

    // Gain mode
    gainMultiplier m_gain;

    // Gain coefficient
    double m_coefficient;

    // Reference mode
    bool m_differential;

    // Sample rate
    sampleRate m_sampleRate;

    // Time it takes to acquire a sample
    unsigned int m_sampleTimeMilliseconds;

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

    // Load controller settings
    virtual void deserialize(ros::NodeHandle node);

    // Gain multiplier
    gainMultiplier getGain();
    void setGain(gainMultiplier gain);

    // Reference mode
    bool getDifferential();
    void setDifferential(bool differential);

    // Sample rate
    sampleRate getSampleRate();
    void setSampleRate(sampleRate sampleRate);

private:
    static const unsigned char DEVICE_IDS[];
    static const char* OP[];
    static const char* MULTIPLEXER[];
    static const char* GAIN[];
    static const char* MODE[];
    static const int RATE[];
    static const char* COMP[];
    static const char* LATCH[];
    static const char* ALERT[];
    static const char* POLARITY[];

    struct config
    {
        alertMode alert : 2;
        latchMode latch: 1;
        alertPolarity polarity: 1;
        comparatorMode comparator: 1;
        sampleRate rate: 3;
        sampleMode mode: 1;
        gainMultiplier gain: 3;
        referenceMode multiplexer: 3;
        state operation: 1;

        explicit operator unsigned int()
        {
            return (unsigned int)(*((unsigned short*)this));
        }
    };

private:
    bool openDevice(int device);
    bool testDevice(int device);
    bool configureDevice(int device, int deviceChannel, state operation);
    inline int getDevice(int channel) { return channel >> 2;}
    bool configure(int channel, state operation);
    bool trigger(int channel);
    bool poll(int channel);
    void wait(int device);
    void dump(config* conf);
    double getCoefficient();
    const char* getError(int result);

public:
    // Create instance
    static controller* create(const char* path);
};

} // namespace str1ker
