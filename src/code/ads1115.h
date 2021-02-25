/*
                                                                                     @@@@@@@                  
 @@@@@@@@@@@@  @@@@@@@@@@@@   @@@@@@@@@@@@       @  @@@@@@@@@@@@@  @           @  @@@       @@@  @@@@@@@@@@@@ 
@              @ @           @            @    @ @  @              @        @@@      @@@@@@@    @            @
 @@@@@@@@@@@@  @   @         @@@@@@@@@@@@@   @   @   @             @   @@@@@      @@@       @@@ @@@@@@@@@@@@@ 
             @ @     @       @            @      @    @@           @@@@      @                  @            @
 @@@@@@@@@@@@  @       @     @            @      @      @@@@@@@@@  @          @   @@@       @@@ @            @
                                                                                     @@@@@@@                  
 ads1115.h

 Analog to Digital Converter using ads1115
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

 Wiring ADS1115 on Raspberri PI 4B I2C0:

 VDD - 5V
 GND - GND
 SCL - SCL
 SDA - SDA

 Chaining multiple ADS1115 devices on I2C bus:

 Channels 0 - 3: tie ADDR to GND, I2C address 0x48
 Channels 4 - 7: tie ADDR to VDD, I2C address 0x49
 Channels 8 - 11: tie ADDR to SDA, I2C address 0x4A
 Channels 12 - 15: tie ADDR to SCL, I2C address 0x4B

*/

class ads1115 : public adc
{
public:
    enum registers
    {
        CONVERT     = 0x00, // Conversion register
        CONFIG      = 0x01, // Configuration register
        LOWTHRESH   = 0x02, // Lo_thresh register
        HITHRESH    = 0x03  // Hi_thresh register
    };

    enum command
    {
        NOOP    = 0x00,     // No operation
        CONV    = 0x80      // Single conversion
    };

    enum gainMultiplier
    {
        PGA_6_144V  = 0x00, // +/-6.144V range = Gain 2/3
        PGA_4_096V  = 0x02, // +/-4.096V range = Gain 1
        PGA_2_048V  = 0x04, // +/-2.048V range = Gain 2 (default)
        PGA_1_024V  = 0x06, // +/-1.024V range = Gain 4
        PGA_0_512V  = 0x08, // +/-0.512V range = Gain 8
        PGA_0_256V  = 0x0A  // +/-0.256V range = Gain 16
    };

    enum referenceMode
    {
        DIFF_0_1    = 0x00, // Differential P = AIN0, N = AIN1 (default)
        DIFF_0_3    = 0x10, // Differential P = AIN0, N = AIN3
        DIFF_1_3    = 0x20, // Differential P = AIN1, N = AIN3
        DIFF_2_3    = 0x30, // Differential P = AIN2, N = AIN3
        SINGLE_0    = 0x40, // Single-ended P = AIN0, N = GND
        SINGLE_1    = 0x50, // Single-ended P = AIN1, N = GND
        SINGLE_2    = 0x60, // Single-ended P = AIN2, N = GND
        SINGLE_3    = 0x70  // Single-ended P = AIN3, N = GND
    };

    enum sampleMode
    {
        CONTINUOUS	= 0x00, // Continuous conversion mode
        SINGLE		= 0x01  // Power-down single-shot mode (default)
    };

    enum sampleRate
    {
        SPS_8       = 0x00, // 8 samples per second
        SPS_16      = 0x20, // 16 samples per second
        SPS_32      = 0x40, // 32 samples per second
        SPS_64      = 0x60, // 64 samples per second
        SPS_128     = 0x80, // 128 samples per second (default)
        SPS_250     = 0xA0, // 250 samples per second
        SPS_475     = 0xC0, // 475 samples per second
        SPS_860     = 0xE0  // 860 samples per second
    };

    enum alertMode
    {
        ALERT1      = 0x00, // Assert ALERT/RDY after one conversions
        ALERT2      = 0x01, // Assert ALERT/RDY after two conversions
        ALERT4      = 0x02, // Assert ALERT/RDY after four conversions
        NONE        = 0x03  // Disable the comparator and put ALERT/RDY in high state (default)
    };

public:
    // Controller type
    static const char TYPE[];

private:
    static const unsigned char DEVICE_IDS[];

private:
     // I2C bus ID where ADS1115 is attached
    int m_i2cBus;

    // I2C device handles
    int m_i2c[MAX_DEVICES];

    // Whether ADS was initialized
    bool m_initialized;

    // Number of chained ADS1115 devices up to MAX_DEVICES
    int m_devices;

    // Gain mode
    gainMultiplier m_gain;

    // Gain coefficient
    double m_coefficient;

    // Reference mode
    bool m_differential;

    // Sample mode
    sampleMode m_sampleMode;

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

    // Load controller settings
    virtual void deserialize(ros::NodeHandle node);

    // Gain multiplier
    gainMultiplier getGain();
    void setGain(gainMultiplier gain);

    // Reference mode
    bool getDifferential();
    void setDifferential(bool differential);

    // Sample mode
    sampleMode getSampleMode();
    void setSampleMode(sampleMode sampleMode);

    // Sample rate
    sampleRate getSampleRate();
    void setSampleRate(sampleRate sampleRate);

private:
    double getCoefficient();
    bool configure(int device, int channel);

public:
    // Create instance
    static controller* create(const char* path);
};

} // namespace str1ker
