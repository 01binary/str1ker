/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 arduinoMicro.h

 Arduino Micro as Analog to Digital Converter
 Created 02/22/2021

 Copyright (C) 2022 Valeriy Novytskyy
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

class arduinoMicro : public adc
{
public:
    // Arduino Micro analog channels A0 - A11
    static const int CHANNELS = 12;

    // Arduino Micro analog max
    static const int SAMPLE_MAX = 1023;

    // USB serial baud rate
    static const int BAUD_RATE = 4800;

    // Message publishing queue size
    const int PUBLISH_QUEUE_SIZE = 16;

    // Controller type
    static const char TYPE[];

private:
    // USB device name, like /dev/ttyACM0
    std::string m_device;

    // USB device handle
    int m_usbHandle;

    // Publisher for last sample on each channel
    ros::Publisher m_pub;

    // Last sample recorded for each channel
    uint16_t m_lastSample[CHANNELS];

public:
    arduinoMicro(class robot& robot, const char* path);

public:
    // Create instance
    static controller* create(class robot& robot, const char* path);

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
    bool sampleReady();
    bool readSample(SAMPLE& sample);
};

} // namespace str1ker
