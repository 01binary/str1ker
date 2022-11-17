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
    // Arduino Micro analog channels
    static const int CHANNELS = 12;

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
    arduinoMicro(const char* path);

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
};

} // namespace str1ker
