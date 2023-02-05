/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 arduinoMicro.cpp

 Arduino Micro as Analog to Digital Converter
 Created 11/14/2022

 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <pigpiod_if2.h>
#include "robot.h"
#include "arduinoMicro.h"
#include "controllerFactory.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace str1ker;
using namespace std;

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const char arduinoMicro::TYPE[] = "arduinoMicro";
const unsigned char SIGNATURE[] = { 'a', 'd', 'c', 'd' };

/*----------------------------------------------------------*\
| Types
\*----------------------------------------------------------*/

struct SAMPLE
{
  unsigned char signature[sizeof(SIGNATURE)];
  uint16_t readings[arduinoMicro::CHANNELS];
};

/*----------------------------------------------------------*\
| arduinoMicro implementation
\*----------------------------------------------------------*/

REGISTER_SINGLETON(arduinoMicro)

arduinoMicro::arduinoMicro(robot& robot, const char* path) :
    adc(robot, path),
    m_usbHandle(-1)
{
  memset(m_lastSample, 0, sizeof(m_lastSample));
}

controller* arduinoMicro::create(robot& robot, const char* path)
{
    return new arduinoMicro(robot, path);
}

const char* arduinoMicro::getType()
{
    return arduinoMicro::TYPE;
}

bool arduinoMicro::init()
{
    if (!m_enable || m_usbHandle >= 0) return true;

    if (!m_device.length()) {
      return false;
    }

    m_usbHandle = serial_open(m_robot.getGpio(), (char*)m_device.c_str(), arduinoMicro::BAUD_RATE, 0);

    if (m_usbHandle < 0)
    {
        ROS_ERROR("failed to initialize Arduino Micro as ADC on %s", m_device.c_str());
        return false;
    }

    ROS_INFO("  initialized %s %s on %s", getPath(), getType(), m_device.c_str());

    return true;
}

int arduinoMicro::getValue(int channel)
{
    if (!m_enable) return 0;
    return m_lastSample[channel];
}

int arduinoMicro::getMaxValue()
{
    return numeric_limits<uint16_t>::max();
}

int arduinoMicro::getChannels()
{
    return CHANNELS;
}

void arduinoMicro::deserialize(ros::NodeHandle node)
{
    controller::deserialize(node);
    ros::param::get(getControllerPath("dev"), m_device);
}

void arduinoMicro::publish()
{
    if (!m_enable) return;

    SAMPLE sample;

    if (serial_data_available(m_robot.getGpio(), m_usbHandle) < sizeof(SAMPLE))
    {
      // Not enough data to read
      return;
    }

    if (serial_read(m_robot.getGpio(), m_usbHandle, (char*)&sample, sizeof(SAMPLE)) != sizeof(SAMPLE))
    {
      // Failed to read
      setLastError("failed to read from serial port");
      return;
    }

    if (memcmp(sample.signature, SIGNATURE, sizeof(SIGNATURE)) != 0)
    {
      // Invalid or corrupted data
      setLastError("corrupted payload read from serial port");
      return;
    }

    std_msgs::MultiArrayDimension dim;
    std_msgs::UInt16MultiArray msg;

    dim.size = CHANNELS;
    dim.stride = 1;
    dim.label = "channels";
    msg.layout.dim.push_back(dim);
    msg.data.reserve(dim.size);

    for (int n = 0; n < CHANNELS; n++)
    {
      m_lastSample[n] = sample.readings[n];
      msg.data.push_back(sample.readings[n]);
    }

    m_pub.publish(msg);

    setLastError(NULL);
}
