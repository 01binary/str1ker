/*
                                                                                     @@@@@@@                  
 @@@@@@@@@@@@  @@@@@@@@@@@@   @@@@@@@@@@@@       @  @@@@@@@@@@@@@  @           @  @@@       @@@  @@@@@@@@@@@@ 
@              @ @           @            @    @ @  @              @        @@@      @@@@@@@    @            @
 @@@@@@@@@@@@  @   @         @@@@@@@@@@@@@   @   @   @             @   @@@@@      @@@       @@@ @@@@@@@@@@@@@ 
             @ @     @       @            @      @    @@           @@@@      @                  @            @
 @@@@@@@@@@@@  @       @     @            @      @      @@@@@@@@@  @          @   @@@       @@@ @            @
                                                                                     @@@@@@@                  
 solenoid.cpp

 Solenoid implementation
 Created 1/19/2021

 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <unistd.h>
#include <pigpio.h>
#include <ros/ros.h>
#include "solenoid.h"
#include "actuatorFactory.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace str1ker;

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const char solenoid::TYPE[] = "solenoid";

/*----------------------------------------------------------*\
| solenoid implementation
\*----------------------------------------------------------*/

REGISTER_ACTUATOR(solenoid)

solenoid::solenoid(const char* path) :
    actuator(path),
    m_output(0)
{
}

solenoid::solenoid(const char* path, int output) :
    actuator(path),
    m_output(output)
{
}

const char* solenoid::getType()
{
    return solenoid::TYPE;
}

bool solenoid::init()
{
    if (!m_enable) return true;
    
    gpioSetMode(m_output, PI_OUTPUT);

    ROS_INFO("  initialized %s %s", getPath(), getType());

    return true;
}

void solenoid::trigger(double durationSeconds)
{
    if (!m_enable) return;

    useconds_t durationMicroseconds = durationSeconds / 1000000.0;

    gpioWrite(m_output, 1);
    usleep(durationMicroseconds);

    gpioWrite(m_output, 0);
    usleep(durationMicroseconds);
}

void solenoid::deserialize()
{
    actuator::deserialize();

    ros::param::get(getComponentPath("output"), m_output);
}

actuator* solenoid::create(const char* path)
{
    return new solenoid(path);
}
