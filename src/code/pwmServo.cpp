/*
                                                                                     @@@@@@@                  
 @@@@@@@@@@@@  @@@@@@@@@@@@   @@@@@@@@@@@@       @  @@@@@@@@@@@@@  @           @  @@@       @@@  @@@@@@@@@@@@ 
@              @ @           @            @    @ @  @              @        @@@      @@@@@@@    @            @
 @@@@@@@@@@@@  @   @         @@@@@@@@@@@@@   @   @   @             @   @@@@@      @@@       @@@ @@@@@@@@@@@@@ 
             @ @     @       @            @      @    @@           @@@@      @                  @            @
 @@@@@@@@@@@@  @       @     @            @      @      @@@@@@@@@  @          @   @@@       @@@ @            @
                                                                                     @@@@@@@                  
 pwmServo.cpp

 PWM servo controller implementation
 Created 1/19/2021

 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <math.h>
#include <pigpio.h>
#include <ros/ros.h>
#include "pwmServo.h"
#include "actuatorFactory.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace std;
using namespace str1ker;

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const char pwmServo::TYPE[] = "servo";

/*----------------------------------------------------------*\
| pwmServo implementation
\*----------------------------------------------------------*/

REGISTER_ACTUATOR(pwmServo)

pwmServo::pwmServo(const char* path, int lpwm, int rpwm, int time) :
    servo(path),
    m_lpwm(lpwm),
    m_rpwm(rpwm),
    m_pos(0.0)
{
}

pwmServo::pwmServo(const char* path):
    servo(path),
    m_lpwm(0),
    m_rpwm(0),
    m_pos(0.0)
{
}

const char* pwmServo::getType()
{
    return pwmServo::TYPE;
}

bool pwmServo::init()
{
    if (!m_enable) return true;

    gpioSetMode(m_lpwm, PI_OUTPUT);
    gpioSetMode(m_rpwm, PI_OUTPUT);

    ROS_INFO("  initialized %s %s on %d (RPWM forward) and %d (LPWM reverse)",
        getPath(), getType(), m_rpwm, m_lpwm);

    return true;
}

double pwmServo::getPos()
{
    return m_pos;
}

void pwmServo::rotate(double delta)
{
    if (!m_enable) return;

    bool forward = delta >= 0;

    gpioPWM(m_rpwm, forward ? MAX_SPEED : 0);
    gpioPWM(m_lpwm, forward ? 0 : MAX_SPEED);

    // todo: <unistd.h> usleep(microseconds)
    sleep(int(m_time * abs(delta)));

    gpioPWM(m_rpwm, 0);
    gpioPWM(m_lpwm, 0);

    m_pos = m_pos + delta;

    if (m_pos > 1.0)
        m_pos = 1.0;
    else if (m_pos < 0.0)
        m_pos = 0.0;

    sleep(1);
}

void pwmServo::deserialize()
{
    servo::deserialize();

    ros::param::get(getComponentPath("lpwm"), m_lpwm);
    ros::param::get(getComponentPath("rpwm"), m_rpwm);
    ros::param::get(getComponentPath("time"), m_time);
}

actuator* pwmServo::create(const char* path)
{
    return new pwmServo(path);
}
