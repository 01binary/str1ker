/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 pwm.ino

 Arduino PWM Node
 Created 3/12/2023

 Copyright (C) 2023 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#define USE_USBCON

#include <Wire.h>
#include <Arduino.h>
#include <ros.h>
#include <Adafruit_PWMServoDriver.h>
#include <str1ker/Pwm.h>
#include <str1ker/PwmChannel.h>

/*----------------------------------------------------------*\
| Declarations
\*----------------------------------------------------------*/

void pwmCallback(const str1ker::Pwm& msg);

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const char TOPIC[] = "robot/pwm";
const double PWM_MAX = 4096.0;
const double CHANNEL_MAX = 255.0;
const int CHANNELS = 16;
const int QUEUE_SIZE = 32;

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

ros::NodeHandle node;
ros::Subscriber<str1ker::Pwm> sub(TOPIC, pwmCallback);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

/*----------------------------------------------------------*\
| Initialize Node
\*----------------------------------------------------------*/

void setup()
{
  pwm.setPWMFreq(700);
  pwm.begin();
  
  node.initNode();
  node.subscribe(sub);
}

/*----------------------------------------------------------*\
| Run Node
\*----------------------------------------------------------*/

void loop()
{
  node.spinOnce();
  delay(10);
}

/*----------------------------------------------------------*\
| Analog output
\*----------------------------------------------------------*/

void analog(int channel, double value)
{
  pwm.setPWM(channel, int((1.0 - value) * PWM_MAX), int(value * PWM_MAX));
}

/*----------------------------------------------------------*\
| Digital output
\*----------------------------------------------------------*/

void digital(int channel, bool value)
{
  if (value)
    pwm.setPWM(channel, int(PWM_MAX), 0);
  else
    pwm.setPWM(channel, 0, int(PWM_MAX));
}

/*----------------------------------------------------------*\
| Handle PWM requests
\*----------------------------------------------------------*/

void pwmCallback(const str1ker::Pwm& msg)
{
  for (uint32_t n = 0; n < msg.channels_length; n++)
  {
    str1ker::PwmChannel& request = msg.channels[n];
    if (request.channel >= CHANNELS) continue;

    if (request.mode == str1ker::PwmChannel::MODE_ANALOG)
    {
      analog(request.channel, request.value / CHANNEL_MAX);
    }
    else
    {
      digital(request.channel, bool(request.value));
    }

    if (request.duration > 0)
    {
      delay(request.duration);
      digital(request.channel, request.value ? false : true);
    }
  }
}
