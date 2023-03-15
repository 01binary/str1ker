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

#include <Arduino.h>
#include <ros.h>
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
const int QUEUE_SIZE = 16;
const int ADDRESS = 0;
const int PINS[] =
{
  9,  // Channel 0 - Digital 9
  10, // Channel 1 - Digital 10
  11, // Channel 2 - Digital 11
  A0, // Channel 3 - Analog 0
  A1, // Channel 4 - Analog 1
  A2  // Channel 5 - Analog 2
};

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

ros::NodeHandle node;
ros::Subscriber<str1ker::Pwm> sub("robot/pwm", pwmCallback);

/*----------------------------------------------------------*\
| Initialize Node
\*----------------------------------------------------------*/

void setup()
{
  for (int n = 0; n < sizeof(PINS) / sizeof(int); n++)
  {
    pinMode(PINS[n], OUTPUT);
  }
  
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
| Handle PWM requests
\*----------------------------------------------------------*/

void pwmCallback(const str1ker::Pwm& msg)
{
  for (uint32_t n = 0; n < msg.channels_length; n++)
  {
    str1ker::PwmChannel& request = msg.channels[n];
    int pinIndex = request.channel - ADDRESS;

    if (pinIndex < 0 || pinIndex > sizeof(PINS) / sizeof(int)) continue;

    if (request.mode == str1ker::PwmChannel::MODE_ANALOG)
    {
      analogWrite(PINS[pinIndex], request.value);
    }
    else if (request.mode == str1ker::PwmChannel::MODE_DIGITAL)
    {
      digitalWrite(PINS[pinIndex], request.value);
    }
  }
}
