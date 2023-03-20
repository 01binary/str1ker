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
#include <stdio.h>

#define ADDRESS 0

/*----------------------------------------------------------*\
| Declarations
\*----------------------------------------------------------*/

void pwmCallback(const str1ker::Pwm& msg);

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const char TOPIC[] = "robot/pwm%d";
const int QUEUE_SIZE = 16;
const int PINS[] =
{
  // Waveform channels

  9,  // Channel 0 - Digital 9
  10, // Channel 1 - Digital 10
  11, // Channel 2 - Digital 11
  3,  // Channel 3 - SCL (also Digital)

  // Binary channels

  A0, // Channel 4 - Analog 0
  A1, // Channel 5 - Analog 1
  A2, // Channel 6 - Analog 2
  2,  // Channel 7 - SDA
};

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

ros::NodeHandle node;
ros::Subscriber<str1ker::Pwm>* pSub;

/*----------------------------------------------------------*\
| Initialize Node
\*----------------------------------------------------------*/

void setup()
{
  for (int n = 0; n < sizeof(PINS) / sizeof(int); n++)
  {
    pinMode(PINS[n], OUTPUT);
  }

  char topic[32] = {0};
  sprintf(topic, TOPIC, ADDRESS);
  if (!ADDRESS) topic[strlen(topic) - 1] = 0;
  
  pSub = new ros::Subscriber<str1ker::Pwm>(topic, pwmCallback);
  
  node.initNode();
  node.subscribe(*pSub);
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

    if (request.channel >= sizeof(PINS) / sizeof(int)) continue;

    if (request.mode == str1ker::PwmChannel::MODE_ANALOG)
    {
      analogWrite(PINS[request.channel], request.value);
    }
    else if (request.mode == str1ker::PwmChannel::MODE_DIGITAL)
    {
      digitalWrite(PINS[request.channel], request.value);
    }

    if (request.duration > 0)
    {
      int inverse = request.value ? 0 : 255;

      delay(request.duration);

      if (request.mode == str1ker::PwmChannel::MODE_ANALOG)
      {
        analogWrite(PINS[request.channel], inverse);
      }
      else if (request.mode == str1ker::PwmChannel::MODE_DIGITAL)
      {
        digitalWrite(PINS[request.channel], inverse);
      }
    }
  }
}
