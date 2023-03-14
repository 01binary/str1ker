/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 pwm.ino

 PWM Arduino Subscriber
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
const int CHANNELS = 6;
const uint8_t INVALID = 0xFF;
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

ros::NodeHandle  node;
ros::Subscriber<str1ker::Pwm> sub("robot/pwm", pwmCallback);


void setup()
{
  for (int channel = 0; channel < CHANNELS; channel++)
  {
    pinMode(PINS[channel], OUTPUT);
    analogWrite(PINS[channel], 0);
  }
  
  node.initNode();
  node.subscribe(sub);
}

void loop()
{
  node.spinOnce();
  delay(10);
}

void setChannel(int channel, uint8_t dutyCycle)
{
  channel -= ADDRESS;
  
  if (channel >= 0 && channel < CHANNELS)
    analogWrite(PINS[channel], dutyCycle);
}

void pwmCallback(const str1ker::Pwm& msg)
{
  if (msg.channel1 != INVALID) setChannel(msg.channel1, msg.dutyCycle1);
  if (msg.channel2 != INVALID) setChannel(msg.channel2, msg.dutyCycle2);
}