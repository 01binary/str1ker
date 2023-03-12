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
const int CHANNELS = 2;
const int LPWM = 0;
const int RPWM = 1;
const int PINS[][2] =
{
  // LPWM, RPWM
  { 3, 9 },
  { 10, 11 }
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
    pinMode(PINS[channel][LPWM], OUTPUT);
    pinMode(PINS[channel][RPWM], OUTPUT);
  }
  
  node.initNode();
  node.subscribe(sub);
}

void loop()
{
  node.spinOnce();
  delay(10);
}

void pwmCallback(const str1ker::Pwm& msg)
{
  int channel = msg.channel - ADDRESS;
  if (channel < 0 || channel > CHANNELS) return;

  analogWrite(PINS[channel][RPWM], msg.direction ? msg.dutyCycle : 0);
  analogWrite(PINS[channel][LPWM], msg.direction ? 0 : msg.dutyCycle);
}
