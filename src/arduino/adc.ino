/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 adc.ino

 Str1ker Drumming Robot Arduino ADC
 Created 03/09/2023

 Copyright (C) 2023 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#define USE_USBCON

#include <Arduino.h>
#include <ros.h>
#include <str1ker/Adc.h>

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const int CHANNELS = 12;

const int PINS[] =
{
  A0,   // Analog 0
  A1,   // Analog 1
  A2,   // Analog 2
  
  A3,   // Analog 3
  A4,   // Analog 4
  A5,   // Analog 5

  A6,   // Digital 4
  A7,   // Digital 6
  A8,   // Digital 8

  A9,   // Digital 9
  A10,  // Digital 10
  A11,  // Digital 12
};

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

ros::NodeHandle node;
str1ker::Adc msg;
uint16_t adc[CHANNELS] = {0};
ros::Publisher pub("robot/adc", &msg);

/*----------------------------------------------------------*\
| Initialize node
\*----------------------------------------------------------*/

void setup()
{ 
  for (int channel = 0; channel < CHANNELS; channel++)
  {
    pinMode(PINS[channel], INPUT_PULLUP);
  }
    
  node.initNode();
  node.advertise(pub);
}

/*----------------------------------------------------------*\
| Run node
\*----------------------------------------------------------*/

void loop()
{
  for (int channel = 0; channel < CHANNELS; channel++)
  {
    adc[channel] = (uint16_t)analogRead(PINS[channel]);
  }

  msg.adc_length = CHANNELS;
  msg.adc = adc;
    
  pub.publish(&msg);
  node.spinOnce();

  delay(250);
}