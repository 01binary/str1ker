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

#define USE_USBCON

#include <Arduino.h>
#include <ros.h>
#include <str1ker/Adc.h>

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

ros::NodeHandle node;
str1ker::Adc msg;
uint16_t* adc = &msg.adc0;
ros::Publisher pub("robot/adc", &msg);

void setup()
{ 
  for (int channel = 0; channel < CHANNELS; channel++)
    pinMode(PINS[channel], INPUT_PULLUP);
    
  node.initNode();
  node.advertise(pub);
}

void loop()
{
  for (int channel = 0; channel < CHANNELS; channel++)
  {
    adc[channel] = (uint16_t)analogRead(PINS[channel]);
  }
    
  pub.publish(&msg);
  node.spinOnce();

  delay(250);
}
