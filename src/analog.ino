/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 analog.ino

 Arduino Analog Controller
 Created 3/22/2023

 Copyright (C) 2023 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#define USE_USBCON

#include <Wire.h>                     // Serial communication
#include <Arduino.h>                  // ROS arduino node
#include <ros.h>                      // ROS publish/subscribe
#include <Adafruit_PWMServoDriver.h>  // Analog write library
#include <str1ker/Adc.h>              // Analog read request
#include <str1ker/Pwm.h>              // Analog write request
#include <str1ker/PwmChannel.h>       // Channel value

/*----------------------------------------------------------*\
| Declarations
\*----------------------------------------------------------*/

void writePwm(const str1ker::Pwm& msg);

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const char TOPIC[] = "robot/analog";
const int PWM_CHANNELS = 16;
const int PWM_FREQ_HZ = 700;
const double PWM_OUTPUT_MAX = 4096.0;
const double PWM_INPUT_MAX = 255.0;
const int ANALOG_CHANNELS = 12;
const int RATE_HZ = 6;
const int DELAY = int(1.0 / RATE_HZ);
const int ANALOG_PINS[] =
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

// ADC publisher
str1ker::Adc msg;
uint16_t adc[ANALOG_CHANNELS] = {0};
ros::Publisher pub(TOPIC, &msg);

// PWM subscriber
ros::Subscriber<str1ker::Pwm> sub(TOPIC, writePwm);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
ros::NodeHandle node;

/*----------------------------------------------------------*\
| Initialization
\*----------------------------------------------------------*/

void initAdc()
{
  for (int channel = 0; channel < ANALOG_CHANNELS; channel++)
  {
    pinMode(ANALOG_PINS[channel], INPUT_PULLUP);
  }

  node.advertise(pub);
}

void initPwm()
{
  pwm.setPWMFreq(PWM_FREQ_HZ);
  pwm.begin();

  node.subscribe(sub);
}

void setup()
{
  node.initNode();
  initAdc();
  initPwm();
}

/*----------------------------------------------------------*\
| Analog input
\*----------------------------------------------------------*/

void readAdc()
{
  for (int channel = 0; channel < ANALOG_CHANNELS; channel++)
  {
    adc[channel] = (uint16_t)analogRead(ANALOG_PINS[channel]);
  }

  msg.adc_length = ANALOG_CHANNELS;
  msg.adc = adc;
    
  pub.publish(&msg);
}

/*----------------------------------------------------------*\
| Analog output helper
\*----------------------------------------------------------*/

void analog(int channel, double value)
{
  pwm.setPWM(channel, int((1.0 - value) * PWM_OUTPUT_MAX), int(value * PWM_OUTPUT_MAX));
}

/*----------------------------------------------------------*\
| Digital output helper
\*----------------------------------------------------------*/

void digital(int channel, bool value)
{
  if (value)
    pwm.setPWM(channel, int(PWM_OUTPUT_MAX), 0);
  else
    pwm.setPWM(channel, 0, int(PWM_OUTPUT_MAX));
}

/*----------------------------------------------------------*\
| Analog output
\*----------------------------------------------------------*/

void writePwm(const str1ker::Pwm& msg)
{
  for (uint32_t n = 0; n < msg.channels_length; n++)
  {
    str1ker::PwmChannel& request = msg.channels[n];
    if (request.channel >= PWM_CHANNELS) continue;

    if (request.mode == str1ker::PwmChannel::MODE_ANALOG)
    {
      analog(request.channel, request.value / PWM_INPUT_MAX);
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

/*----------------------------------------------------------*\
| Message handling
\*----------------------------------------------------------*/

void loop()
{
  readAdc();
  node.spinOnce();
  delay(DELAY);
}
