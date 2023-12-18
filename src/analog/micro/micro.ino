/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 micro.ino

 Arduino Micro Analog Controller
 Created 3/22/2023

 Copyright (C) 2023 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/


/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#define USE_USBCON

#include <ros.h>            // ROS communication
#include <str1ker/Adc.h>    // Analog read request
#include <str1ker/Pwm.h>    // Analog write request

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

// ROS topics and spin rate
const char ADC_TOPIC[] = "adc";
const char PWM_TOPIC[] = "pwm";
const double RATE_HZ = 50.0;
const int DELAY = 1000.0 / RATE_HZ;

// Analog output
const int PWM_PINS[] =
{
  3,    // ~D3/SCL
  11,   // ~D11
  5,    // ~D5
  6,    // ~D6/A7
  9,    // ~D9/A9
  10,   // ~D10/A10
  13    // D13~
};
const int PWM_CHANNELS = sizeof(PWM_PINS) / sizeof(int);

// Analog input
const int ADC_PINS[] =
{
  A0,   // A0/D18
  A1,   // A1/D19
  A2,   // A2/D20
  A3,   // A3/D21
  A4,   // A4/D22
  A5,   // A5/D23
  A6,   // A6/D4
  A11   // D12/A11
};
const int ADC_CHANNELS = sizeof(ADC_PINS) / sizeof(int);

/*----------------------------------------------------------*\
| Declarations
\*----------------------------------------------------------*/

void writePwm(const str1ker::Pwm& msg);

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

// ADC publisher
str1ker::Adc msg;
uint16_t adc[ADC_CHANNELS] = {0};
ros::Publisher pub(ADC_TOPIC, &msg);

// PWM subscriber
ros::Subscriber<str1ker::Pwm> sub(PWM_TOPIC, writePwm);

// ROS node
ros::NodeHandle node;

/*----------------------------------------------------------*\
| Initialization
\*----------------------------------------------------------*/

void initAdc()
{
  for (int channel = 0; channel < ADC_CHANNELS; channel++)
  {
    pinMode(ADC_PINS[channel], INPUT_PULLUP);
  }

  node.advertise(pub);
}

void initPwm()
{
  for (int channel = 0; channel < PWM_CHANNELS; channel++)
  {
    pinMode(PWM_PINS[channel], OUTPUT);
  }

  node.subscribe(sub);
}

void setup()
{
  node.initNode();
  initAdc();
  initPwm();

  // Ensure the node is ready
  delay(1000);
  node.negotiateTopics();
  delay(1000);
}

/*----------------------------------------------------------*\
| Analog input
\*----------------------------------------------------------*/

void readAdc()
{
  for (int channel = 0; channel < ADC_CHANNELS; channel++)
  {
    adc[channel] = (uint16_t)analogRead(ADC_PINS[channel]);
  }

  msg.adc_length = ADC_CHANNELS;
  msg.adc = adc;
    
  pub.publish(&msg);
}

/*----------------------------------------------------------*\
| Analog output
\*----------------------------------------------------------*/

void writePwm(const str1ker::Pwm& msg)
{
  for (int n = 0; n < msg.channels_length; n++)
  {
    str1ker::PwmChannel& request = msg.channels[n];

    if (request.channel >= 0 && request.channel < PWM_CHANNELS)
    {
        if (request.mode == str1ker::PwmChannel::MODE_ANALOG)
        {
            // Write PWM waveform
            analogWrite(PWM_PINS[request.channel], request.value);
        }
        else if (request.mode == str1ker::PwmChannel::MODE_DIGITAL)
        {
            // Write high or low
            digitalWrite(PWM_PINS[request.channel], request.value ? HIGH : LOW);

            if (request.duration > 0)
            {
                // Reset after waiting if duration specified
                delay(request.duration);
                digitalWrite(PWM_PINS[request.channel], LOW);
            }
        }
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
