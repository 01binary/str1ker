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

#include <ros.h>                // ROS communication
#include <str1ker/Adc.h>        // ADC/quadrature input request
#include <str1ker/Pwm.h>        // PWM/digital output request
#include <QuadratureEncoder.h>  // QuadratureEncoder library

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

//
// ROS interface
//

const char ADC_TOPIC[] = "adc";
const char PWM_TOPIC[] = "pwm";
const double RATE_HZ = 50.0;
const int DELAY = 1000.0 / RATE_HZ;

//
// PWM outputs
//

const int PWM_CHANNELS = 7;
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

//
// ADC inputs
//

const int ADC_CHANNELS = 8;
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

//
// Quadrature inputs
//

const int QUADRATURE_CHANNELS = 1;
const int QUADRATURE_PINS[][2] =
{
  { 2, 7 }
};

/*----------------------------------------------------------*\
| Declarations
\*----------------------------------------------------------*/

void setup();
void read();
void write(const str1ker::Pwm& msg);
void loop();

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

// Absolute and relative encoder readings
int16_t readings[ADC_CHANNELS + QUADRATURE_CHANNELS] = {0};

// Relative encoders
Encoders** encoders;

// ADC publisher
str1ker::Adc msg;
ros::Publisher pub(ADC_TOPIC, &msg);

// PWM subscriber
ros::Subscriber<str1ker::Pwm> sub(PWM_TOPIC, write);

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

void initQuadrature()
{
  encoders = new Encoders*[QUADRATURE_CHANNELS];
 
  for (int channel = 0; channel < QUADRATURE_CHANNELS; channel++)
  {
    encoders[channel] = new Encoders(
      QUADRATURE_PINS[channel][0], QUADRATURE_PINS[channel][1]);

    readings[ADC_CHANNELS + channel] = 0;
  }
}

void setup()
{
  // Initialize ROS interface
  node.initNode();
  node.negotiateTopics();

  // Initialize inputs and outputs
  initAdc();
  initPwm();
  initQuadrature();
}

/*----------------------------------------------------------*\
| Absolute encoder input
\*----------------------------------------------------------*/

void readAdc()
{
  for (int channel = 0; channel < ADC_CHANNELS; channel++)
  {
    readings[channel] = (int16_t)analogRead(ADC_PINS[channel]);
  }
}

/*----------------------------------------------------------*\
| Relative encoder input
\*----------------------------------------------------------*/

void readQuadrature()
{
  for (int channel = 0; channel < QUADRATURE_CHANNELS; channel++)
  {
    readings[ADC_CHANNELS + channel] = encoders[channel]->getEncoderCount();
  }
}

/*----------------------------------------------------------*\
| Input
\*----------------------------------------------------------*/

void read()
{
  readAdc();
  readQuadrature();

  msg.adc_length = ADC_CHANNELS + QUADRATURE_CHANNELS;
  msg.adc = readings;

  pub.publish(&msg);
}

/*----------------------------------------------------------*\
| Output
\*----------------------------------------------------------*/

void write(const str1ker::Pwm& msg)
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
  read();
  node.spinOnce();
  delay(DELAY);
}
