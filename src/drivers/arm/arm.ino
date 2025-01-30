
/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 arm.ino
 Arm Driver Board
 Copyright (C) 2025 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros.h>
#include <ros/time.h>
#include "freertos/FreeRTOS.h"
#include "interface.h"
#include "motor.h"
#include "encoder..h"
#include "pid.h"

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const int STARTUP_DELAY = 3000;           // Delay prevents "published too soon" errors
const int BASE_LPWM = 1;                  // Base motor left PWM pin
const int BASE_RPWM = 2;                  // Base motor right PWM pin
const int BASE_IS = 3;                    // Base motor current sense pin
const int BASE_A = 4;                     // Base quadrature encoder A pin
const int BASE_B = 5;                     // Base quadrature encoder B pin
const int BASE_CS = 6;                    // Base absolute encoder chip select pin
const int BASE_CLK = 7;                   // Base absolute encoder clock pin
const int BASE_MISO = 8;                  // Base absolute encoder MISO pin
const int BASE_MOSI = 9;                  // Base absolute encoder MOSI pin
const int SHOULDER_LPWM = 10;             // Shoulder motor left PWM pin
const int SHOULDER_RPWM = 11;             // Shoulder motor right PWM pin
const int SHOULDER_IS = 12;               // Shoulder motor current sense pin
const int SHOULDER_POT = 14;              // Shoulder motor potentiometer pin
const int ELBOW_LPWM = 13;                // Elbow motor left PWM pin
const int ELBOW_RPWM = 16;                // Elbow motor right PWM pin
const int ELBOW_IS = 11;                  // Elbow motor current sense pin
const int ELBOW_POT = 15;                 // Elbow motor potentiometer pin
const int GRIPPER_PIN = 17;               // Gripper solenoid pin

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

ros::Time time;                           // Current time
Actuator<FusionEncoder, Motor> base;      // Base hardware
Actuator<Potentiometer, Motor> shoulder;  // Shoulder hardware
Actuator<Potentiometer, Motor> elbow;     // Elbow hardware
Solenoid gripper;                         // Gripper hardware

/*----------------------------------------------------------*\
| Forward Declarations
\*----------------------------------------------------------*/

void initializeHardware();
void initializeControllers();
void motorControl();

/*----------------------------------------------------------*\
| Functions
\*----------------------------------------------------------*/

void init()
{
  delay(STARTUP_DELAY);

  readSettings();

  initializeRosInterface();
  initializeHardware();
  initializeControllers();
}

void loop()
{
  if (millis() > STARTUP_DELAY)
  {
    stateFeedback();
    node.spinOnce();
  }
}

void initializeHardware()
{
  // Base
  base.motor.initialize(BASE_LPWM, BASE_RPWM, BASE_IS);
  base.encoder.initialize(BASE_CS, BASE_CLK, BASE_MISO, BASE_MOSI, BASE_A, BASE_B);
  base.readSettings();

  // Shoulder
  shoulder.motor.initialize(SHOULDER_LPWM, SHOULDER_RPWM, SHOULDER_IS);
  shoulder.encoder.initialize(SHOULDER_POT);
  shoulder.readSettings()

  // Elbow
  elbow.motor.initialize(ELBOW_LPWM, ELBOW_RPWM, ELBOW_IS);
  elbow.encoder.initialize(ELBOW_POT);
  elbow.readSettings()

  // Gripper
  gripper.initialize(GRIPPER_PIN);
}

void initializeControllers()
{
  xTaskCreate(
    motorControl,
    "MotorControl",
    2048,
    NULL,
    configMAX_PRIORITIES - 1,
    NULL
  );
}

ros::Time getTime()
{
  uint32_t ms = millis();
  uint32_t sec = ms / 1000;
  uint32_t ns = (ms - sec * 1000) * 1000000;

  return ros::Time(sec, ns);
}

void motorControl()
{
  while (true)
  {
    ros::Time now = getTime();
    double dt = (now - time).toSec();
    time = now;

    base.update();
    shoulder.update();
    elbow.update();
  }
}
