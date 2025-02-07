
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
#include <EEPROMex.h>
#include <Arduino_FreeRTOS.h>
#include "actuator.h"
#include "interface.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const int RATE_HZ = 50;                   // Default real time update rate
const int STARTUP_DELAY = 3000;           // Prevent "published too soon" errors
const int MEMBASE = 350;                  // Flash storage offset
const int MEMSIZE = 4096 - MEMBASE;       // Flash storage size
const int MAX_WRITES = 250;               // Max flash storage writes
const int RT = configMAX_PRIORITIES - 1;  // Real time thread priority

const int BASE_LPWM = 11;                 // Base motor left PWM pin
const int BASE_RPWM = 3;                  // Base motor right PWM pin
const int BASE_IS = A2;                   // Base motor current sense pin
const int BASE_A = 18;                    // Base quadrature encoder A pin
const int BASE_B = 19;                    // Base quadrature encoder B pin
const int BASE_CS = 53;                   // Base absolute encoder chip select pin

const int SHOULDER_LPWM = 6;              // Shoulder motor left PWM pin
const int SHOULDER_RPWM = 5;              // Shoulder motor right PWM pin
const int SHOULDER_IS = A3;               // Shoulder motor current sense pin
const int SHOULDER_POT = A0;              // Shoulder motor potentiometer pin

const int ELBOW_LPWM = 10;                // Elbow motor left PWM pin
const int ELBOW_RPWM = 9;                 // Elbow motor right PWM pin
const int ELBOW_IS = A4;                  // Elbow motor current sense pin
const int ELBOW_POT = A1;                 // Elbow motor potentiometer pin

const int GRIPPER_PIN = 13;               // Gripper solenoid pin

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

int rateHz;                               // Real time update rate
Actuator<FusionEncoder, Motor> base;      // Base hardware
Actuator<Potentiometer, Motor> shoulder;  // Shoulder hardware
Actuator<Potentiometer, Motor> elbow;     // Elbow hardware
Solenoid gripper;                         // Gripper hardware

/*----------------------------------------------------------*\
| Forward Declarations
\*----------------------------------------------------------*/

void readSettings();
void writeSettings();
void realTimeMotorControl();

/*----------------------------------------------------------*\
| Functions
\*----------------------------------------------------------*/

void init()
{
  initializeRosInterface();
  readSettings();

  base.motor.initialize(BASE_LPWM, BASE_RPWM, BASE_IS);
  base.encoder.initialize(BASE_CS, BASE_A, BASE_B);
  base.readSettings();

  shoulder.motor.initialize(SHOULDER_LPWM, SHOULDER_RPWM, SHOULDER_IS);
  shoulder.encoder.initialize(SHOULDER_POT);
  shoulder.readSettings()

  elbow.motor.initialize(ELBOW_LPWM, ELBOW_RPWM, ELBOW_IS);
  elbow.encoder.initialize(ELBOW_POT);
  elbow.readSettings()

  gripper.initialize(GRIPPER_PIN);

  xTaskCreate(realTimeMotorControl, "motor", 2048, NULL, RT, NULL);
}

void loop()
{
  if (millis() > STARTUP_DELAY)
  {
    stateFeedback();
    node.spinOnce();
  }
}

void readSettings()
{
  EEPROM.setMemPool(MEMBASE, MEMSIZE);
  EEPROM.setMaxAllowedWrites(MAX_WRITES);

  rateHz = EEPROM.readInt(EEPROM.getAddress(sizeof(int)));
}

void writeSettings()
{
  EEPROM.writeInt(EEPROM.getAddress(sizeof(int)), rateHz);
}

ros::Time getTime()
{
  uint32_t ms = millis();
  uint32_t sec = ms / 1000;
  uint32_t ns = (ms - sec * 1000) * 1000000;

  return ros::Time(sec, ns);
}

void realTimeMotorControl()
{
  ros::Time time = getTime();
  const TickType_t frequency = pdMS_TO_TICKS(int(1000.0 / rateHz));
  TickType_t lastTick = xTaskGetTickCount();

  while (true)
  {
    ros::Time now = getTime();
    double timeStep = (now - time).toSec();
    time = now;

    base.update(timeStep);
    shoulder.update(timeStep);
    elbow.update(timeStep);

    vTaskDelayUntil(&lastTick, frequency);
  }
}
