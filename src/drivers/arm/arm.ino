
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
#include "interface.h"
#include "actuator.h"
#include "motor.h"
#include "encoder.h"
#include "solenoid.h"
#include "pid.h"

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const int RATE_HZ = 50;                   // Default real time update rate
const int STARTUP_DELAY = 3000;           // Prevent "published too soon" errors
const float MS_TO_SECONDS = 0.001f;       // Milliseconds to seconds conversion factor

const int BASE_LPWM = 11;                 // Base motor left PWM pin
const int BASE_RPWM = 3;                  // Base motor right PWM pin
const int BASE_SENSE = A2;                // Base motor current sense pin
const int BASE_A = 18;                    // Base quadrature encoder A pin
const int BASE_B = 19;                    // Base quadrature encoder B pin
const int BASE_CS = 53;                   // Base absolute encoder chip select pin

const int SHOULDER_LPWM = 6;              // Shoulder motor left PWM pin
const int SHOULDER_RPWM = 5;              // Shoulder motor right PWM pin
const int SHOULDER_SENSE = A3;            // Shoulder motor current sense pin
const int SHOULDER_POTENTIOMETER = A0;    // Shoulder motor potentiometer pin

const int ELBOW_LPWM = 10;                // Elbow motor left PWM pin
const int ELBOW_RPWM = 9;                 // Elbow motor right PWM pin
const int ELBOW_SENSE = A4;               // Elbow motor current sense pin
const int ELBOW_POTENTIOMETER = A1;       // Elbow motor potentiometer pin

const int GRIPPER_PIN = 13;               // Gripper solenoid pin

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

Actuator<FusionEncoder, Motor> base;      // Base hardware
Actuator<Potentiometer, Motor> shoulder;  // Shoulder hardware
Actuator<Potentiometer, Motor> elbow;     // Elbow hardware
Solenoid gripper;                         // Gripper hardware
uint32_t lastTime = 0;

/*----------------------------------------------------------*\
| Functions
\*----------------------------------------------------------*/

void setup()
{
  base.motor.initialize(BASE_LPWM, BASE_RPWM, BASE_SENSE);
  base.encoder.initialize(BASE_CS, BASE_A, BASE_B);
  shoulder.motor.initialize(SHOULDER_LPWM, SHOULDER_RPWM, SHOULDER_SENSE);
  shoulder.encoder.initialize(SHOULDER_POTENTIOMETER);
  elbow.motor.initialize(ELBOW_LPWM, ELBOW_RPWM, ELBOW_SENSE);
  elbow.encoder.initialize(ELBOW_POTENTIOMETER);
  gripper.initialize(GRIPPER_PIN);

  ros::NodeHandle& node = initializeRos();

  base.loadSettings(node, "base");
  shoulder.loadSettings(node, "shoulder");
  elbow.loadSettings(node, "elbow");
}

void loop()
{
  uint32_t now = millis();
  const uint32_t periodMs = 1000 / RATE_HZ;

  if (now < STARTUP_DELAY)
  {
    lastTime = now;
    node.spinOnce();
    return;
  }

  uint32_t elapsedMs = now - lastTime;

  if (elapsedMs < periodMs)
  {
    node.spinOnce();
    return;
  }

  float timeStep = elapsedMs * MS_TO_SECONDS;

  if (timeStep <= 0.0f)
  {
    node.spinOnce();
    return;
  }

  base.update(timeStep);
  shoulder.update(timeStep);
  elbow.update(timeStep);

  stateFeedback();

  node.spinOnce();
  lastTime = now;
}
