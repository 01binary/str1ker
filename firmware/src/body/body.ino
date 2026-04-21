/*
                                                                                     ███████
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████
 body.ino
 Main Board Firmware
 Copyright (C) 2026 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <Wire.h>
#include <firmware.h>

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const int STARTUP_DELAY = 1000;

const int BUS_CURRENT_SENSOR_PIN = A0;

const int HEAD_PAN_ENABLE = 6;
const int HEAD_PAN_STEP = 24;
const int HEAD_PAN_DIR = 25;
const int HEAD_PAN_ENCODER_STATUS = 33;
const int HEAD_PAN_CS = 0;

const int TORSO_PAN_ENABLE = 7;
const int TORSO_PAN_DIR = 20;
const int TORSO_PAN_PWM = 21;
const int TORSO_PAN_ENCODER_STATUS = 8;
const int TORSO_PAN_CS = 1;

const int HEAD_TILT_POT = A1;
const int HEAD_TILT_MOTOR_A_EN = 27;
const int HEAD_TILT_MOTOR_A_LPWM = 2;
const int HEAD_TILT_MOTOR_A_RPWM = 3;
const int HEAD_TILT_MOTOR_B_EN = 28;
const int HEAD_TILT_MOTOR_B_LPWM = 4;
const int HEAD_TILT_MOTOR_B_RPWM = 5;

const int TORSO_TILT_POT_A = A2;
const int TORSO_TILT_POT_B = A3;
const int TORSO_TILT_MOTOR_A_EN = 29;
const int TORSO_TILT_MOTOR_A_LPWM = 9;
const int TORSO_TILT_MOTOR_A_RPWM = 10;
const int TORSO_TILT_MOTOR_B_EN = 30;
const int TORSO_TILT_MOTOR_B_LPWM = 22;
const int TORSO_TILT_MOTOR_B_RPWM = 23;

const int MOUTH_SERVO_PIN = 26;

/*----------------------------------------------------------*\
| Forward Declarations
\*----------------------------------------------------------*/

void initializeBoard();
void initializeDevices();

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

StepperMotor headPanMotor;
AbsoluteEncoder headPanEncoder;
TeknicMotor torsoPanMotor;
AbsoluteEncoder torsoPanEncoder;
ServoMotor mouthServo;
CurrentSensor busCurrentSensor;
VoltageCurrentSensor busVoltageCurrentSensor;

/*----------------------------------------------------------*\
| Entry Points
\*----------------------------------------------------------*/

void setup()
{
  initializeBoard();
  initializeDevices();
  delay(STARTUP_DELAY);
}

void loop()
{
}

/*----------------------------------------------------------*\
| Functions
\*----------------------------------------------------------*/

void initializeBoard()
{
  analogReadResolution(12);
  analogReadAveraging(8);

  Wire.begin();
  Wire.setClock(400000);
}

void initializeDevices()
{
  headPanMotor.initialize(HEAD_PAN_ENABLE, HEAD_PAN_STEP, HEAD_PAN_DIR);
  headPanEncoder.initialize(HEAD_PAN_ENCODER_STATUS, HEAD_PAN_CS);

  torsoPanMotor.initialize(TORSO_PAN_ENABLE, TORSO_PAN_DIR, TORSO_PAN_PWM);
  torsoPanEncoder.initialize(TORSO_PAN_ENCODER_STATUS, TORSO_PAN_CS);

  mouthServo.initialize(MOUTH_SERVO_PIN);

  busCurrentSensor.initialize(BUS_CURRENT_SENSOR_PIN);
  busVoltageCurrentSensor.initialize();

  // The remaining head-tilt and torso-tilt devices are intentionally left at the
  // pin-definition stage until the ROS interface and control strategy are finalized.
}
