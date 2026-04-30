/*
                                                                                     ███████
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████
 body.ino
 Main Robot Board Firmware
 Copyright (C) 2026 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <Wire.h>       // I2C
#include <firmware.h>   // See ../readme.md

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const int STARTUP_DELAY = 1000;
const int I2C_FREQUENCY = 400000;
const unsigned long POWER_DISPLAY_UPDATE_INTERVAL = 100;

const int BUS_CURRENT_SENSOR_PIN = A0;

const int HEAD_PAN_ENABLE = 6;
const int HEAD_PAN_STEP = 24;
const int HEAD_PAN_DIR = 25;
const int HEAD_PAN_ENCODER_STATUS_REGISTER = 11; // U9.QD
const int HEAD_PAN_CS = 0;

const int TORSO_PAN_ENABLE = 7;
const int TORSO_PAN_STATUS = 8;
const int TORSO_PAN_DIR = 20;
const int TORSO_PAN_PWM = 21;
const int TORSO_PAN_ENCODER_STATUS_REGISTER = 10; // U9.QC
const int TORSO_PAN_CS = 1;

const int HEAD_TILT_POTENTIOMETER = A1;
const int HEAD_TILT_MOTOR1_EN = 27;
const int HEAD_TILT_MOTOR1_LPWM = 2;
const int HEAD_TILT_MOTOR1_RPWM = 3;
const int HEAD_TILT_MOTOR2_EN = 28;
const int HEAD_TILT_MOTOR2_LPWM = 4;
const int HEAD_TILT_MOTOR2_RPWM = 5;

const int TORSO_TILT_POTENTIOMETER1 = A2;
const int TORSO_TILT_POTENTIOMETER2 = A3;
const int TORSO_TILT_MOTOR1_EN = 29;
const int TORSO_TILT_MOTOR1_LPWM = 9;
const int TORSO_TILT_MOTOR1_RPWM = 10;
const int TORSO_TILT_MOTOR2_EN = 30;
const int TORSO_TILT_MOTOR2_LPWM = 22;
const int TORSO_TILT_MOTOR2_RPWM = 23;

const int MOUTH_SERVO_SIG = 26;

const int BATTERY_METER_LEVELS = 10;
const int BATTERY_METER_REGISTERS[BATTERY_METER_LEVELS] =
{
  0, // U8.QA
  1, // U8.QB
  2, // U8.QC
  3, // U8.QD
  4, // U8.QE
  5, // U8.QF
  6, // U8.QG
  7, // U8.QH
  8, // U9.QA
  9  // U9.QB
};

const int SHIFT_REGISTER_DATA_PIN = 11;
const int SHIFT_REGISTER_CLOCK_PIN = 13;
const int SHIFT_REGISTER_LATCH_PIN = 31;
const int SHIFT_REGISTER_COUNT = 2;
const int SHIFT_REGISTER_OUTPUT_COUNT = 16;

/*----------------------------------------------------------*\
| Forward Declarations
\*----------------------------------------------------------*/

void initializeSerial();
void initializeADC();
void initializeI2C();
void updatePowerDisplay();
void writeRegister(int id, bool value);

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

StepperMotor headPanMotor;
AbsoluteEncoder headPanEncoder;
TeknicMotor torsoPanMotor;
AbsoluteEncoder torsoPanEncoder;
Motor headTiltMotor1;
Motor headTiltMotor2;
Potentiometer headTiltPot;
Motor torsoTiltMotor1;
Motor torsoTiltMotor2;
Potentiometer torsoTiltPot1;
Potentiometer torsoTiltPot2;
ServoMotor mouthServo;
CurrentSensor busCurrentSensor;
VoltageCurrentSensor busVoltageCurrentSensor;
ShiftRegister<SHIFT_REGISTER_COUNT> statusLeds;
Meter batteryLevelMeter;
PowerDisplay powerDisplay;

/*----------------------------------------------------------*\
| Entry Points
\*----------------------------------------------------------*/

void setup()
{
  initializeADC();
  initializeI2C();
  initializeSerial();

  // Head pan
  headPanMotor.initialize(HEAD_PAN_ENABLE, HEAD_PAN_STEP, HEAD_PAN_DIR);

  headPanEncoder.initialize(
    HEAD_PAN_ENCODER_STATUS_REGISTER,
    HEAD_PAN_CS,
    0,
    AbsoluteEncoder::MAX,
    0.0,
    1.0,
    false,
    writeRegister
  );

  // Head tilt
  headTiltMotor1.initialize(
    HEAD_TILT_MOTOR1_EN,
    HEAD_TILT_MOTOR1_LPWM,
    HEAD_TILT_MOTOR1_RPWM,
    Motor::CURRENT_SENSE_UNUSED
  );

  headTiltMotor2.initialize(
    HEAD_TILT_MOTOR2_EN,
    HEAD_TILT_MOTOR2_LPWM,
    HEAD_TILT_MOTOR2_RPWM,
    Motor::CURRENT_SENSE_UNUSED
  );

  headTiltPot.initialize(HEAD_TILT_POTENTIOMETER);

  // Torso pan
  torsoPanMotor.initialize(TORSO_PAN_ENABLE, TORSO_PAN_DIR, TORSO_PAN_PWM, TORSO_PAN_STATUS);

  torsoPanEncoder.initialize(
    TORSO_PAN_ENCODER_STATUS_REGISTER,
    TORSO_PAN_CS,
    0,
    AbsoluteEncoder::MAX,
    0.0,
    1.0,
    false,
    writeRegister
  );

  // Torso tilt
  torsoTiltMotor1.initialize(
    TORSO_TILT_MOTOR1_EN,
    TORSO_TILT_MOTOR1_LPWM,
    TORSO_TILT_MOTOR1_RPWM,
    Motor::CURRENT_SENSE_UNUSED
  );

  torsoTiltMotor2.initialize(
    TORSO_TILT_MOTOR2_EN,
    TORSO_TILT_MOTOR2_LPWM,
    TORSO_TILT_MOTOR2_RPWM,
    Motor::CURRENT_SENSE_UNUSED
  );

  torsoTiltPot1.initialize(TORSO_TILT_POTENTIOMETER1);
  torsoTiltPot2.initialize(TORSO_TILT_POTENTIOMETER2);

  // Mouth
  mouthServo.initialize(MOUTH_SERVO_SIG);

  // Sensors
  busCurrentSensor.initialize(BUS_CURRENT_SENSOR_PIN);
  busVoltageCurrentSensor.initialize();

  // Indicators
  statusLeds.initialize(
    SHIFT_REGISTER_DATA_PIN,
    SHIFT_REGISTER_CLOCK_PIN,
    SHIFT_REGISTER_LATCH_PIN,
    SHIFT_REGISTER_OUTPUT_COUNT,
    HIGH
  );

  batteryLevelMeter.initialize(
    BATTERY_METER_LEVELS,
    BATTERY_METER_REGISTERS,
    writeRegister
  );

  powerDisplay.initialize();
  
  //
  // Testing
  //

  digitalWrite(HEAD_PAN_ENABLE, HIGH);
  digitalWrite(HEAD_PAN_STEP, HIGH);
  digitalWrite(HEAD_PAN_DIR, HIGH);

  digitalWrite(TORSO_PAN_ENABLE, HIGH);
  digitalWrite(TORSO_PAN_DIR, HIGH);
  digitalWrite(TORSO_PAN_PWM, HIGH);

  digitalWrite(HEAD_TILT_MOTOR1_EN, HIGH);
  digitalWrite(HEAD_TILT_MOTOR1_LPWM, HIGH);
  digitalWrite(HEAD_TILT_MOTOR1_RPWM, HIGH);
  digitalWrite(HEAD_TILT_MOTOR2_EN, HIGH);
  digitalWrite(HEAD_TILT_MOTOR2_LPWM, HIGH);
  digitalWrite(HEAD_TILT_MOTOR2_RPWM, HIGH);

  delay(STARTUP_DELAY);
}

void loop()
{
  updatePowerDisplay();
}

/*----------------------------------------------------------*\
| Functions
\*----------------------------------------------------------*/

void initializeADC()
{
  analogReadResolution(12);
  analogReadAveraging(8);
}

void initializeI2C()
{
  Wire.begin();
  Wire.setClock(I2C_FREQUENCY);
}

void initializeSerial()
{
  Serial.begin(115200);
  while (!Serial && millis() < 2000);
  Serial.write("setup starting");
}

void updatePowerDisplay()
{
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();

  if (lastUpdate != 0 && now - lastUpdate < POWER_DISPLAY_UPDATE_INTERVAL)
  {
    return;
  }

  lastUpdate = now;

  float voltage = busVoltageCurrentSensor.readVoltage();
  float current = busVoltageCurrentSensor.readCurrent();

  powerDisplay.update(voltage, current);
}

void writeRegister(int id, bool value)
{
  statusLeds.write(id, value);
}
