/*
                                                                                     ███████
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████
 main.ino
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

const int CURRENT_SENSE_PIN = A0;
const int VOLTAGE_SENSE_PIN = A3;

const int HEAD_PAN_ENABLE = 0;
const int HEAD_PAN_DIR = 1;
const int HEAD_PAN_STEP = 2;

const int TORSO_PAN_ENABLE = 3;
const int TORSO_PAN_DIR = 4;
const int TORSO_PAN_PWM = 5;

const int HEAD_TILT_POTENTIOMETER = A1;
const int HEAD_TILT_MOTOR1_EN = 7;
const int HEAD_TILT_MOTOR1_LPWM = 9;
const int HEAD_TILT_MOTOR1_RPWM = 10;
const int HEAD_TILT_MOTOR2_EN = 8;
const int HEAD_TILT_MOTOR2_LPWM = 11;
const int HEAD_TILT_MOTOR2_RPWM = 12;

const int TORSO_TILT_POTENTIOMETER = A2;
const int TORSO_TILT_MOTOR1_EN = 20;
const int TORSO_TILT_MOTOR1_LPWM = 22;
const int TORSO_TILT_MOTOR1_RPWM = 23;
const int TORSO_TILT_MOTOR2_EN = 21;
const int TORSO_TILT_MOTOR2_LPWM = 28;
const int TORSO_TILT_MOTOR2_RPWM = 29;

const int MOUTH_SERVO_SIG = 36;

const int BATTERY_METER_LEVELS = 10;
const int BATTERY_METER_PINS[BATTERY_METER_LEVELS] =
{
  24, // LED1 (Lowest charge)
  25, // LED2
  26, // LED3
  27, // LED4
  30, // LED5
  31, // LED6
  32, // LED7
  33, // LED8
  34, // LED9
  35  // LED10 (Highest charge)
};

const int STARTUP_DELAY = 1000;
const int I2C_FREQUENCY = 400000;

const uint8_t POWER_DISPLAY_ADDRESS_1 = 0x3C;
const uint8_t POWER_DISPLAY_ADDRESS_2 = 0x3D;
const unsigned long POWER_UPDATE_INTERVAL = 100;
const float BATTERY_FULL_VOLTS = 29.2f;
const float BATTERY_EMPTY_VOLTS = 20.0f;
const float CHARGE_SMOOTHING_ALPHA = 0.15f;

/*----------------------------------------------------------*\
| Forward Declarations
\*----------------------------------------------------------*/

void initializeSerial();
void initializeADC();
void initializeI2C();
void updatePowerMonitoring();
float estimateBatteryCharge(float packVoltage);

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

StepperMotor headPanMotor;
TeknicMotor torsoPanMotor;
Motor headTiltMotor1;
Motor headTiltMotor2;
Potentiometer headTiltPot;
Motor torsoTiltMotor1;
Motor torsoTiltMotor2;
Potentiometer torsoTiltPot;
ServoMotor mouthServo;

CurrentSensor currentSensor;
VoltageSensor voltageSensor;

Meter batteryLevelMeter(
  BATTERY_METER_LEVELS,
  BATTERY_METER_PINS
);

PowerDisplay voltageCurrentDisplay(
  POWER_DISPLAY_ADDRESS_1,
  PowerDisplay::VOLTAGE_CURRENT
);

PowerDisplay powerDisplay(
  POWER_DISPLAY_ADDRESS_2,
  PowerDisplay::POWER
);

float busVoltage = 0.0f;
float busCurrent = 0.0f;
float batteryCharge = 0.0f;

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

  // Torso pan
  torsoPanMotor.initialize(TORSO_PAN_ENABLE, TORSO_PAN_DIR, TORSO_PAN_PWM);

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

  torsoTiltPot.initialize(TORSO_TILT_POTENTIOMETER);

  // Mouth
  mouthServo.initialize(MOUTH_SERVO_SIG);

  // Sensors
  currentSensor.initialize(CURRENT_SENSE_PIN);
  voltageSensor.initialize(VOLTAGE_SENSE_PIN);

  voltageCurrentDisplay.initialize();
  powerDisplay.initialize();

  delay(STARTUP_DELAY);

  torsoTiltMotor1.enable();
  torsoTiltMotor1.write(0.5);
  torsoTiltMotor2.enable();
  torsoTiltMotor2.write(0.5);

  headTiltMotor1.enable();
  headTiltMotor1.write(0.5);
  headTiltMotor2.enable();
  headTiltMotor2.write(0.5);

  torsoPanMotor.enable();
  torsoPanMotor.write(0.5);

  headPanMotor.enable();
  headPanMotor.write(0.5);
}

void loop()
{
  updatePowerMonitoring();
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
}

void updatePowerMonitoring()
{
  static unsigned long lastPowerUpdateMs = 0;
  unsigned long now = millis();

  if (lastPowerUpdateMs != 0 && now - lastPowerUpdateMs < POWER_UPDATE_INTERVAL)
  {
    return;
  }

  lastPowerUpdateMs = now;
  busVoltage = voltageSensor.read();
  busCurrent = currentSensor.read();

  float measuredCharge = estimateBatteryCharge(busVoltage);

  if (batteryCharge <= 0.0f)
  {
    batteryCharge = measuredCharge;
  }
  else
  {
    batteryCharge += (measuredCharge - batteryCharge) * CHARGE_SMOOTHING_ALPHA;
  }

  batteryLevelMeter.write(batteryCharge);
  voltageCurrentDisplay.update(busVoltage, busCurrent);
  powerDisplay.update(busVoltage, busCurrent);
}

float estimateBatteryCharge(float packVoltage)
{
  return constrain(
    (packVoltage - BATTERY_EMPTY_VOLTS) / (BATTERY_FULL_VOLTS - BATTERY_EMPTY_VOLTS),
    0.0f,
    1.0f
  );
}
