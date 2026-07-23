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

#include <Wire.h>                    // I2C
#include <ros.h>                     // ROS serial transport
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/JointState.h>
#include <str1ker/MainCommand.h>
#include <str1ker/PowerState.h>
#include <firmware.h>                // See ../readme.md

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
const int ROS_BAUD = 115200;
const int CONTROL_RATE_HZ = 100;
const int TELEMETRY_RATE_HZ = 10;
const int COMMAND_TIMEOUT_MS = 500;
const int MOUTH_STARTUP_POSITION = 90;
const float DEGREES_TO_RADIANS = PI / 180.0f;

const uint8_t POWER_DISPLAY_ADDRESS_1 = 0x3C;
const uint8_t POWER_DISPLAY_ADDRESS_2 = 0x3D;
const unsigned long POWER_UPDATE_INTERVAL = 100;
const float BATTERY_FULL_VOLTS = 29.2f;
const float BATTERY_EMPTY_VOLTS = 20.0f;
const float CHARGE_SMOOTHING_ALPHA = 0.15f;

/*----------------------------------------------------------*\
| Forward Declarations
\*----------------------------------------------------------*/

void initializeADC();
void initializeI2C();
void initializeRos();
void initializeRosInterface();
void initializeActuators();
void mainCommand(const str1ker::MainCommand& msg);
void updateActuators();
void updatePowerMonitoring();
void publishTelemetry();
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
float headPanCommand = 0.0f;
float headTiltCommand = 0.0f;
float torsoPanCommand = 0.0f;
float torsoTiltCommand = 0.0f;
uint32_t lastCommandTime = 0;

ros::NodeHandle node;
str1ker::PowerState powerStateMsg;
sensor_msgs::BatteryState batteryStateMsg;
sensor_msgs::JointState jointStateMsg;
ros::Publisher powerStatePub("main/power", &powerStateMsg);
ros::Publisher batteryStatePub("main/battery", &batteryStateMsg);
ros::Publisher jointStatePub("main/joint_states", &jointStateMsg);
ros::Subscriber<str1ker::MainCommand> commandSub("main/command", mainCommand);

char headTiltJointName[] = "head_tilt_joint";
char torsoTiltJointName[] = "torso_tilt_joint";
char mouthJointName[] = "mouth_joint";
char* jointNames[] =
{
  headTiltJointName,
  torsoTiltJointName,
  mouthJointName
};
float jointPositions[3] = {0.0f, 0.0f, 0.0f};

/*----------------------------------------------------------*\
| Entry Points
\*----------------------------------------------------------*/

void setup()
{
  initializeADC();
  initializeI2C();
  initializeActuators();

  voltageCurrentDisplay.initialize();
  powerDisplay.initialize();

  initializeRos();
  initializeRosInterface();
  delay(STARTUP_DELAY);
}

void loop()
{
  static uint32_t lastControlTime = 0;
  static uint32_t lastTelemetryTime = 0;
  uint32_t now = millis();

  // AccelStepper must be serviced as often as possible to generate step pulses.
  headPanMotor.write(headPanCommand);
  node.spinOnce();

  if (lastControlTime == 0 ||
      now - lastControlTime >= (1000 / CONTROL_RATE_HZ))
  {
    updateActuators();
    lastControlTime = now;
  }

  if (lastTelemetryTime == 0 ||
      now - lastTelemetryTime >= (1000 / TELEMETRY_RATE_HZ))
  {
    updatePowerMonitoring();
    publishTelemetry();
    lastTelemetryTime = now;
  }
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

void initializeRos()
{
  node.getHardware()->setBaud(ROS_BAUD);
  node.initNode();
}

void initializeRosInterface()
{
  jointStateMsg.name_length = 3;
  jointStateMsg.name = jointNames;
  jointStateMsg.position_length = 3;
  jointStateMsg.position = jointPositions;
  jointStateMsg.velocity_length = 0;
  jointStateMsg.effort_length = 0;

  batteryStateMsg.power_supply_status =
    sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  batteryStateMsg.power_supply_health =
    sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  batteryStateMsg.power_supply_technology =
    sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIFE;
  batteryStateMsg.present = true;
  batteryStateMsg.temperature = NAN;
  batteryStateMsg.charge = NAN;
  batteryStateMsg.capacity = NAN;
  batteryStateMsg.design_capacity = NAN;

  node.advertise(powerStatePub);
  node.advertise(batteryStatePub);
  node.advertise(jointStatePub);
  node.subscribe(commandSub);
}

void initializeActuators()
{
  headPanMotor.initialize(HEAD_PAN_ENABLE, HEAD_PAN_STEP, HEAD_PAN_DIR);
  torsoPanMotor.initialize(TORSO_PAN_ENABLE, TORSO_PAN_DIR, TORSO_PAN_PWM);

  headTiltMotor1.initialize(
    HEAD_TILT_MOTOR1_EN, HEAD_TILT_MOTOR1_LPWM,
    HEAD_TILT_MOTOR1_RPWM, Motor::CURRENT_SENSE_UNUSED);
  headTiltMotor2.initialize(
    HEAD_TILT_MOTOR2_EN, HEAD_TILT_MOTOR2_LPWM,
    HEAD_TILT_MOTOR2_RPWM, Motor::CURRENT_SENSE_UNUSED);
  torsoTiltMotor1.initialize(
    TORSO_TILT_MOTOR1_EN, TORSO_TILT_MOTOR1_LPWM,
    TORSO_TILT_MOTOR1_RPWM, Motor::CURRENT_SENSE_UNUSED);
  torsoTiltMotor2.initialize(
    TORSO_TILT_MOTOR2_EN, TORSO_TILT_MOTOR2_LPWM,
    TORSO_TILT_MOTOR2_RPWM, Motor::CURRENT_SENSE_UNUSED);

  // This sketch selects 12-bit ADC reads, so use the full 0..4095 range.
  headTiltPot.initialize(HEAD_TILT_POTENTIOMETER, 0, 4095);
  torsoTiltPot.initialize(TORSO_TILT_POTENTIOMETER, 0, 4095);
  mouthServo.initialize(MOUTH_SERVO_SIG);
  mouthServo.write(MOUTH_STARTUP_POSITION);

  currentSensor.initialize(CURRENT_SENSE_PIN);
  voltageSensor.initialize(VOLTAGE_SENSE_PIN);

  headPanMotor.enable();
  torsoPanMotor.enable();
  headTiltMotor1.enable();
  headTiltMotor2.enable();
  torsoTiltMotor1.enable();
  torsoTiltMotor2.enable();
}

void mainCommand(const str1ker::MainCommand& msg)
{
  headPanCommand = constrain(msg.headPanVelocity, -1.0f, 1.0f);
  headTiltCommand = constrain(msg.headTiltVelocity, -1.0f, 1.0f);
  torsoPanCommand = constrain(msg.torsoPanVelocity, -1.0f, 1.0f);
  torsoTiltCommand = constrain(msg.torsoTiltVelocity, -1.0f, 1.0f);
  mouthServo.write(msg.mouthPosition);
  lastCommandTime = millis();
}

void updateActuators()
{
  if (lastCommandTime == 0 || millis() - lastCommandTime > COMMAND_TIMEOUT_MS)
  {
    headPanCommand = 0.0f;
    headTiltCommand = 0.0f;
    torsoPanCommand = 0.0f;
    torsoTiltCommand = 0.0f;
  }

  torsoPanMotor.write(torsoPanCommand);
  headTiltMotor1.write(headTiltCommand);
  headTiltMotor2.write(headTiltCommand);
  torsoTiltMotor1.write(torsoTiltCommand);
  torsoTiltMotor2.write(torsoTiltCommand);
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

void publishTelemetry()
{
  ros::Time stamp = node.now();
  float watts = busVoltage * busCurrent;

  powerStateMsg.voltage = busVoltage;
  powerStateMsg.current = busCurrent;
  powerStateMsg.power = watts;
  powerStatePub.publish(&powerStateMsg);

  batteryStateMsg.header.stamp = stamp;
  batteryStateMsg.voltage = busVoltage;
  batteryStateMsg.current = busCurrent;
  batteryStateMsg.percentage = batteryCharge;
  batteryStatePub.publish(&batteryStateMsg);

  jointPositions[0] = headTiltPot.read();
  jointPositions[1] = torsoTiltPot.read();
  jointPositions[2] = float(mouthServo.read()) * DEGREES_TO_RADIANS;
  jointStateMsg.header.stamp = stamp;
  jointStatePub.publish(&jointStateMsg);
}

float estimateBatteryCharge(float packVoltage)
{
  return constrain(
    (packVoltage - BATTERY_EMPTY_VOLTS) / (BATTERY_FULL_VOLTS - BATTERY_EMPTY_VOLTS),
    0.0f,
    1.0f
  );
}
