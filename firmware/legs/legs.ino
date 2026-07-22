/*
                                                                                     ███████
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████
 legs.ino
 Mobile Robot Platform Firmware
 Copyright (C) 2026 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <Wire.h>
#include <Encoder.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Adafruit_BNO055.h>
#include "leg.h"

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const int SERIAL_BAUD = 115200;           // USB ROS serial console rate
const int I2C_FREQUENCY_HZ = 400000;      // I2C bus frequency
const int RATE_HZ = 100;                  // Main control/sampling rate
const int REPORT_HZ = 10;                 // Human-readable telemetry rate
const int STARTUP_DELAY = 1000;           // Allow USB serial to appear
const int COMMAND_TIMEOUT_MS = 500;       // Stop if cmd_vel is stale
const float MS_TO_SECONDS = 0.001f;       // Milliseconds to seconds conversion factor

const float WHEEL_RADIUS_M = 0.1016f;
const float WHEEL_BASE_LENGTH_M = 1.180f;
const float WHEEL_BASE_WIDTH_M = 0.418f;
const float WHEEL_SEPARATION_SUM_M =
  (WHEEL_BASE_LENGTH_M) + (WHEEL_BASE_WIDTH_M) * 0.5f;
const float MAX_WHEEL_RAD_S = 12.0f;
const bool INVERT_RIGHT_WHEELS = true;

const int BNO055_ADDRESS = 0x28;          // BNO055 default I2C address
const int BNO055_RESET_PIN = 26;          // BNO055 hardware reset pin

const int LED_MUX_ADDRESS = 0x34;         // IS31FL3739 I2C address with AD tied to GND

const int FRONT_LEFT_ACTUATOR_POT = A0;   // Front-left actuator feedback pin
const int FRONT_LEFT_WHEEL_A = 0;         // Front-left wheel encoder A
const int FRONT_LEFT_WHEEL_B = 1;         // Front-left wheel encoder B
const int FRONT_LEFT_WHEEL_INDEX = 22;    // Front-left wheel encoder index
const int FRONT_LEFT_WHEEL_EN = 6;        // Front-left wheel driver enable
const int FRONT_LEFT_WHEEL_RPWM = 0;      // Front-left wheel RPWM
const int FRONT_LEFT_WHEEL_LPWM = 1;      // Front-left wheel LPWM
const int FRONT_LEFT_ACTUATOR_EN = 9;     // Front-left actuator driver enable
const int FRONT_LEFT_ACTUATOR_RPWM = 2;   // Front-left actuator RPWM
const int FRONT_LEFT_ACTUATOR_LPWM = 3;   // Front-left actuator LPWM

const int FRONT_RIGHT_ACTUATOR_POT = A1;  // Front-right actuator feedback pin
const int FRONT_RIGHT_WHEEL_A = 2;        // Front-right wheel encoder A
const int FRONT_RIGHT_WHEEL_B = 3;        // Front-right wheel encoder B
const int FRONT_RIGHT_WHEEL_INDEX = 23;   // Front-right wheel encoder index
const int FRONT_RIGHT_WHEEL_EN = 10;      // Front-right wheel driver enable
const int FRONT_RIGHT_WHEEL_RPWM = 4;     // Front-right wheel RPWM
const int FRONT_RIGHT_WHEEL_LPWM = 5;     // Front-right wheel LPWM
const int FRONT_RIGHT_ACTUATOR_EN = 11;   // Front-right actuator driver enable
const int FRONT_RIGHT_ACTUATOR_RPWM = 6;  // Front-right actuator RPWM
const int FRONT_RIGHT_ACTUATOR_LPWM = 7;  // Front-right actuator LPWM

const int REAR_LEFT_ACTUATOR_POT = A2;    // Rear-left actuator feedback pin
const int REAR_LEFT_WHEEL_A = 4;          // Rear-left wheel encoder A
const int REAR_LEFT_WHEEL_B = 5;          // Rear-left wheel encoder B
const int REAR_LEFT_WHEEL_INDEX = 24;     // Rear-left wheel encoder index
const int REAR_LEFT_WHEEL_EN = 12;        // Rear-left wheel driver enable
const int REAR_LEFT_WHEEL_RPWM = 8;       // Rear-left wheel RPWM
const int REAR_LEFT_WHEEL_LPWM = 9;       // Rear-left wheel LPWM
const int REAR_LEFT_ACTUATOR_EN = 13;     // Rear-left actuator driver enable
const int REAR_LEFT_ACTUATOR_RPWM = 10;   // Rear-left actuator RPWM
const int REAR_LEFT_ACTUATOR_LPWM = 11;   // Rear-left actuator LPWM

const int REAR_RIGHT_ACTUATOR_POT = A3;   // Rear-right actuator feedback pin
const int REAR_RIGHT_WHEEL_A = 7;         // Rear-right wheel encoder A
const int REAR_RIGHT_WHEEL_B = 8;         // Rear-right wheel encoder B
const int REAR_RIGHT_WHEEL_INDEX = 25;    // Rear-right wheel encoder index
const int REAR_RIGHT_WHEEL_EN = 20;       // Rear-right wheel driver enable
const int REAR_RIGHT_WHEEL_RPWM = 12;     // Rear-right wheel RPWM
const int REAR_RIGHT_WHEEL_LPWM = 13;     // Rear-right wheel LPWM
const int REAR_RIGHT_ACTUATOR_EN = 21;    // Rear-right actuator driver enable
const int REAR_RIGHT_ACTUATOR_RPWM = 14;  // Rear-right actuator RPWM
const int REAR_RIGHT_ACTUATOR_LPWM = 15;  // Rear-right actuator LPWM

const uint8_t IS31FL3739_PWM_START = 0x03;
const uint8_t IS31FL3739_PWM_END = 0x8F;
const uint8_t IS31FL3739_SCALING_START = 0x92;
const uint8_t IS31FL3739_SCALING_COUNT = 14;
const uint8_t IS31FL3739_CONFIGURATION = 0xA0;
const uint8_t IS31FL3739_GLOBAL_CURRENT = 0xA1;
const uint8_t IS31FL3739_PULL_RESISTORS = 0xB0;
const uint8_t IS31FL3739_PWM_FREQUENCY = 0xB2;
const uint8_t IS31FL3739_RESET = 0xCF;

const uint8_t IS31FL3739_RESET_COMMAND = 0xAE;
const uint8_t IS31FL3739_8_SW_8_CS_NORMAL = 0x11;
const uint8_t IS31FL3739_GLOBAL_CURRENT_TEST = 0x20;
const uint8_t IS31FL3739_2K_PULLS_WHEN_OFF = 0x33;
const uint8_t IS31FL3739_PWM_32_KHZ = 0x01;
const uint8_t LED_FULL_ON = 0xFF;
const uint8_t LED_OFF = 0x00;

const uint8_t DISPLAY_UPDATE_PERIOD_MS = 250;
const uint8_t DISPLAY_0_START_TENTHS = 0;
const uint8_t DISPLAY_1_START_TENTHS = 3;
const uint8_t DISPLAY_2_START_TENTHS = 6;
const uint8_t DISPLAY_3_START_TENTHS = 9;

const uint8_t SEG_A = 1 << 0; // Top of upper square
const uint8_t SEG_B = 1 << 1; // Right of upper square
const uint8_t SEG_C = 1 << 2; // Right of lower square
const uint8_t SEG_D = 1 << 3; // Bottom of lower square
const uint8_t SEG_E = 1 << 4; // Left of lower square
const uint8_t SEG_F = 1 << 5; // Left of upper square
const uint8_t SEG_G = 1 << 6; // Link of both squares
const uint8_t SEG_DP = 1 << 7;

const uint8_t DIGIT_SEGMENTS[] =
{
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,         // 0
  SEG_B | SEG_C,                                         // 1
  SEG_A | SEG_B | SEG_D | SEG_E | SEG_G,                 // 2
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_G,                 // 3
  SEG_B | SEG_C | SEG_F | SEG_G,                         // 4
  SEG_A | SEG_C | SEG_D | SEG_F | SEG_G,                 // 5
  SEG_A | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G,         // 6
  SEG_A | SEG_B | SEG_C,                                 // 7
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G, // 8
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_F | SEG_G          // 9
};

/*----------------------------------------------------------*\
| Forward Declarations
\*----------------------------------------------------------*/

void initializeSerial();
void initializeADC();
void initializeI2C();
void initializeImu();
void initializeLegs();
void initializeEncoders();
void enableMotors();
void disableMotors();
void stopMotors();
void updateLegs(float timeStep);
void updateCommandWatchdog();
void setWheelCommands(
  float frontLeftCommand,
  float frontRightCommand,
  float rearLeftCommand,
  float rearRightCommand
);
void commandVelocity(const geometry_msgs::Twist& msg);
void frontLeftIndexInterrupt();
void frontRightIndexInterrupt();
void rearLeftIndexInterrupt();
void rearRightIndexInterrupt();
void initializeWheelDisplays();
void writeDisplay(uint8_t display, uint8_t tenths);

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

Adafruit_BNO055 accelerometer(BNO055_ID, BNO055_ADDRESS, &Wire);

bool accelerometerInitialized = false;
bool motorsEnabled = false;

Leg frontLeft;
Leg frontRight;
Leg rearLeft;
Leg rearRight;

ros::NodeHandle node;
ros::Subscriber<geometry_msgs::Twist> commandSub("cmd_vel", commandVelocity);
uint32_t lastCommandTime = 0;

/*----------------------------------------------------------*\
| Entry Points
\*----------------------------------------------------------*/

void setup()
{
  initializeADC();
  initializeI2C();
  initializeImu();
  initializeLegs();
  initializeEncoders();
  initializeRos();
  initializeRosInterface();
  initializeWheelDisplays();
  enableMotors();
}

void loop()
{
  static uint32_t lastUpdate = 0;
  uint32_t now = millis();
  const uint32_t updatePeriod = 1000 / RATE_HZ;

  if (now < STARTUP_DELAY)
  {
    node.spinOnce();
    return;
  }

  if (lastUpdate == 0)
  {
    lastUpdate = now;
    node.spinOnce();
    return;
  }

  uint32_t elapsed = now - lastUpdate;

  if (elapsed < updatePeriod)
  {
    node.spinOnce();
    return;
  }

  float timeStep = elapsed * MS_TO_SECONDS;

  if (timeStep <= 0.0f)
  {
    lastUpdate = now;
    node.spinOnce();
    return;
  }

  updateLegs(timeStep);
  lastUpdate = now;
  node.spinOnce();
}

/*----------------------------------------------------------*\
| Functions
\*----------------------------------------------------------*/

void initializeRos()
{
  node.getHardware()->setBaud(SERIAL_BAUD);
  node.initNode();
}

void initializeRosInterface()
{
  node.subscribe(commandSub);
}

void initializeADC()
{
  analogReadResolution(12);
  analogReadAveraging(8);
}

void initializeI2C()
{
  Wire.begin();
  Wire.setClock(I2C_FREQUENCY_HZ);
}

void initializeImu()
{
  pinMode(BNO055_RESET_PIN, OUTPUT);
  digitalWrite(BNO055_RESET_PIN, LOW);
  delay(10);
  digitalWrite(BNO055_RESET_PIN, HIGH);
  delay(650);

  accelerometerInitialized = accelerometer.begin();

  if (accelerometerInitialized)
  {
    accelerometer.setExtCrystalUse(true);
  }
}

void initializeLegs()
{
  frontLeft.initialize(
    "front_left",
    FRONT_LEFT_ACTUATOR_POT,
    FRONT_LEFT_ACTUATOR_EN,
    FRONT_LEFT_ACTUATOR_LPWM,
    FRONT_LEFT_ACTUATOR_RPWM,
    FRONT_LEFT_WHEEL_EN,
    FRONT_LEFT_WHEEL_LPWM,
    FRONT_LEFT_WHEEL_RPWM,
    FRONT_LEFT_WHEEL_A,
    FRONT_LEFT_WHEEL_B,
    FRONT_LEFT_WHEEL_INDEX
  );

  frontRight.initialize(
    "front_right",
    FRONT_RIGHT_ACTUATOR_POT,
    FRONT_RIGHT_ACTUATOR_EN,
    FRONT_RIGHT_ACTUATOR_LPWM,
    FRONT_RIGHT_ACTUATOR_RPWM,
    FRONT_RIGHT_WHEEL_EN,
    FRONT_RIGHT_WHEEL_LPWM,
    FRONT_RIGHT_WHEEL_RPWM,
    FRONT_RIGHT_WHEEL_A,
    FRONT_RIGHT_WHEEL_B,
    FRONT_RIGHT_WHEEL_INDEX
  );

  rearLeft.initialize(
    "rear_left",
    REAR_LEFT_ACTUATOR_POT,
    REAR_LEFT_ACTUATOR_EN,
    REAR_LEFT_ACTUATOR_LPWM,
    REAR_LEFT_ACTUATOR_RPWM,
    REAR_LEFT_WHEEL_EN,
    REAR_LEFT_WHEEL_LPWM,
    REAR_LEFT_WHEEL_RPWM,
    REAR_LEFT_WHEEL_A,
    REAR_LEFT_WHEEL_B,
    REAR_LEFT_WHEEL_INDEX
  );

  rearRight.initialize(
    "rear_right",
    REAR_RIGHT_ACTUATOR_POT,
    REAR_RIGHT_ACTUATOR_EN,
    REAR_RIGHT_ACTUATOR_LPWM,
    REAR_RIGHT_ACTUATOR_RPWM,
    REAR_RIGHT_WHEEL_EN,
    REAR_RIGHT_WHEEL_LPWM,
    REAR_RIGHT_WHEEL_RPWM,
    REAR_RIGHT_WHEEL_A,
    REAR_RIGHT_WHEEL_B,
    REAR_RIGHT_WHEEL_INDEX
  );

  disableMotors();
}

void initializeEncoders()
{
  attachInterrupt(digitalPinToInterrupt(FRONT_LEFT_WHEEL_INDEX), frontLeftIndexInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(FRONT_RIGHT_WHEEL_INDEX), frontRightIndexInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(REAR_LEFT_WHEEL_INDEX), rearLeftIndexInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(REAR_RIGHT_WHEEL_INDEX), rearRightIndexInterrupt, RISING);
}

void enableMotors()
{
  frontLeft.enable();
  frontRight.enable();
  rearLeft.enable();
  rearRight.enable();

  motorsEnabled = true;
}

void disableMotors()
{
  frontLeft.disable();
  frontRight.disable();
  rearLeft.disable();
  rearRight.disable();

  motorsEnabled = false;
}

void stopMotors()
{
  frontLeft.stop();
  frontRight.stop();
  rearLeft.stop();
  rearRight.stop();
}

void setWheelCommands(
  float frontLeftCommand,
  float frontRightCommand,
  float rearLeftCommand,
  float rearRightCommand
)
{
  frontLeft.wheelCommand = constrain(frontLeftCommand, -1.0f, 1.0f);
  frontRight.wheelCommand = constrain(frontRightCommand, -1.0f, 1.0f);
  rearLeft.wheelCommand = constrain(rearLeftCommand, -1.0f, 1.0f);
  rearRight.wheelCommand = constrain(rearRightCommand, -1.0f, 1.0f);

  writeDisplay(0, int(abs(frontLeft.wheelCommand / 2.0f + 0.5f)) * 10);
  writeDisplay(1, int(abs(frontLeft.wheelCommand / 2.0f + 0.5f)) * 10);
  writeDisplay(2, int(abs(frontLeft.wheelCommand / 2.0f + 0.5f)) * 10);
  writeDisplay(3, int(abs(frontLeft.wheelCommand / 2.0f + 0.5f)) * 10);
}

void commandVelocity(const geometry_msgs::Twist& msg)
{
  float linearX = msg.linear.x;
  float linearY = msg.linear.y;
  float angularZ = msg.angular.z;

  float frontLeftRadS =
    (linearX - linearY - WHEEL_SEPARATION_SUM_M * angularZ) / WHEEL_RADIUS_M;
  float frontRightRadS =
    (linearX + linearY + WHEEL_SEPARATION_SUM_M * angularZ) / WHEEL_RADIUS_M;
  float rearLeftRadS =
    (linearX + linearY - WHEEL_SEPARATION_SUM_M * angularZ) / WHEEL_RADIUS_M;
  float rearRightRadS =
    (linearX - linearY + WHEEL_SEPARATION_SUM_M * angularZ) / WHEEL_RADIUS_M;

  if (INVERT_RIGHT_WHEELS)
  {
    frontRightRadS *= -1.0f;
    rearRightRadS *= -1.0f;
  }

  setWheelCommands(
    frontLeftRadS / MAX_WHEEL_RAD_S,
    frontRightRadS / MAX_WHEEL_RAD_S,
    rearLeftRadS / MAX_WHEEL_RAD_S,
    rearRightRadS / MAX_WHEEL_RAD_S
  );

  lastCommandTime = millis();
}

void updateCommandWatchdog()
{
  bool commandStale = (millis() - lastCommandTime) > COMMAND_TIMEOUT_MS;

  if (!node.connected() || lastCommandTime == 0 || commandStale)
  {
    stopMotors();
  }
}

void updateLegs(float timeStep)
{
  frontLeft.update(timeStep);
  frontRight.update(timeStep);
  rearLeft.update(timeStep);
  rearRight.update(timeStep);
}


void reportImu()
{
  if (!accelerometerInitialized)
  {
    return;
  }

  imu::Vector<3> linear = accelerometer.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> gyro = accelerometer.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> euler = accelerometer.getVector(Adafruit_BNO055::VECTOR_EULER);
}

void frontLeftIndexInterrupt()
{
  if (frontLeft.quadratureEncoder)
  {
    frontLeft.quadratureEncoder->handleIndexRise();
  }
}

void frontRightIndexInterrupt()
{
  if (frontRight.quadratureEncoder)
  {
    frontRight.quadratureEncoder->handleIndexRise();
  }
}

void rearLeftIndexInterrupt()
{
  if (rearLeft.quadratureEncoder)
  {
    rearLeft.quadratureEncoder->handleIndexRise();
  }
}

void rearRightIndexInterrupt()
{
  if (rearRight.quadratureEncoder)
  {
    rearRight.quadratureEncoder->handleIndexRise();
  }
}

void writeLedMuxRegister(uint8_t address, uint8_t value)
{
  Wire.beginTransmission(LED_MUX_ADDRESS);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission();
}

void fillLedMuxRegisters(uint8_t startAddress, uint8_t count, uint8_t value)
{
  Wire.beginTransmission(LED_MUX_ADDRESS);
  Wire.write(startAddress);

  for (uint8_t index = 0; index < count; index++)
  {
    Wire.write(value);
  }

  Wire.endTransmission();
}

void initializeWheelDisplays()
{
  writeLedMuxRegister(IS31FL3739_RESET, IS31FL3739_RESET_COMMAND);
  delay(1);

  writeLedMuxRegister(IS31FL3739_CONFIGURATION, IS31FL3739_8_SW_8_CS_NORMAL);
  writeLedMuxRegister(IS31FL3739_GLOBAL_CURRENT, IS31FL3739_GLOBAL_CURRENT_TEST);
  writeLedMuxRegister(IS31FL3739_PULL_RESISTORS, IS31FL3739_2K_PULLS_WHEN_OFF);
  writeLedMuxRegister(IS31FL3739_PWM_FREQUENCY, IS31FL3739_PWM_32_KHZ);
  fillLedMuxRegisters(IS31FL3739_SCALING_START, IS31FL3739_SCALING_COUNT, LED_FULL_ON);
  fillLedMuxRegisters(IS31FL3739_PWM_START, IS31FL3739_PWM_END - IS31FL3739_PWM_START + 1, LED_FULL_ON);
}

void writeDisplay(int display, int tenths)
{
  const uint8_t leftSw = (display * 2) + 1;
  const uint8_t rightSw = leftSw + 1;
  const uint8_t leftDigit = (tenths >= 10) ? 1 : 0;
  const uint8_t rightDigit = tenths % 10;

  for (uint8_t cs = 1; cs <= 8; cs++)
  {
    uint8_t csOffset = 0x00;

    switch (cs)
    {
      case 1: csOffset = 0x00; break;
      case 2: csOffset = 0x01; break;
      case 3: csOffset = 0x02; break;
      case 4: csOffset = 0x03; break;
      case 5: csOffset = 0x04; break;
      case 6: csOffset = 0x0B; break;
      case 7: csOffset = 0x0C; break;
      default: csOffset = 0x0D; break;
    }

    const uint8_t leftRegister = IS31FL3739_PWM_START + ((leftSw - 1) * 0x10) + csOffset;
    const uint8_t rightRegister = IS31FL3739_PWM_START + ((rightSw - 1) * 0x10) + csOffset;
    const uint8_t leftSegments = DIGIT_SEGMENTS[leftDigit] | SEG_DP;
    const uint8_t rightSegments = DIGIT_SEGMENTS[rightDigit];
    const uint8_t mask = 1 << (cs - 1);

    writeLedMuxRegister(leftRegister, (leftSegments & mask) ? LED_FULL_ON : LED_OFF);
    writeLedMuxRegister(rightRegister, (rightSegments & mask) ? LED_FULL_ON : LED_OFF);
  }
}
