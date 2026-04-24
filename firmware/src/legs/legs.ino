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
#include <Adafruit_BNO055.h>
#include <Adafruit_PWMServoDriver.h>
#include "leg.h"

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const int SERIAL_BAUD = 115200;           // USB serial console rate
const int I2C_FREQUENCY_HZ = 400000;      // I2C bus frequency
const int RATE_HZ = 100;                  // Main control/sampling rate
const int REPORT_HZ = 10;                 // Human-readable telemetry rate
const int STARTUP_DELAY = 1000;           // Allow USB serial to appear
const float MS_TO_SECONDS = 0.001f;       // Milliseconds to seconds conversion factor

const int PCA9685_ADDRESS = 0x40;         // Default PCA9685 I2C address
const int PCA9685_FREQUENCY_HZ = 1526;    // Near PCA9685 max, suitable for IBT2 PWM inputs

const int BNO055_ADDRESS = 0x28;          // BNO055 default I2C address
const int BNO055_RESET_PIN = 26;          // BNO055 hardware reset pin

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

/*----------------------------------------------------------*\
| Forward Declarations
\*----------------------------------------------------------*/

void initializeSerial();
void initializeADC();
void initializeI2C();
void initializePwmDriver();
void initializeImu();
void initializeLegs();
void initializeEncoders();
void enableMotors();
void disableMotors();
void stopMotors();
void updateLegs(float timeStep);
void reportState();
void reportImu();
void reportLeg(const Leg& leg);
void handleSerial();
void printHelp();
void writeMuxPwmChannel(int channel, int pwm);
void frontLeftIndexInterrupt();
void frontRightIndexInterrupt();
void rearLeftIndexInterrupt();
void rearRightIndexInterrupt();

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS, Wire);
Adafruit_BNO055 accelerometer(BNO055_ID, BNO055_ADDRESS, &Wire);
bool accelerometerInitialized = false;
bool motorsEnabled = false;
Leg frontLeft;
Leg frontRight;
Leg rearLeft;
Leg rearRight;

/*----------------------------------------------------------*\
| Entry Points
\*----------------------------------------------------------*/

void setup()
{
  initializeSerial();
  initializeADC();
  initializeI2C();
  initializePwmDriver();
  initializeImu();
  initializeLegs();
  initializeEncoders();

  //frontLeft->enable();
  //frontLeft->wheelCommand = 1.0;

  //frontRight->enable();
  //frontRight->wheelCommand = -1.0;

  rearLeft.enable();
  //rearLeft.wheelCommand = 1.0;

  rearRight.enable();
  //rearRight.wheelCommand = 1.0;
}

void loop()
{
  /*static uint32_t lastUpdate = 0;
  static uint32_t lastReport = 0;

  uint32_t now = millis();
  const uint32_t updatePeriod = 1000 / RATE_HZ;
  const uint32_t reportPeriod = 1000 / REPORT_HZ;

  if (now < STARTUP_DELAY)
  {
    return;
  }

  if (lastUpdate == 0)
  {
    lastUpdate = now;
    lastReport = now;
    return;
  }

  uint32_t elapsed = now - lastUpdate;

  if (elapsed < updatePeriod)
  {
    return;
  }

  float timeStep = elapsed * MS_TO_SECONDS;

  if (timeStep <= 0.0f)
  {
    lastUpdate = now;
    return;
  }

  updateLegs(timeStep);
  lastUpdate = now;

  if ((now - lastReport) >= reportPeriod)
  {
    // TODO: publish report
    lastReport = now;
  }*/
}

/*----------------------------------------------------------*\
| Functions
\*----------------------------------------------------------*/

void initializeSerial()
{
  Serial.begin(SERIAL_BAUD);

  while (!Serial && millis() < STARTUP_DELAY)
  {
    delay(10);
  }
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

void initializePwmDriver()
{
  pwm.begin();
  pwm.setPWMFreq(PCA9685_FREQUENCY_HZ);
}

void writeMuxPwmChannel(int channelId, int pwmValue)
{
  int limited = constrain(pwmValue, 0, int(Motor::PWM_MAX));
  uint16_t scaled = uint16_t((uint32_t(limited) * 4095u) / uint32_t(Motor::PWM_MAX));
  pwm.setPWM(channelId, 0, scaled);
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

  Serial.print("accelerometer ");
  Serial.println(accelerometerInitialized ? "ready" : "not detected");
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
    FRONT_LEFT_WHEEL_INDEX,
    writeMuxPwmChannel
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
    FRONT_RIGHT_WHEEL_INDEX,
    writeMuxPwmChannel
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
    REAR_LEFT_WHEEL_INDEX,
    writeMuxPwmChannel
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
    REAR_RIGHT_WHEEL_INDEX,
    writeMuxPwmChannel
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

void updateLegs(float timeStep)
{
  frontLeft.update(timeStep);
  frontRight.update(timeStep);
  rearLeft.update(timeStep);
  rearRight.update(timeStep);
}

void reportState()
{
  Serial.print("motors ");
  Serial.println(motorsEnabled ? "enabled" : "disabled");

  reportLeg(frontLeft);
  reportLeg(frontRight);
  reportLeg(rearLeft);
  reportLeg(rearRight);

  reportImu();
  Serial.println();
}

void reportImu()
{
  if (!accelerometerInitialized)
  {
    Serial.println("accelerometer unavailable");
    return;
  }

  imu::Vector<3> linear = accelerometer.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> gyro = accelerometer.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> euler = accelerometer.getVector(Adafruit_BNO055::VECTOR_EULER);

  Serial.print("accelerometer euler_deg ");
  Serial.print(euler.x(), 3);
  Serial.print(" ");
  Serial.print(euler.y(), 3);
  Serial.print(" ");
  Serial.print(euler.z(), 3);
  Serial.print(" gyro_rad_s ");
  Serial.print(gyro.x(), 3);
  Serial.print(" ");
  Serial.print(gyro.y(), 3);
  Serial.print(" ");
  Serial.print(gyro.z(), 3);
  Serial.print(" linear_m_s2 ");
  Serial.print(linear.x(), 3);
  Serial.print(" ");
  Serial.print(linear.y(), 3);
  Serial.print(" ");
  Serial.println(linear.z(), 3);
}

void reportLeg(const Leg& leg)
{
  Serial.print(leg.name);
  Serial.print(" actuator_raw ");
  Serial.print(leg.actuatorSensor.raw());
  Serial.print(" actuator_norm ");
  Serial.print(leg.actuatorPosition, 4);
  Serial.print(" wheel_counts ");
  Serial.print(leg.quadratureEncoder ? leg.quadratureEncoder->counts : 0);
  Serial.print(" wheel_rev ");
  Serial.print(leg.wheelRevolutions, 5);
  Serial.print(" wheel_rad ");
  Serial.print(leg.wheelRadians, 5);
  Serial.print(" wheel_rps ");
  Serial.print(leg.wheelVelocity, 5);
  Serial.print(" index_seen ");
  Serial.print(leg.quadratureEncoder ? leg.quadratureEncoder->indexCount : 0);
  Serial.print(" index_state ");
  Serial.println((leg.quadratureEncoder && leg.quadratureEncoder->indexState) ? 1 : 0);
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
