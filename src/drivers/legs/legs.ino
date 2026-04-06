/*
                                                                                     ███████
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████
 legs.ino
 Leg Driver Board
 Copyright (C) 2026 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Dependencies
\*----------------------------------------------------------*/

// - Encoder
// - Adafruit BNO055
// - Adafruit Unified Sensor
// - Adafruit PWM Servo Driver Library

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <Wire.h>
#include <Encoder.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_PWMServoDriver.h>
#include "../common/pca9685_motor.h"
#include "../common/potentiometer.h"
#include "../common/wheel_encoder.h"

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const int SERIAL_BAUD = 115200;                     // USB serial console rate
const int RATE_HZ = 100;                            // Main control/sampling rate
const int REPORT_HZ = 10;                           // Human-readable telemetry rate
const int STARTUP_DELAY = 1000;                     // Allow USB serial to appear
const float MILLISECONDS_TO_SECONDS = 0.001f;       // Milliseconds to seconds conversion factor
const float TWO_PI_F = 6.28318530718f;              // One full revolution in radians

const uint8_t PCA9685_ADDRESS = 0x40;               // Default PCA9685 I2C address
const uint16_t PCA9685_FREQUENCY_HZ = 1526;         // Near PCA9685 max, suitable for IBT2 PWM inputs
const uint8_t BNO055_ID = 55;                       // Adafruit_BNO055 default sensor id
const uint8_t BNO055_ADDRESS = 0x28;                // Default BNO055 I2C address

const float ENCODER_PPR = 1000.0;                   // AS5047 pulses per revolution
const float ENCODER_CPR = ENCODER_PPR * 4.0f;       // AS5047 counts per revolution

const int FRONT_LEFT_ACTUATOR_POT = A0;             // Front-left actuator feedback pin
const int FRONT_RIGHT_ACTUATOR_POT = A1;            // Front-right actuator feedback pin
const int REAR_LEFT_ACTUATOR_POT = A2;              // Rear-left actuator feedback pin
const int REAR_RIGHT_ACTUATOR_POT = A3;             // Rear-right actuator feedback pin

const int FRONT_LEFT_WHEEL_A = 0;                   // Front-left wheel encoder A
const int FRONT_LEFT_WHEEL_B = 1;                   // Front-left wheel encoder B
const int FRONT_RIGHT_WHEEL_A = 2;                  // Front-right wheel encoder A
const int FRONT_RIGHT_WHEEL_B = 3;                  // Front-right wheel encoder B
const int REAR_LEFT_WHEEL_A = 4;                    // Rear-left wheel encoder A
const int REAR_LEFT_WHEEL_B = 5;                    // Rear-left wheel encoder B
const int REAR_RIGHT_WHEEL_A = 7;                   // Rear-right wheel encoder A
const int REAR_RIGHT_WHEEL_B = 8;                   // Rear-right wheel encoder B

const int FRONT_LEFT_WHEEL_INDEX = 22;              // Front-left wheel encoder index
const int FRONT_RIGHT_WHEEL_INDEX = 23;             // Front-right wheel encoder index
const int REAR_LEFT_WHEEL_INDEX = 24;               // Rear-left wheel encoder index
const int REAR_RIGHT_WHEEL_INDEX = 25;              // Rear-right wheel encoder index

const int FRONT_LEFT_WHEEL_EN = 6;                  // Front-left wheel driver enable
const int FRONT_RIGHT_WHEEL_EN = 9;                 // Front-right wheel driver enable
const int REAR_LEFT_WHEEL_EN = 10;                  // Rear-left wheel driver enable
const int REAR_RIGHT_WHEEL_EN = 11;                 // Rear-right wheel driver enable

const int FRONT_LEFT_ACTUATOR_EN = 12;              // Front-left actuator driver enable
const int FRONT_RIGHT_ACTUATOR_EN = 13;             // Front-right actuator driver enable
const int REAR_LEFT_ACTUATOR_EN = 20;               // Rear-left actuator driver enable
const int REAR_RIGHT_ACTUATOR_EN = 21;              // Rear-right actuator driver enable

const int BNO055_RESET_PIN = 26;                    // BNO055 hardware reset pin

const uint8_t FRONT_LEFT_WHEEL_LPWM = 0;            // PCA9685 wheel LPWM
const uint8_t FRONT_LEFT_WHEEL_RPWM = 1;            // PCA9685 wheel RPWM
const uint8_t FRONT_RIGHT_WHEEL_LPWM = 2;           // PCA9685 wheel LPWM
const uint8_t FRONT_RIGHT_WHEEL_RPWM = 3;           // PCA9685 wheel RPWM
const uint8_t REAR_LEFT_WHEEL_LPWM = 4;             // PCA9685 wheel LPWM
const uint8_t REAR_LEFT_WHEEL_RPWM = 5;             // PCA9685 wheel RPWM
const uint8_t REAR_RIGHT_WHEEL_LPWM = 6;            // PCA9685 wheel LPWM
const uint8_t REAR_RIGHT_WHEEL_RPWM = 7;            // PCA9685 wheel RPWM

const uint8_t FRONT_LEFT_ACTUATOR_LPWM = 8;         // PCA9685 actuator LPWM
const uint8_t FRONT_LEFT_ACTUATOR_RPWM = 9;         // PCA9685 actuator RPWM
const uint8_t FRONT_RIGHT_ACTUATOR_LPWM = 10;       // PCA9685 actuator LPWM
const uint8_t FRONT_RIGHT_ACTUATOR_RPWM = 11;       // PCA9685 actuator RPWM
const uint8_t REAR_LEFT_ACTUATOR_LPWM = 12;         // PCA9685 actuator LPWM
const uint8_t REAR_LEFT_ACTUATOR_RPWM = 13;         // PCA9685 actuator RPWM
const uint8_t REAR_RIGHT_ACTUATOR_LPWM = 14;        // PCA9685 actuator LPWM
const uint8_t REAR_RIGHT_ACTUATOR_RPWM = 15;        // PCA9685 actuator RPWM

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

struct Leg
{
  const char* name;
  int actuatorPotPin;
  int actuatorEnablePin;
  uint8_t actuatorLeftChannel;
  uint8_t actuatorRightChannel;
  int wheelEnablePin;
  uint8_t wheelLeftChannel;
  uint8_t wheelRightChannel;
  Potentiometer actuatorSensor;
  Pca9685Motor actuatorMotor;
  Pca9685Motor wheelMotor;
  WheelEncoder wheelEncoder;

  float actuatorCommand;
  float wheelCommand;
  float actuatorPosition;
  float wheelRevolutions;
  float wheelRadians;
  float wheelVelocity;

  Leg(
    const char* legName,
    Adafruit_PWMServoDriver& pwmDriver,
    int actuatorPotPin,
    int actuatorEnablePin,
    uint8_t actuatorLeftChannel,
    uint8_t actuatorRightChannel,
    int wheelEnablePin,
    uint8_t wheelLeftChannel,
    uint8_t wheelRightChannel,
    uint8_t wheelPinA,
    uint8_t wheelPinB,
    uint8_t wheelIndexPin):
    name(legName),
    actuatorPotPin(actuatorPotPin),
    actuatorEnablePin(actuatorEnablePin),
    actuatorLeftChannel(actuatorLeftChannel),
    actuatorRightChannel(actuatorRightChannel),
    wheelEnablePin(wheelEnablePin),
    wheelLeftChannel(wheelLeftChannel),
    wheelRightChannel(wheelRightChannel),
    actuatorMotor(pwmDriver),
    wheelMotor(pwmDriver),
    wheelEncoder(wheelPinA, wheelPinB, wheelIndexPin, ENCODER_CPR),
    actuatorCommand(0.0f),
    wheelCommand(0.0f),
    actuatorPosition(0.0f),
    wheelRevolutions(0.0f),
    wheelRadians(0.0f),
    wheelVelocity(0.0f)
  {
  }

  void initialize()
  {
    actuatorSensor.initialize(actuatorPotPin);
    actuatorMotor.initialize(actuatorEnablePin, actuatorLeftChannel, actuatorRightChannel);
    wheelMotor.initialize(wheelEnablePin, wheelLeftChannel, wheelRightChannel);
    actuatorSensor.read();
    wheelEncoder.initialize();
    stop();
  }

  void enable()
  {
    actuatorMotor.enable();
    wheelMotor.enable();
  }

  void disable()
  {
    actuatorMotor.disable();
    wheelMotor.disable();
  }

  void stop()
  {
    actuatorCommand = 0.0f;
    wheelCommand = 0.0f;
    actuatorMotor.stop();
    wheelMotor.stop();
  }

  void update(float timeStep)
  {
    actuatorPosition = actuatorSensor.read();
    wheelEncoder.update(timeStep);

    wheelRevolutions = wheelEncoder.revolutions;
    wheelRadians = wheelRevolutions * TWO_PI_F;
    wheelVelocity = wheelEncoder.velocity;

    actuatorMotor.write(actuatorCommand);
    wheelMotor.write(wheelCommand);
  }
};

/*----------------------------------------------------------*\
| Forward Declarations
\*----------------------------------------------------------*/

void initializeSerial();
void initializeI2c();
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
void frontLeftIndexInterrupt();
void frontRightIndexInterrupt();
void rearLeftIndexInterrupt();
void rearRightIndexInterrupt();

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

Adafruit_PWMServoDriver pwmDriver = Adafruit_PWMServoDriver(PCA9685_ADDRESS, Wire);
Adafruit_BNO055 imu(BNO055_ID, BNO055_ADDRESS, &Wire);

Leg frontLeft(
  "front_left",
  pwmDriver,
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

Leg frontRight(
  "front_right",
  pwmDriver,
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

Leg rearLeft(
  "rear_left",
  pwmDriver,
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

Leg rearRight(
  "rear_right",
  pwmDriver,
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

Leg* legs[] =
{
  &frontLeft,
  &frontRight,
  &rearLeft,
  &rearRight
};

const size_t LEG_COUNT = sizeof(legs) / sizeof(legs[0]);

bool imuReady = false;
bool motorsEnabled = false;

/*----------------------------------------------------------*\
| Entry Points
\*----------------------------------------------------------*/

void setup()
{
  initializeSerial();
  initializeI2c();
  initializePwmDriver();
  initializeImu();
  initializeLegs();
  initializeEncoders();
  printHelp();
}

void loop()
{
  static uint32_t lastUpdate = 0;
  static uint32_t lastReport = 0;

  uint32_t now = millis();
  const uint32_t updatePeriod = 1000 / RATE_HZ;
  const uint32_t reportPeriod = 1000 / REPORT_HZ;

  handleSerial();

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

  float timeStep = elapsed * MILLISECONDS_TO_SECONDS;

  if (timeStep <= 0.0f)
  {
    lastUpdate = now;
    return;
  }

  updateLegs(timeStep);
  lastUpdate = now;

  if ((now - lastReport) >= reportPeriod)
  {
    reportState();
    lastReport = now;
  }
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

  analogReadResolution(12);
  analogReadAveraging(8);
}

void initializeI2c()
{
  Wire.begin();
  Wire.setClock(400000);
}

void initializePwmDriver()
{
  pwmDriver.begin();
  pwmDriver.setPWMFreq(PCA9685_FREQUENCY_HZ);
}

void initializeImu()
{
  pinMode(BNO055_RESET_PIN, OUTPUT);
  digitalWrite(BNO055_RESET_PIN, LOW);
  delay(10);
  digitalWrite(BNO055_RESET_PIN, HIGH);
  delay(650);

  imuReady = imu.begin();

  if (imuReady)
  {
    imu.setExtCrystalUse(true);
  }

  Serial.print("imu ");
  Serial.println(imuReady ? "ready" : "not detected");
}

void initializeLegs()
{
  for (size_t index = 0; index < LEG_COUNT; ++index)
  {
    legs[index]->initialize();
  }

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
  for (size_t index = 0; index < LEG_COUNT; ++index)
  {
    legs[index]->enable();
  }

  motorsEnabled = true;
}

void disableMotors()
{
  for (size_t index = 0; index < LEG_COUNT; ++index)
  {
    legs[index]->disable();
  }

  motorsEnabled = false;
}

void stopMotors()
{
  for (size_t index = 0; index < LEG_COUNT; ++index)
  {
    legs[index]->stop();
  }
}

void updateLegs(float timeStep)
{
  for (size_t index = 0; index < LEG_COUNT; ++index)
  {
    legs[index]->update(timeStep);
  }
}

void reportState()
{
  Serial.print("motors ");
  Serial.println(motorsEnabled ? "enabled" : "disabled");

  for (size_t index = 0; index < LEG_COUNT; ++index)
  {
    reportLeg(*legs[index]);
  }

  reportImu();
  Serial.println();
}

void reportImu()
{
  if (!imuReady)
  {
    Serial.println("imu unavailable");
    return;
  }

  imu::Vector<3> linear = imu.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> gyro = imu.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> euler = imu.getVector(Adafruit_BNO055::VECTOR_EULER);

  Serial.print("imu euler_deg ");
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
  Serial.print(leg.wheelEncoder.counts);
  Serial.print(" wheel_rev ");
  Serial.print(leg.wheelRevolutions, 5);
  Serial.print(" wheel_rad ");
  Serial.print(leg.wheelRadians, 5);
  Serial.print(" wheel_rps ");
  Serial.print(leg.wheelVelocity, 5);
  Serial.print(" index_seen ");
  Serial.print(leg.wheelEncoder.indexCount);
  Serial.print(" index_state ");
  Serial.println(leg.wheelEncoder.indexState ? 1 : 0);
}

void handleSerial()
{
  while (Serial.available() > 0)
  {
    char command = char(Serial.read());

    switch (command)
    {
      case 'e':
        enableMotors();
        Serial.println("motors enabled");
        break;

      case 'd':
        disableMotors();
        Serial.println("motors disabled");
        break;

      case 's':
        stopMotors();
        Serial.println("motors stopped");
        break;

      case 'r':
        for (size_t index = 0; index < LEG_COUNT; ++index)
        {
          legs[index]->wheelEncoder.reset();
        }
        Serial.println("wheel encoders reset");
        break;

      case '?':
      case 'h':
        printHelp();
        break;

      default:
        break;
    }
  }
}

void printHelp()
{
  Serial.println("legs firmware");
  Serial.println("commands: e=enable d=disable s=stop r=reset_encoders h=help");
  Serial.println("telemetry reports actuator potentiometers, wheel encoders, and imu");
}

void frontLeftIndexInterrupt()
{
  frontLeft.wheelEncoder.handleIndexRise();
}

void frontRightIndexInterrupt()
{
  frontRight.wheelEncoder.handleIndexRise();
}

void rearLeftIndexInterrupt()
{
  rearLeft.wheelEncoder.handleIndexRise();
}

void rearRightIndexInterrupt()
{
  rearRight.wheelEncoder.handleIndexRise();
}
