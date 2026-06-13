#include <Adafruit_PWMServoDriver.h>
#include <Encoder.h>

const int I2C_FREQUENCY_HZ = 400000;      // I2C bus frequency

const int PCA9685_ADDRESS = 0x40;         // Default PCA9685 I2C address
const int PCA9685_FREQUENCY_HZ = 1526;    // Near PCA9685 max, suitable for IBT2 PWM inputs
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

const int PWM_MAX = 4095;
const double ADC_MAX = 4095.0;

const int SERIAL_BAUD = 115200;
const int REPORT_PERIOD_MS = 100;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS, Wire);
Encoder wheelEncoder(FRONT_RIGHT_WHEEL_A, FRONT_RIGHT_WHEEL_B);
volatile uint32_t frontLeftIndexRevolutions = 0;

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

void frontLeftIndexInterrupt()
{
  frontLeftIndexRevolutions++;
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

void setup()
{
  analogReadResolution(12);
  analogReadAveraging(8);

  pinMode(FRONT_LEFT_WHEEL_EN, OUTPUT);
  pinMode(FRONT_LEFT_ACTUATOR_EN, OUTPUT);
  pinMode(FRONT_RIGHT_WHEEL_EN, OUTPUT);
  pinMode(FRONT_RIGHT_ACTUATOR_EN, OUTPUT);
  pinMode(REAR_LEFT_WHEEL_EN, OUTPUT);
  pinMode(REAR_LEFT_ACTUATOR_EN, OUTPUT);
  pinMode(REAR_RIGHT_WHEEL_EN, OUTPUT);
  pinMode(REAR_RIGHT_ACTUATOR_EN, OUTPUT);

  digitalWrite(FRONT_LEFT_WHEEL_EN, HIGH);
  digitalWrite(FRONT_LEFT_ACTUATOR_EN, HIGH);
  digitalWrite(FRONT_RIGHT_WHEEL_EN, HIGH);
  digitalWrite(FRONT_RIGHT_ACTUATOR_EN, HIGH);
  digitalWrite(REAR_LEFT_WHEEL_EN, HIGH);
  digitalWrite(REAR_LEFT_ACTUATOR_EN, HIGH);
  digitalWrite(REAR_RIGHT_WHEEL_EN, HIGH);
  digitalWrite(REAR_RIGHT_ACTUATOR_EN, HIGH);

  Wire.begin();
  Wire.setClock(I2C_FREQUENCY_HZ);
  initializeWheelDisplays();

  pwm.begin();
  pwm.setPWMFreq(PCA9685_FREQUENCY_HZ);
  pwm.setPin(FRONT_LEFT_WHEEL_RPWM, PWM_MAX);
  pwm.setPin(FRONT_LEFT_WHEEL_LPWM, PWM_MAX);
  pwm.setPin(FRONT_LEFT_ACTUATOR_RPWM, PWM_MAX);
  pwm.setPin(FRONT_LEFT_ACTUATOR_LPWM, PWM_MAX);
  pwm.setPin(FRONT_RIGHT_WHEEL_RPWM, PWM_MAX);
  pwm.setPin(FRONT_RIGHT_WHEEL_LPWM, PWM_MAX);
  pwm.setPin(FRONT_RIGHT_ACTUATOR_RPWM, PWM_MAX);
  pwm.setPin(FRONT_RIGHT_ACTUATOR_LPWM, PWM_MAX);
  pwm.setPin(REAR_LEFT_WHEEL_RPWM, PWM_MAX);
  pwm.setPin(REAR_LEFT_WHEEL_LPWM, PWM_MAX);
  pwm.setPin(REAR_LEFT_ACTUATOR_RPWM, PWM_MAX);
  pwm.setPin(REAR_LEFT_ACTUATOR_LPWM, PWM_MAX);
  pwm.setPin(REAR_RIGHT_WHEEL_RPWM, PWM_MAX);
  pwm.setPin(REAR_RIGHT_WHEEL_LPWM, PWM_MAX);
  pwm.setPin(REAR_RIGHT_ACTUATOR_RPWM, PWM_MAX);
  pwm.setPin(REAR_RIGHT_ACTUATOR_LPWM, PWM_MAX);

  pinMode(FRONT_RIGHT_WHEEL_A, INPUT);
  pinMode(FRONT_RIGHT_WHEEL_B, INPUT);
  pinMode(FRONT_RIGHT_WHEEL_INDEX, INPUT);
  wheelEncoder.write(0);
  attachInterrupt(digitalPinToInterrupt(FRONT_RIGHT_WHEEL_INDEX), frontLeftIndexInterrupt, RISING);

  Serial.begin(SERIAL_BAUD);
  Serial.println("front_left_encoder_test");
  Serial.println("pulses revolutions_from_index");
}

void loop()
{
  static uint32_t lastReport = 0;

  if ((millis() - lastReport) >= REPORT_PERIOD_MS)
  {
    lastReport = millis();

    const int32_t pulses = wheelEncoder.read();

    noInterrupts();
    const uint32_t revolutionsFromIndex = frontLeftIndexRevolutions;
    interrupts();

    Serial.print(pulses);
    Serial.print(" ");
    Serial.println(revolutionsFromIndex);
  }

  delay(1000);
}
