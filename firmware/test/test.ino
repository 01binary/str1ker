#include <Wire.h>
#include <Encoder.h>

const int I2C_FREQUENCY_HZ = 400000;      // I2C bus frequency

const int LED_MUX_ADDRESS = 0x34;         // IS31FL3739 I2C address with AD tied to GND

const int FRONT_LEFT_ACTUATOR_POT = 16;   // Front-left actuator feedback pin
const int FRONT_LEFT_WHEEL_A = 32;        // Front-left wheel encoder A
const int FRONT_LEFT_WHEEL_B = 33;        // Front-left wheel encoder B
const int FRONT_LEFT_WHEEL_INDEX = 22;    // Front-left wheel encoder index
const int FRONT_LEFT_WHEEL_EN = 26;       // Front-left wheel driver enable
const int FRONT_LEFT_WHEEL_RPWM = 0;      // Front-left wheel RPWM
const int FRONT_LEFT_WHEEL_LPWM = 1;      // Front-left wheel LPWM
const int FRONT_LEFT_ACTUATOR_EN = 27;    // Front-left actuator driver enable
const int FRONT_LEFT_ACTUATOR_RPWM = 2;   // Front-left actuator RPWM
const int FRONT_LEFT_ACTUATOR_LPWM = 3;   // Front-left actuator LPWM

const int FRONT_RIGHT_ACTUATOR_POT = A17; // Front-right actuator feedback pin
const int FRONT_RIGHT_WHEEL_A = 34;       // Front-right wheel encoder A
const int FRONT_RIGHT_WHEEL_B = 35;       // Front-right wheel encoder B
const int FRONT_RIGHT_WHEEL_INDEX = 23;   // Front-right wheel encoder index
const int FRONT_RIGHT_WHEEL_EN = 28;      // Front-right wheel driver enable
const int FRONT_RIGHT_WHEEL_RPWM = 4;     // Front-right wheel RPWM
const int FRONT_RIGHT_WHEEL_LPWM = 5;     // Front-right wheel LPWM
const int FRONT_RIGHT_ACTUATOR_EN = 29;   // Front-right actuator driver enable
const int FRONT_RIGHT_ACTUATOR_RPWM = 6;  // Front-right actuator RPWM
const int FRONT_RIGHT_ACTUATOR_LPWM = 7;  // Front-right actuator LPWM

const int REAR_LEFT_ACTUATOR_POT = A2;    // Rear-left actuator feedback pin
const int REAR_LEFT_WHEEL_A = 36;         // Rear-left wheel encoder A
const int REAR_LEFT_WHEEL_B = 37;         // Rear-left wheel encoder B
const int REAR_LEFT_WHEEL_INDEX = 24;     // Rear-left wheel encoder index
const int REAR_LEFT_WHEEL_EN = 30;        // Rear-left wheel driver enable
const int REAR_LEFT_WHEEL_RPWM = 8;       // Rear-left wheel RPWM
const int REAR_LEFT_WHEEL_LPWM = 9;       // Rear-left wheel LPWM
const int REAR_LEFT_ACTUATOR_EN = 31;     // Rear-left actuator driver enable
const int REAR_LEFT_ACTUATOR_RPWM = 10;   // Rear-left actuator RPWM
const int REAR_LEFT_ACTUATOR_LPWM = 11;   // Rear-left actuator LPWM

const int REAR_RIGHT_ACTUATOR_POT = A3;   // Rear-right actuator feedback pin
const int REAR_RIGHT_WHEEL_A = 38;        // Rear-right wheel encoder A
const int REAR_RIGHT_WHEEL_B = 39;        // Rear-right wheel encoder B
const int REAR_RIGHT_WHEEL_INDEX = 25;    // Rear-right wheel encoder index
const int REAR_RIGHT_WHEEL_EN = 20;       // Rear-right wheel driver enable
const int REAR_RIGHT_WHEEL_RPWM = 12;     // Rear-right wheel RPWM
const int REAR_RIGHT_WHEEL_LPWM = 13;     // Rear-right wheel LPWM
const int REAR_RIGHT_ACTUATOR_EN = 21;    // Rear-right actuator driver enable
const int REAR_RIGHT_ACTUATOR_RPWM = 14;  // Rear-right actuator RPWM
const int REAR_RIGHT_ACTUATOR_LPWM = 15;  // Rear-right actuator LPWM

const int PWM_MAX = 4095;
const int PWM_HALF = PWM_MAX / 2;
const double ADC_MAX = 4095.0;

const int SERIAL_BAUD = 115200;
const int REPORT_PERIOD_MS = 100;

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

void writeDisplay(uint8_t display, uint8_t tenths)
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

  analogWrite(FRONT_LEFT_WHEEL_LPWM, PWM_HALF);
  analogWrite(FRONT_LEFT_WHEEL_RPWM, PWM_HALF);
  analogWrite(FRONT_LEFT_ACTUATOR_LPWM, PWM_HALF);
  analogWrite(FRONT_LEFT_ACTUATOR_RPWM, PWM_HALF);

  analogWrite(FRONT_RIGHT_WHEEL_LPWM, PWM_HALF);
  analogWrite(FRONT_RIGHT_WHEEL_RPWM, PWM_HALF);
  analogWrite(FRONT_RIGHT_ACTUATOR_LPWM, PWM_HALF);
  analogWrite(FRONT_RIGHT_ACTUATOR_RPWM, PWM_HALF);

  analogWrite(REAR_LEFT_WHEEL_LPWM, PWM_HALF);
  analogWrite(REAR_LEFT_WHEEL_RPWM, PWM_HALF);
  analogWrite(REAR_LEFT_ACTUATOR_LPWM, PWM_HALF);
  analogWrite(REAR_LEFT_ACTUATOR_RPWM, PWM_HALF);

  analogWrite(REAR_RIGHT_WHEEL_LPWM, PWM_HALF);
  analogWrite(REAR_RIGHT_WHEEL_RPWM, PWM_HALF);
  analogWrite(REAR_RIGHT_ACTUATOR_LPWM, PWM_HALF);
  analogWrite(REAR_RIGHT_ACTUATOR_RPWM, PWM_HALF);

  Wire.begin();
  Wire.setClock(I2C_FREQUENCY_HZ);

  initializeWheelDisplays();
  writeDisplay(0, DISPLAY_0_START_TENTHS);
  writeDisplay(1, DISPLAY_1_START_TENTHS);
  writeDisplay(2, DISPLAY_2_START_TENTHS);
  writeDisplay(3, DISPLAY_3_START_TENTHS);

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
  static uint32_t lastDisplayUpdate = 0;
  static uint8_t display0Tenths = DISPLAY_0_START_TENTHS;
  static uint8_t display1Tenths = DISPLAY_1_START_TENTHS;
  static uint8_t display2Tenths = DISPLAY_2_START_TENTHS;
  static uint8_t display3Tenths = DISPLAY_3_START_TENTHS;

  if ((millis() - lastDisplayUpdate) >= DISPLAY_UPDATE_PERIOD_MS)
  {
    lastDisplayUpdate = millis();

    display0Tenths = (display0Tenths + 1) % 11;
    display1Tenths = (display1Tenths + 1) % 11;
    display2Tenths = (display2Tenths + 1) % 11;
    display3Tenths = (display3Tenths + 1) % 11;

    writeDisplay(0, display0Tenths);
    writeDisplay(1, display1Tenths);
    writeDisplay(2, display2Tenths);
    writeDisplay(3, display3Tenths);
  }

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
