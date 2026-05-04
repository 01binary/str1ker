#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ShiftRegister74HC595.h>


const int I2C_FREQUENCY = 400000;

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

void setup()
{
    analogReadResolution(12);
    analogReadAveraging(8);

    Wire.begin();
    Wire.setClock(I2C_FREQUENCY);

    pinMode(HEAD_PAN_ENABLE, OUTPUT);
    digitalWrite(HEAD_PAN_ENABLE, HIGH);
    digitalWrite(HEAD_PAN_ENABLE, LOW);
}

void loop()
{
    delay(1000);
}
