#include <Adafruit_PWMServoDriver.h>

const int PCA9685_ADDRESS = 0x40;         // Default PCA9685 I2C address
const int PCA9685_FREQUENCY_HZ = 1526;    // Near PCA9685 max, suitable for IBT2 PWM inputs

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS, Wire);

// Any 1-4 enabled pins high (red LEDs on): works fine
// Any 5 enabled pins high (red LEDs on): resets Teensy every 1000ms, causing red LEDs to blink at that rate
// Any 6 enabled pins high: resets teensy every 700ms, causing red LEDs to blink at that rate
// Any 7 enabled pins high: resets teensy every 500ms, causing red LEDs to blink at that rate
// All 8 enabled pins high: strobes LEDs and makes them dim, presumably resetting Teensy many times per second

void setup()
{
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(20, OUTPUT);
  pinMode(21, OUTPUT);

  digitalWrite(6, HIGH);
  digitalWrite(9, HIGH);
  digitalWrite(10, HIGH);
  digitalWrite(11, HIGH);
  digitalWrite(12, HIGH);
  digitalWrite(13, HIGH);
  digitalWrite(20, HIGH);
  digitalWrite(21, LOW);

  pwm.begin();
  pwm.setPWMFreq(PCA9685_FREQUENCY_HZ);
  pwm.setPWM(0, 0, 0);
  pwm.setPWM(1, 0, 0);
  pwm.setPWM(2, 0, 0);
  pwm.setPWM(3, 0, 0);
  pwm.setPWM(4, 0, 0);
  pwm.setPWM(5, 0, 0);
  pwm.setPWM(6, 0, 0);
  pwm.setPWM(7, 0, 0);
  pwm.setPWM(8, 0, 0);
  pwm.setPWM(9, 0, 0);
  pwm.setPWM(10, 0, 0);
  pwm.setPWM(11, 0, 0);
  pwm.setPWM(12, 0, 0);
  pwm.setPWM(13, 0, 0);
  pwm.setPWM(14, 0, 0);
  pwm.setPWM(15, 0, 0);
}

void loop()
{
  delay(1000);
}
