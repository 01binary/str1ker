#include <SPI.h>

// PWM
const int PWM1_EN_PIN = 0;   // D0
const int PWM2_EN_PIN = 6;   // example
const int PWM3_EN_PIN = 21;  // example
const int LPWM1_PIN = 3;
const int RPWM1_PIN = 1;
const int LPWM2_PIN = 8;
const int RPWM2_PIN = 7;
const int LPWM3_PIN = 23;
const int RPWM3_PIN = 22;
const int PWM_VALUE = 128;  // ~50% duty

// Solenoid
const int SOL = 17;

// Encoder
const int ENCODER_STATUS = 9;
const int CS_PIN = 10;
const int CLOCK_PIN = 13;
const int DATA_PIN = 12;
const int BIT_COUNT = 12;

SPISettings as5045SPI(1000000, MSBFIRST, SPI_MODE1); // 1 MHz to start

void setup() {
  pinMode(PWM1_EN_PIN, OUTPUT);
  pinMode(PWM2_EN_PIN, OUTPUT);
  pinMode(PWM3_EN_PIN, OUTPUT);
  pinMode(LPWM1_PIN, OUTPUT);
  pinMode(RPWM1_PIN, OUTPUT);
  pinMode(LPWM2_PIN, OUTPUT);
  pinMode(RPWM2_PIN, OUTPUT);
  pinMode(LPWM3_PIN, OUTPUT);
  pinMode(RPWM3_PIN, OUTPUT);
  pinMode(ENCODER_STATUS, OUTPUT);
  pinMode(SOL, OUTPUT);

  digitalWrite(PWM1_EN_PIN, LOW);
  digitalWrite(PWM2_EN_PIN, LOW);
  digitalWrite(PWM3_EN_PIN, LOW);

  analogWriteFrequency(LPWM1_PIN, 20000);
  analogWriteFrequency(RPWM1_PIN, 20000);
  analogWriteFrequency(LPWM2_PIN, 20000);
  analogWriteFrequency(RPWM2_PIN, 20000);
  analogWriteFrequency(LPWM3_PIN, 20000);
  analogWriteFrequency(RPWM3_PIN, 20000);

  analogWrite(RPWM1_PIN, 0);
  analogWrite(LPWM1_PIN, 0);

  analogWrite(RPWM2_PIN, 0);
  analogWrite(LPWM2_PIN, 0);

  analogWrite(RPWM3_PIN, 0);
  analogWrite(LPWM3_PIN, 0);

  digitalWrite(SOL, LOW);

  digitalWrite(ENCODER_STATUS, HIGH);
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  SPI.begin();  // uses hardware pins: SCK=13, MISO=12, MOSI=11 (MOSI unused here)

  Serial.begin(115200);
}

union AS5045Reading
{
  struct
  {
    uint16_t status   : 4;
    uint16_t position : 12;
  };

  uint16_t data;
};

void loop() {
  /*
  // Select
  digitalWrite(CS_PIN, LOW);

  // Read
  SPI.beginTransaction(SPISettings(
    1e6,
    MSBFIRST,
    SPI_MODE0
  ));

  AS5045Reading reading;
  reading.data = SPI.transfer16(0);

  SPI.endTransaction();

  // Deselect
  digitalWrite(CS_PIN, HIGH);

  // Debug
  float angle = float(reading.position) / 4096.0f;

  Serial.print("angle ");
  Serial.print(angle);
  Serial.println("");

  delay(50);
  */

  float value = float(analogRead(A0)) / 1024.0f;
  Serial.print("pot ");
  Serial.println(value);
}
