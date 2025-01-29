/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <EEPROMex.h>

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

int rate = 50;                      // Control loop rate (Hz)

double baseKp = 1.0;                // Base joint proportional gain
double baseKi = 0.1;                // Base joint integral gain
double baseKd = 0.1;                // Base joint derivative gain
double baseIMin = -1.0;             // Base joint integral minimum
double baseIMax = 1.0;              // Base joint integral maximum
double baseTolerance = 0.05;        // Base joint tolerance
double baseEncoderMin = 0.0;        // Base joint encoder minimum
double baseEncoderMax = 0.0;        // Base joint encoder maximum
double basePwmMin = 0.0;            // Base joint PWM minimum
double basePwmMax = 0.0;            // Base joint PWM maximum
double baseStallThreshold = 0.0;    // Base joint stall threshold
bool baseEncoderInvert = false;     // Base joint encoder invert
bool baseQuadratureInvert = false;  // Base joint quadrature invert
bool basePwmInvert = false;         // Base joint PWM invert

double shoulderKp = 1.0;            // Shoulder joint proportional gain
double shoulderKi = 0.1;            // Shoulder joint integral gain
double shoulderKd = 0.1;            // Shoulder joint derivative gain
double shoulderIMin = -1.0;         // Shoulder joint integral minimum
double shoulderIMax = 1.0;          // Shoulder joint integral maximum
double shoulderTolerance = 0.05;    // Shoulder joint tolerance
double shoulderEncoderMin = 0.0;    // Shoulder joint encoder minimum
double shoulderEncoderMax = 0.0;    // Shoulder joint encoder maximum
double shoulderPwmMin = 0.0;        // Shoulder joint PWM minimum
double shoulderPwmMax = 0.0;        // Shoulder joint PWM maximum
double shoulderStallThreshold = 0.0;// Shoulder joint stall threshold
bool shoulderEncoderInvert = false; // Shoulder joint encoder invert
bool shoulderPwmInvert = false;     // Shoulder joint PWM invert

double elbowKp = 1.0;               // Elbow joint proportional gain
double elbowKi = 0.1;               // Elbow joint integral gain
double elbowKd = 0.1;               // Elbow joint derivative gain
double elbowIMin = -1.0;            // Elbow joint integral minimum
double elbowIMax = 1.0;             // Elbow joint integral maximum
double elbowTolerance = 0.05;       // Elbow joint tolerance
double elbowEncoderMin = 0.0;       // Elbow joint encoder minimum
double elbowEncoderMax = 0.0;       // Elbow joint encoder maximum
double elbowPwmMin = 0.0;           // Elbow joint PWM minimum
double elbowPwmMax = 0.0;           // Elbow joint PWM maximum
double elbowStallThreshold = 0.0;   // Elbow joint stall threshold
bool elbowEncoderInvert = false;    // Elbow joint encoder invert
bool elbowPwmInvert = false;        // Elbow joint PWM invert

/*----------------------------------------------------------*\
| Functions
\*----------------------------------------------------------*/

void initializeSettings()
{
  EEPROM.setMemPool(memBase, EEPROMSizeUno);
  EEPROM.setMaxAllowedWrites(maxAllowedWrites);
}

void readSettings()
{
  rate = EEPROM.readInt("rate", rate);

  baseKp = EEPROM.readDouble("baseKp", baseKp);
  baseKi = EEPROM.readDouble("baseKi", baseKi);
  baseKd = EEPROM.readDouble("baseKd", baseKd);
  baseIMin = EEPROM.readDouble("baseIMin", baseIMin);
  baseIMax = EEPROM.readDouble("baseIMax", baseIMax);
  baseTolerance = EEPROM.readDouble("baseTolerance", baseTolerance);
  baseEncoderMin = EEPROM.readDouble("baseEncoderMin", baseEncoderMin);
  baseEncoderMax = EEPROM.readDouble("baseEncoderMax", baseEncoderMax);
  basePwmMin = EEPROM.readDouble("basePwmMin", basePwmMin);
  basePwmMax = EEPROM.readDouble("basePwmMax", basePwmMax);
  baseStallThreshold = EEPROM.readDouble("baseStallThreshold", baseStallThreshold);
  baseEncoderInvert = EEPROM.readBool("baseEncoderInvert", baseEncoderInvert);
  baseQuadratureInvert = EEPROM.readBool("baseQuadratureInvert", baseQuadratureInvert);
  basePwmInvert = EEPROM.readBool("basePwmInvert", basePwmInvert);

  shoulderKp = EEPROM.readDouble("shoulderKp", shoulderKp);
  shoulderKi = EEPROM.readDouble("shoulderKi", shoulderKi);
  shoulderKd = EEPROM.readDouble("shoulderKd", shoulderKd);
  shoulderIMin = EEPROM.readDouble("shoulderIMin", shoulderIMin);
  shoulderIMax = EEPROM.readDouble("shoulderIMax", shoulderIMax);
  shoulderTolerance = EEPROM.readDouble("shoulderTolerance", shoulderTolerance);
  shoulderEncoderMin = EEPROM.readDouble("shoulderEncoderMin", shoulderEncoderMin);
  shoulderEncoderMax = EEPROM.readDouble("shoulderEncoderMax", shoulderEncoderMax);
  shoulderPwmMin = EEPROM.readDouble("shoulderPwmMin", shoulderPwmMin);
  shoulderPwmMax = EEPROM.readDouble("shoulderPwmMax", shoulderPwmMax);
  shoulderStallThreshold = EEPROM.readDouble("shoulderStallThreshold", shoulderStallThreshold);
  shoulderEncoderInvert = EEPROM.readBool("shoulderEncoderInvert", shoulderEncoderInvert);
  shoulderPwmInvert = EEPROM.readBool("shoulderPwmInvert", shoulderPwmInvert);

  elbowKp = EEPROM.readDouble("elbowKp", elbowKp);
  elbowKi = EEPROM.readDouble("elbowKi", elbowKi);
  elbowKd = EEPROM.readDouble("elbowKd", elbowKd);
  elbowIMin = EEPROM.readDouble("elbowIMin", elbowIMin);
  elbowIMax = EEPROM.readDouble("elbowIMax", elbowIMax);
  elbowTolerance = EEPROM.readDouble("elbowTolerance", elbowTolerance);
  elbowEncoderMin = EEPROM.readDouble("elbowEncoderMin", elbowEncoderMin);
  elbowEncoderMax = EEPROM.readDouble("elbowEncoderMax", elbowEncoderMax);
  elbowPwmMin = EEPROM.readDouble("elbowPwmMin", elbowPwmMin);
  elbowPwmMax = EEPROM.readDouble("elbowPwmMax", elbowPwmMax);
  elbowStallThreshold = EEPROM.readDouble("elbowStallThreshold", elbowStallThreshold);
  elbowEncoderInvert = EEPROM.readBool("elbowEncoderInvert", elbowEncoderInvert);
  elbowPwmInvert = EEPROM.readBool("elbowPwmInvert", elbowPwmInvert);

  preferences.end();
}

void writeSettings()
{
  EEPROM.writeInt("rate", rate);

  EEPROM.writeDouble("baseKp", baseKp);
  EEPROM.writeDouble("baseKi", baseKi);
  EEPROM.writeDouble("baseKd", baseKd);
  EEPROM.writeDouble("baseIMin", baseIMin);
  EEPROM.writeDouble("baseIMax", baseIMax);
  EEPROM.writeDouble("baseTolerance", baseTolerance);
  EEPROM.writeDouble("baseEncoderMin", baseEncoderMin);
  EEPROM.writeDouble("baseEncoderMax", baseEncoderMax);
  EEPROM.writeDouble("basePwmMin", basePwmMin);
  EEPROM.writeDouble("basePwmMax", basePwmMax);
  EEPROM.writeDouble("baseStallThreshold", baseStallThreshold);
  EEPROM.writeBool("baseEncoderInvert", baseEncoderInvert);
  EEPROM.writeBool("baseQuadratureInvert", baseQuadratureInvert);
  EEPROM.writeBool("basePwmInvert", basePwmInvert);

  EEPROM.writeDouble("shoulderKp", shoulderKp);
  EEPROM.writeDouble("shoulderKi", shoulderKi);
  EEPROM.writeDouble("shoulderKd", shoulderKd);
  EEPROM.writeDouble("shoulderIMin", shoulderIMin);
  EEPROM.writeDouble("shoulderIMax", shoulderIMax);
  EEPROM.writeDouble("shoulderTolerance", shoulderTolerance);
  EEPROM.writeDouble("shoulderEncoderMin", shoulderEncoderMin);
  EEPROM.writeDouble("shoulderEncoderMax", shoulderEncoderMax);
  EEPROM.writeDouble("shoulderPwmMin", shoulderPwmMin);
  EEPROM.writeDouble("shoulderPwmMax", shoulderPwmMax);
  EEPROM.writeDouble("shoulderStallThreshold", shoulderStallThreshold);
  EEPROM.writeBool("shoulderEncoderInvert", shoulderEncoderInvert);
  EEPROM.writeBool("shoulderPwmInvert", shoulderPwmInvert);

  EEPROM.writeDouble("elbowKp", elbowKp);
  EEPROM.writeDouble("elbowKi", elbowKi);
  EEPROM.writeDouble("elbowKd", elbowKd);
  EEPROM.writeDouble("elbowIMin", elbowIMin);
  EEPROM.writeDouble("elbowIMax", elbowIMax);
  EEPROM.writeDouble("elbowTolerance", elbowTolerance);
  EEPROM.writeDouble("elbowEncoderMin", elbowEncoderMin);
  EEPROM.writeDouble("elbowEncoderMax", elbowEncoderMax);
  EEPROM.writeDouble("elbowPwmMin", elbowPwmMin);
  EEPROM.writeDouble("elbowPwmMax", elbowPwmMax);
  EEPROM.writeDouble("elbowStallThreshold", elbowStallThreshold);
  EEPROM.writeBool("elbowEncoderInvert", elbowEncoderInvert);
  EEPROM.writeBool("elbowPwmInvert", elbowPwmInvert);
}