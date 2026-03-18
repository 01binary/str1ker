
/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 meter.h
 Voltage/Current Metering
 Copyright (C) 2026 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <Adafruit_INA260.h>

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

// Multiplier to convert mA to A
const float MA_TO_A = 0.001;

// Multiplier to convert mV to V
const float MV_TO_V = 0.001;

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

Adafruit_INA260 ina260 = Adafruit_INA260();
bool enableVoltageCurrent = false;

/*----------------------------------------------------------*\
| Functions
\*----------------------------------------------------------*/

void initializeVoltageCurrentMeter()
{
  enableVoltageCurrent = ina260.begin();
}

float measureVoltage()
{
  if (!enableVoltageCurrent)
  {
    return 0.0;
  }

  return ina260.readBusVoltage() * MV_TO_V;
}

float measureCurrent()
{
  if (!enableVoltageCurrent)
  {
    return 0.0;
  }

  return ina260.readCurrent() * MA_TO_A;
}
