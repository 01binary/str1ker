/*
                                                                                     @@@@@@@                  
 @@@@@@@@@@@@  @@@@@@@@@@@@   @@@@@@@@@@@@       @  @@@@@@@@@@@@@  @           @  @@@       @@@  @@@@@@@@@@@@ 
@              @ @           @            @    @ @  @              @        @@@      @@@@@@@    @            @
 @@@@@@@@@@@@  @   @         @@@@@@@@@@@@@   @   @   @             @   @@@@@      @@@       @@@ @@@@@@@@@@@@@ 
             @ @     @       @            @      @    @@           @@@@      @                  @            @
 @@@@@@@@@@@@  @       @     @            @      @      @@@@@@@@@  @          @   @@@       @@@ @            @
                                                                                     @@@@@@@                  
 analogPotentiometer.h

 Analog Potentiometer Controller
 Created 1/28/2021

 This software is licensed under GNU GPLv3
*/

#ifndef STR1KER_ANALOG_POTENTIOMETER_H
#define STR1KER_ANALOG_POTENTIOMETER_H

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <string>
#include <vector>
#include "potentiometer.h"
#include "adc.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| potentiometer class
\*----------------------------------------------------------*/

class analogPotentiometer : public potentiometer
{
public:
    // Controller type
    static const char TYPE[];

private:
    // Number of samples to average for de-noising
    const int SAMPLE_COUNT = 8;

    // Number of averages to analyze
    const int AVG_COUNT = 4;

    // Threshold for picking stable average
    const double AVG_THRESHOLD = 0.2;

    // Analog to digital converter (ADC) for reading measurements
    adc* m_adc;

    // Channel index to use when reading from ADC
    int m_id;

    // Samples collected
    std::vector<double> m_samples;

    // Current sample index
    int m_sampleId;

    // Averages collected
    std::vector<double> m_avg;

    // Current average index
    int m_avgId;

    // Last reading
    double m_lastSample;

public:
    analogPotentiometer(const char* path);

public:
    // Get display type
    virtual const char* getType();

    // Initialize potentiometer controller
    virtual bool init();

    // Get absolute position
    virtual double getPos();

    // Deserialize from settings
    virtual void deserialize();

public:
    // Create instance
    static controller* create(const char* path);
};

} // namespace str1ker

#endif // STR1KER_ANALOG_POTENTIOMETER_H
