/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 encoder.h

 Absolute encoder class
 Created 09/20/2023

 Copyright (C) 2023 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros/ros.h>
#include <str1ker/Adc.h>
#include "controller.h"
#include "filter.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| encoder class
\*----------------------------------------------------------*/

class encoder : public controller
{
public:
  // Controller type
  static const char TYPE[];

private:
  // Queue size for subscribers and publishers
  const int QUEUE_SIZE = 16;

  // Defaults for analog input range
  const int ANALOG_MIN = 0;
  const int ANALOG_MAX = 1023;

  // Defaults for analog input filtering
  const int DEFAULT_THRESHOLD = 8;
  const int DEFAULT_AVERAGE = 16;

  // Defaults mapping analog value to joint state
  const double POS_MIN = 0.0;
  const double POS_MAX = 1.0;

private:
  //
  // Configuration
  //

  // Input topic for listening to analog readings
  std::string m_topic = "adc";

  // Analog input channel
  int m_channel;

  // Analog reading min
  int m_minReading = ANALOG_MIN;

  // Analog reading max
  int m_maxReading = ANALOG_MAX;

  // Position (joint state) min
  double m_minPos = POS_MIN;

  // Position (joint state) max
  double m_maxPos = POS_MAX;

  //
  // State
  //

  bool m_ready = false;

  // Last filtered analog reading
  int m_reading = -1;

  // Last position mapped from last reading
  double m_position = std::numeric_limits<double>::infinity();

  // Whether analog input filtering is enabled
  bool m_enableFilter = false;

  // Filter for analog input
  filter m_filter;

  // Subscriber for receiving analog readings from motor absolute encoder
  ros::Subscriber m_sub;

public:
  //
  // Constructors
  //

  encoder(ros::NodeHandle node, std::string path);

  encoder(
    ros::NodeHandle node,
    std::string path,
    std::string topic,
    int input,
    int minReading,
    int maxReading,
    double minPos,
    double maxPos,
    int filterThreshold,
    int filterAverage);

public:
  // Get current filtered analog reading
  inline int getReading() const
  {
    return m_reading;
  }

  // Get current position mapped from filtered analog reading
  inline double getPos() const
  {
    return m_position;
  }

  // Get minimum position
  inline double getMin() const
  {
    return m_minPos;
  }

  // Get maximum position
  inline double getMax() const
  {
    return m_maxPos;
  }

  // Determine if the encoder is ready to provide readings
  bool isReady() const
  {
    return m_ready;
  }

  // Configuration
  virtual bool configure();

  // Initialization
  virtual bool init();

  // Analog reading feedback
  void feedback(const Adc::ConstPtr& msg);

public:
  // Create instance
  static controller* create(ros::NodeHandle node, std::string path);
};

} // namespace str1ker
