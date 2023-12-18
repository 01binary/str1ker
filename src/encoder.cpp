/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 encoder.cpp

 Absolute Encoder Implementation
 Created 09/20/2023

 Copyright (C) 2023 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include "robot.h"
#include "controllerFactory.h"
#include "utilities.h"
#include "encoder.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace str1ker;
using namespace std;

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const char encoder::TYPE[] = "encoder";

/*----------------------------------------------------------*\
| encoder implementation
\*----------------------------------------------------------*/

REGISTER_CONTROLLER(encoder);

//
// Constructors
//

encoder::encoder(class robot& robot, const char* path)
  : controller(robot, path)
  , m_filter(DEFAULT_THRESHOLD, DEFAULT_AVERAGE)
{
}

encoder::encoder(
  class robot& robot,
  const char* path,
  const std::string& topic,
  int input,
  int minReading,
  int maxReading,
  double minPos,
  double maxPos,
  int filterThreshold,
  int filterAverage)
  : controller(robot, path)
  , m_topic(topic)
  , m_filter(filterThreshold, filterAverage)
  , m_channel(input)
  , m_minReading(minReading)
  , m_maxReading(maxReading)
  , m_minPos(minPos)
  , m_maxPos(maxPos)
  , m_ready(false)
{
}

//
// Configuration
//

void encoder::configure(ros::NodeHandle node)
{
  controller::configure(node);

  ros::param::get("topic", m_topic);
  ros::param::get("channel", m_channel);
  ros::param::get("minReading", m_minReading);
  ros::param::get("maxReading", m_maxReading);
  ros::param::get("minPos", m_minPos);
  ros::param::get("maxPos", m_maxPos);

  int threshold = DEFAULT_THRESHOLD, average = DEFAULT_AVERAGE;
  ros::param::get("threshold", threshold);
  ros::param::get("average", average);

  m_filter = filter(threshold, average);
}

//
// Initialization
//

bool encoder::init(ros::NodeHandle& node)
{
  // Subscribe to analog readings
  m_sub = node.subscribe<Adc>(
    m_topic, QUEUE_SIZE, &encoder::feedback, this);

  ROS_INFO("  initialized %s %s on %s channel %d range [%d, %d] -> [%g, %g]",
    getPath(),
    getType(),
    m_topic.c_str(),
    m_channel,
    m_minReading,
    m_maxReading,
    m_minPos,
    m_maxPos);

  return true;
}

//
// Feedback
//

void encoder::feedback(const Adc::ConstPtr& msg)
{
  // Read analog input
  m_reading = m_filter(msg->adc[m_channel]);

  // Re-map to position
  m_position = utilities::map(
    (double)m_reading,
    (double)m_minReading,
    (double)m_maxReading,
    m_minPos,
    m_maxPos);

  // Check if the position is ready to be used
  if (!m_ready)
  {
    m_ready = m_filter.isReady();
  }
}

//
// Dynamic creation
//

controller* encoder::create(robot& robot, const char* path)
{
  return new encoder(robot, path);
}
