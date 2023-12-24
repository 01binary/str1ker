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
#include "hardwareUtilities.h"
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

encoder::encoder(ros::NodeHandle node, string path)
  : controller(node, TYPE, path)
  , m_filter(DEFAULT_THRESHOLD, DEFAULT_AVERAGE)
{
}

encoder::encoder(
  ros::NodeHandle node,
  string path,
  string topic,
  int input,
  int minReading,
  int maxReading,
  double minPos,
  double maxPos,
  int filterThreshold,
  int filterAverage)
  : controller(node, TYPE, path)
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

bool encoder::configure()
{
  controller::configure();

  if (!ros::param::get(getChildPath("topic"), m_topic))
    ROS_WARN("%s did not specify ADC input topic, using %s", getPath().c_str(), m_topic.c_str());

  if (!ros::param::get(getChildPath("channel"), m_channel))
    ROS_WARN("%s did not specify ADC input channel, using %d", getPath().c_str(), m_channel);

  if (!ros::param::get(getChildPath("minReading"), m_minReading))
    ROS_WARN("%s did not specify minReading value, using %d", getPath().c_str(), m_minReading);

  if (!ros::param::get(getChildPath("maxReading"), m_maxReading))
    ROS_WARN("%s did not specify maxReading value, using %d", getPath().c_str(), m_maxReading);

  if (!ros::param::get(getChildPath("minPos"), m_minPos))
    ROS_WARN("%s did not specify minPos value, using %g", getPath().c_str(), m_minPos);

  if (!ros::param::get(getChildPath("maxPos"), m_maxPos))
    ROS_WARN("%s did not specify maxPos value, using %g", getPath().c_str(), m_maxPos);

  int threshold = DEFAULT_THRESHOLD, average = DEFAULT_AVERAGE;

  if (!ros::param::get(getChildPath("threshold"), threshold))
    ROS_WARN("%s did not specify sample threshold, using %d", getPath().c_str(), DEFAULT_THRESHOLD);

  if (!ros::param::get(getChildPath("average"), average))
    ROS_WARN("%s did not specify how many samples to average, using %d", getPath().c_str(), DEFAULT_AVERAGE);

  m_filter = filter(threshold, average);

  return true;
}

//
// Initialization
//

bool encoder::init()
{
  // Subscribe to analog readings
  m_sub = m_node.subscribe<Adc>(
    m_topic, QUEUE_SIZE, &encoder::feedback, this);

  ROS_INFO("  initialized %s %s on %s channel %d: [%d, %d] -> [%g, %g]",
    getPath().c_str(),
    getType().c_str(),
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

controller* encoder::create(ros::NodeHandle node, string path)
{
  return new encoder(node, path);
}
