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
  int absoluteInput,
  int quadratureInput,
  int minReading,
  int maxReading,
  double minPos,
  double maxPos,
  int filterThreshold,
  int filterAverage)
  : controller(node, TYPE, path)
  , m_topic(topic)
  , m_filter(filterThreshold, filterAverage)
  , m_absoluteChannel(absoluteInput)
  , m_quadratureChannel(quadratureInput)
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

  if (!ros::param::get(getChildPath("absoluteChannel"), m_absoluteChannel))
    ROS_WARN("%s did not specify absolute input channel, using %d", getPath().c_str(), m_absoluteChannel);

  if (!ros::param::get(getChildPath("quadratureChannel"), m_quadratureChannel))
    ROS_INFO("%s did not specify quadrature input channel, offset tracking disabled", getPath().c_str());

  if (!ros::param::get(getChildPath("quadratureRange"), m_quadratureRange) && m_quadratureChannel != -1)
    ROS_ERROR("%s did not specify quadrature range corresponding to absolute range", getPath().c_str());

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
  // Subscribe to absolute and quadrature readings
  m_sub = m_node.subscribe<Adc>(
    m_topic, QUEUE_SIZE, &encoder::feedback, this);

  if (m_quadratureChannel == -1)
  {
    ROS_INFO("  initialized %s %s on %s channel %d: [%d, %d] -> [%g, %g]",
      getPath().c_str(),
      getType().c_str(),
      m_topic.c_str(),
      m_absoluteChannel,
      m_minReading,
      m_maxReading,
      m_minPos,
      m_maxPos);
  }
  else
  {
    ROS_INFO("  initialized %s %s on %s absolute channel %d / quadrature channel %d: [%d, %d] -> [%g, %g]",
      getPath().c_str(),
      getType().c_str(),
      m_topic.c_str(),
      m_absoluteChannel,
      m_quadratureChannel,
      m_minReading,
      m_maxReading,
      m_minPos,
      m_maxPos);

    int absoluteRange = abs(m_maxReading - m_minReading);
    m_quadratureMultiplier = double(m_quadratureRange) / double(absoluteRange);
  }

  return true;
}

//
// Feedback
//

void encoder::feedback(const Adc::ConstPtr& msg)
{
  // Read absolute input
  m_reading = m_filter(msg->adc[m_absoluteChannel]);

  // Read quadrature input
  if (m_quadratureChannel != -1)
  {
    m_offset = msg->adc[m_quadratureChannel];
    m_fusedReading = m_filter.isStable()
      ? m_reading
      : m_fusedReading + int(double(m_offset) * m_quadratureMultiplier);
  }
  else
  {
    m_fusedReading = m_reading;
  }

  // Map to joint space
  double position = utilities::map(
    (double)m_fusedReading,
    (double)m_minReading,
    (double)m_maxReading,
    m_minPos,
    m_maxPos);

  // Clamp to valid range
  m_position = utilities::clamp(position, m_minPos, m_maxPos);

  // Check if the position is ready to be used
  if (!m_ready)
  {
    m_ready = m_filter.isStable();
  }
}

//
// Dynamic creation
//

controller* encoder::create(ros::NodeHandle node, string path)
{
  return new encoder(node, path);
}
