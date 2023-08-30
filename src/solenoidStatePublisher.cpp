/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 solenoidPublisher.cpp

 Solenoid State Publisher Class Implementation
 Created 8/29/2023

 Copyright (C) 2023 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <sensor_msgs/JointState.h>
#include "solenoidStatePublisher.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace str1ker;
using namespace std;

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const char solenoidStatePublisher::PREFIX[] = "solenoid_state_publisher";

/*----------------------------------------------------------*\
| solenoidPublisher implementation
\*----------------------------------------------------------*/

solenoidStatePublisher::solenoidStatePublisher(ros::NodeHandle node):
    m_node(node),
    m_rate(50.0)
{
}

bool solenoidStatePublisher::configure()
{
    // Get spin rate
    m_node.getParam(string(PREFIX) + "/rate", m_rate);

    // Get subscribe topic
    if (!m_node.getParam(string(PREFIX) + "/subscribe", m_subscribeTopic))
    {
        ROS_ERROR_NAMED(PREFIX, "Subscribe topic not specified");
        return false;
    }

    // Get publish topic
    if (!m_node.getParam(string(PREFIX) + "/publish", m_publishTopic))
    {
        ROS_ERROR_NAMED(PREFIX, "Publish topic not specified");
        return false;
    }

    // Get joint mapping
    map<string, int> jointToChannel;
    m_node.getParam(string(PREFIX) + "/joints", jointToChannel);

    for (auto mapping: jointToChannel)
    {
        solenoidState state;

        state.channel = mapping.second;
        state.joint = mapping.first;
        state.trigger = false;
        state.reset = ros::Time::now();

        if (!m_node.getParam(string(PREFIX) + "/" + state.joint + "/low", state.low))
        {
            ROS_WARN_NAMED(PREFIX, "Low state not specified for %s", state.joint.c_str());
        }
        
        if (!m_node.getParam(string(PREFIX) + "/" + state.joint + "/high", state.high))
        {
            ROS_WARN_NAMED(PREFIX, "High state not specified for %s", state.joint.c_str());
        }

        m_states.push_back(state);
    }

    return true;
}

bool solenoidStatePublisher::init()
{
    // Initialize joint state publisher
    m_pub = m_node.advertise<sensor_msgs::JointState>(
        m_publishTopic,
        PUBLISH_QUEUE_SIZE
    );

    // Initialize analog subscriber
    m_sub = m_node.subscribe<Pwm>(
        m_subscribeTopic,
        SUBSCRIBE_QUEUE_SIZE,
        &str1ker::solenoidStatePublisher::subscribeCallback,
        this
    );
}

void solenoidStatePublisher::update()
{
    // Publish joint states mapped from solenoid states
    ros::Time time = ros::Time::now();

    sensor_msgs::JointState jointState;
    jointState.name.resize(m_states.size());
    jointState.position.resize(m_states.size());

    for (size_t index = 0; index < m_states.size(); index++)
    {
        solenoidState& state = m_states[index];

        if (time > state.reset && state.trigger)
        {
            state.trigger = false;
        }

        jointState.name[index] = state.joint;
        jointState.position[index] = state.trigger
            ? state.high
            : state.low;
    }

    m_pub.publish(jointState);
}

void solenoidStatePublisher::subscribeCallback(const Pwm::ConstPtr& msg)
{
    auto now = ros::Time::now().toNSec();

    for (auto state: m_states)
    {
        if (msg->channels.size() > state.channel)
        {
            // Extract solenoid command
            auto command = msg->channels[state.channel];
            
            // Update solenoid state
            if (command.mode == command.MODE_DIGITAL)
            {
                state.reset.fromNSec(now + command.duration * 1000);
                state.trigger = command.value != 0;
            }
        }
    }
}

/*----------------------------------------------------------*\
| Node entry point implementation
\*----------------------------------------------------------*/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "solenoid_state_publisher");

    ros::NodeHandle node;
    solenoidStatePublisher solenoidPublisher(node);

    ros::Rate rate(6);
    
    if (!solenoidPublisher.configure() || !solenoidPublisher.init())
    {
        return 1;
    }

    while(node.ok())
    {
        solenoidPublisher.update();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
