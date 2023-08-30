/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 solenoidStatePublisher.h

 Solenoid State Publisher Class Implementation
 Created 8/29/2023

 Copyright (C) 2023 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <string>
#include <map>
#include <vector>
#include <ros/ros.h>
#include "robot.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| Types
\*----------------------------------------------------------*/

struct solenoidState
{
    // Solenoid analog command channel
    int channel;

    // Corresponds to this joint name
    std::string joint;

    // Solenoid is engaged
    bool trigger;

    // Reset trigger state at this time
    ros::Time reset;

    // Joint state to publish when solenoid is off
    double low;

    // Joint state to publish when solenoid is on
    double high;
};

/*----------------------------------------------------------*\
| solenoidStatePublisher class
\*----------------------------------------------------------*/

class solenoidStatePublisher
{
private:
    static const char PREFIX[];

    // Publisher queue size
    const int PUBLISH_QUEUE_SIZE = 4;

    // Subscriber queue size
    const int SUBSCRIBE_QUEUE_SIZE = 4;

    // Spin rate
    double m_rate;

    // Topic to listen to for solenoid messages
    std::string m_subscribeTopic;

    // Topic to publish joint states to
    std::string m_publishTopic;

    // Solenoid states to translate into joint states
    std::vector<solenoidState> m_states;

    // Current node
    ros::NodeHandle m_node;

    // Subscriber to PWM topic
    ros::Subscriber m_sub;

    // Publisher to joint_states topic
    ros::Publisher m_pub;

public:
    solenoidStatePublisher(ros::NodeHandle node);

public:
    bool configure();
    bool init();
    void update();

private:
    // Subscribe callback
    void subscribeCallback(const Pwm::ConstPtr& msg);
};

} // namespace str1ker

/*----------------------------------------------------------*\
| Node entry point
\*----------------------------------------------------------*/

int main(int argc, char** argv);
