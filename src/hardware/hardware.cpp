/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 hardware.h
 Robot hardware drivers implementation
 Created 12/22/2023
 Copyright (C) 2023 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <xmlrpcpp/XmlRpcValue.h>
#include "hardware.h"

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const double str1ker::hardware::DEFAULT_RATE = 50.0;
const char* const DEFAULT_POSITION_TOPIC = "arm/position";
const char* const DEFAULT_STATE_TOPIC = "arm/state";
const char* const DEFAULT_CONTROLLER_NAME = "arm_position_controller";
const char* const JOINTS_SUFFIX = "/joints";
const char* const TYPE_SUFFIX = "/type";
const char* const JTC_TYPE_TOKEN = "JointTrajectoryController";
const char* const REQUIRED_ARM_JOINTS[] = {"base", "shoulder", "elbow"};
const size_t REQUIRED_ARM_JOINT_COUNT = 3;

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace str1ker;
using namespace std;

/*----------------------------------------------------------*\
| hardware implementation
\*----------------------------------------------------------*/

hardware::hardware(ros::NodeHandle node, string configNamespace)
    : m_namespace(configNamespace)
    , m_node(node)
    , m_rate(DEFAULT_RATE)
    , m_controllerManager(this, node)
    , m_positionTopic(DEFAULT_POSITION_TOPIC)
    , m_stateTopic(DEFAULT_STATE_TOPIC)
    , m_controllerName(DEFAULT_CONTROLLER_NAME)
    , m_lastUpdate(0)
    , m_lastFeedback(0)
    , m_debug(false)
{
}

bool hardware::configure()
{
    m_node.param(m_namespace + "/publish_rate", m_rate, DEFAULT_RATE);
    m_node.param(m_namespace + "/debug", m_debug, false);
    m_node.param(m_namespace + "/position_topic", m_positionTopic, string(DEFAULT_POSITION_TOPIC));
    m_node.param(m_namespace + "/state_topic", m_stateTopic, string(DEFAULT_STATE_TOPIC));
    m_node.param(m_namespace + "/controller_name", m_controllerName, string(DEFAULT_CONTROLLER_NAME));

    if (!loadJointNamesFromParams(m_jointNames))
    {
        ROS_ERROR("failed to discover any joints from ROS control parameters");
        return false;
    }

    if (m_jointNames.empty())
    {
        ROS_ERROR("no hardware joints discovered for hardware interface");
        return false;
    }

    if (!loadJointLimitsFromUrdf())
    {
        ROS_ERROR("failed to load joint limits from robot_description");
        return false;
    }

    m_positionPublisher = m_node.advertise<str1ker::PositionCommand>(m_positionTopic, 10);
    m_stateSubscriber = m_node.subscribe(m_stateTopic, 10, &hardware::stateCallback, this);

    string joints;
    for (size_t i = 0; i < m_jointNames.size(); ++i)
    {
        joints += m_jointNames[i];
        if (i + 1 < m_jointNames.size())
        {
            joints += ", ";
        }
    }

    ROS_INFO("hardware configured: controller='%s', joints=[%s], state='%s', position='%s', rate=%.2f Hz",
        m_controllerName.c_str(),
        joints.c_str(),
        m_stateTopic.c_str(),
        m_positionTopic.c_str(),
        m_rate);

    return true;
}

bool hardware::loadJointNamesFromParams(vector<string>& jointNames) const
{
    if (m_controllerName.empty())
    {
        ROS_ERROR("controller name for joint discovery is empty");
        return false;
    }

    string controllerNamespace = m_controllerName;
    if (controllerNamespace.front() != '/')
    {
        controllerNamespace = "/" + controllerNamespace;
    }

    const string jointsParam = controllerNamespace + JOINTS_SUFFIX;
    XmlRpc::XmlRpcValue jointsValue;
    if (!ros::param::get(jointsParam, jointsValue) ||
        jointsValue.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        jointsValue.size() <= 0)
    {
        ROS_ERROR("could not read joint list from '%s'", jointsParam.c_str());
        return false;
    }

    const string controllerTypeParam = controllerNamespace + TYPE_SUFFIX;
    string controllerType;
    if (ros::param::get(controllerTypeParam, controllerType) &&
        controllerType.find(JTC_TYPE_TOKEN) == string::npos)
    {
        ROS_WARN("controller '%s' has type '%s' (expected JointTrajectoryController-style type)",
            controllerNamespace.c_str(),
            controllerType.c_str());
    }

    vector<string> discoveredJoints;
    for (int i = 0; i < jointsValue.size(); ++i)
    {
        if (jointsValue[i].getType() != XmlRpc::XmlRpcValue::TypeString)
        {
            ROS_ERROR("joint entry %d in '%s' is not a string", i, jointsParam.c_str());
            return false;
        }

        discoveredJoints.push_back(static_cast<string>(jointsValue[i]));
    }

    vector<string> requiredJoints(
        REQUIRED_ARM_JOINTS,
        REQUIRED_ARM_JOINTS + REQUIRED_ARM_JOINT_COUNT);

    for (const string& requiredJoint : requiredJoints)
    {
        bool found = false;
        for (const string& candidateJoint : discoveredJoints)
        {
            if (candidateJoint == requiredJoint)
            {
                found = true;
                break;
            }
        }

        if (!found)
        {
            ROS_ERROR(
                "controller configuration is missing required joint '%s'",
                requiredJoint.c_str());
            return false;
        }
    }

    jointNames = discoveredJoints;
    return true;
}

bool hardware::loadJointLimitsFromUrdf()
{
    string description;
    if (!ros::param::get("robot_description", description))
    {
        ROS_ERROR("could not read 'robot_description'");
        return false;
    }

    urdf::Model model;
    if (!model.initString(description))
    {
        ROS_ERROR("failed to parse robot_description URDF");
        return false;
    }

    bool allLoaded = true;

    for (const string& jointName : m_jointNames)
    {
        const urdf::JointConstSharedPtr joint = model.getJoint(jointName);
        if (!joint)
        {
            ROS_ERROR("joint '%s' was discovered but is missing from URDF", jointName.c_str());
            allLoaded = false;
            continue;
        }

        joint_limits_interface::JointLimits limits;
        if (!joint_limits_interface::getJointLimits(joint, limits))
        {
            ROS_ERROR("joint '%s' has no limits in URDF", jointName.c_str());
            allLoaded = false;
            continue;
        }

        m_limits[jointName] = limits;
    }

    return allLoaded;
}

bool hardware::init()
{
    if (m_jointNames.empty())
    {
        ROS_ERROR("cannot initialize hardware without joints");
        return false;
    }

    for (const string& jointName : m_jointNames)
    {
        m_pos[jointName] = 0.0;
        m_vel[jointName] = 0.0;
        m_effort[jointName] = 0.0;
        m_cmd[jointName] = 0.0;
        m_feedbackPos[jointName] = 0.0;
        m_feedbackVel[jointName] = 0.0;

        hardware_interface::JointStateHandle stateHandle(
            jointName,
            &m_pos[jointName],
            &m_vel[jointName],
            &m_effort[jointName]);
        m_stateInterface.registerHandle(stateHandle);

        hardware_interface::JointHandle commandHandle(stateHandle, &m_cmd[jointName]);
        m_posInterface.registerHandle(commandHandle);

        const auto limits = m_limits.find(jointName);
        if (limits != m_limits.end())
        {
            joint_limits_interface::PositionJointSaturationHandle saturationHandle(
                commandHandle,
                limits->second);
            m_satInterface.registerHandle(saturationHandle);
        }
        else
        {
            ROS_WARN("joint '%s' has no limits interface registered", jointName.c_str());
        }
    }

    registerInterface(&m_stateInterface);
    registerInterface(&m_posInterface);
    registerInterface(&m_satInterface);

    m_lastUpdate = ros::Time::now();

    return true;
}

void hardware::update()
{
    const ros::Time now = ros::Time::now();

    ros::Duration period;
    if (m_lastUpdate.isZero())
    {
        period = ros::Duration(1.0 / m_rate);
    }
    else
    {
        period = now - m_lastUpdate;
        if (period.toSec() <= 0.0)
        {
            period = ros::Duration(1.0 / m_rate);
        }
    }

    read();

    m_controllerManager.update(now, period);
    m_satInterface.enforceLimits(period);

    if (m_debug)
    {
        debug();
    }

    write();

    m_lastUpdate = now;
}

bool hardware::readJoint(
    const str1ker::StateFeedback& msg,
    const string& jointName,
    double& position,
    double& velocity) const
{
    if (jointName == "base")
    {
        position = msg.basePosition;
        velocity = msg.baseVelocity;
        return true;
    }

    if (jointName == "shoulder")
    {
        position = msg.shoulderPosition;
        velocity = msg.shoulderVelocity;
        return true;
    }

    if (jointName == "elbow")
    {
        position = msg.elbowPosition;
        velocity = msg.elbowVelocity;
        return true;
    }

    return false;
}

bool hardware::writeJoint(
    str1ker::PositionCommand& msg,
    const string& jointName,
    double command) const
{
    if (jointName == "base")
    {
        msg.base = command;
        return true;
    }

    if (jointName == "shoulder")
    {
        msg.shoulder = command;
        return true;
    }

    if (jointName == "elbow")
    {
        msg.elbow = command;
        return true;
    }

    return false;
}

void hardware::stateCallback(const str1ker::StateFeedbackConstPtr& msg)
{
    lock_guard<mutex> lock(m_stateMutex);

    for (const string& jointName : m_jointNames)
    {
        double position = 0.0;
        double velocity = 0.0;

        if (readJoint(*msg, jointName, position, velocity))
        {
            m_feedbackPos[jointName] = position;
            m_feedbackVel[jointName] = velocity;
        }
    }

    m_lastFeedback = ros::Time::now();
}

void hardware::read()
{
    ros::Time lastFeedback;
    {
        lock_guard<mutex> lock(m_stateMutex);
        for (const string& jointName : m_jointNames)
        {
            m_pos[jointName] = m_feedbackPos[jointName];
            m_vel[jointName] = m_feedbackVel[jointName];
        }

        lastFeedback = m_lastFeedback;
    }
}

void hardware::write()
{
    str1ker::PositionCommand msg;

    for (const string& jointName : m_jointNames)
    {
        const auto it = m_cmd.find(jointName);
        if (it == m_cmd.end())
        {
            continue;
        }

        if (!writeJoint(msg, jointName, it->second))
        {
            ROS_WARN_THROTTLE(5.0, "cannot map joint '%s' to PositionCommand", jointName.c_str());
        }
    }

    m_positionPublisher.publish(msg);
}

void hardware::debug()
{
    for (const string& jointName : m_jointNames)
    {
        ROS_INFO_NAMED(
            "hardware",
            "%s: pos %g vel %g eff %g cmd %g",
            jointName.c_str(),
            m_pos[jointName],
            m_vel[jointName],
            m_effort[jointName],
            m_cmd[jointName]);
    }
}

void hardware::run()
{
    ros::Rate rate(m_rate);
    ros::AsyncSpinner spinner(2);

    // Controller manager can block with single-threaded spin.
    spinner.start();

    while (m_node.ok())
    {
        update();
        rate.sleep();
    }
}

/*----------------------------------------------------------*\
| Node entry point implementation
\*----------------------------------------------------------*/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hardware");

    ros::NodeHandle node;
    hardware hw(node, "robot");

    ROS_INFO("loading hardware");

    if (!hw.configure() || !hw.init())
    {
        ROS_FATAL("hardware failed to initialize");
        return 1;
    }

    ROS_INFO("hardware initialized successfully");

    hw.run();

    return 0;
}
