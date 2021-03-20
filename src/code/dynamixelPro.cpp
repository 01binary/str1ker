/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 dynamixelPro.cpp

 Dynamixel Pro Servo Controller Implementation
 Created 1/19/2021

 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <math.h>
#include <pigpio.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include "dynamixelPro.h"
#include "controllerFactory.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace std;
using namespace str1ker;

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const char dynamixelPro::TYPE[] = "dynamixelPro";

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

DynamixelWorkbench* dynamixelPro::s_wb = NULL;

/*----------------------------------------------------------*\
| dynamixelPro implementation
\*----------------------------------------------------------*/

REGISTER_CONTROLLER(dynamixelPro)

dynamixelPro::dynamixelPro(const char* path) :
    servo(path),
    m_id(0),
    m_pos(NULL),
    m_goal(NULL)
{
}

dynamixelPro::dynamixelPro(const char* path, int id) :
    servo(path),
    m_id(id),
    m_pos(NULL),
    m_goal(NULL)
{
}

const char* dynamixelPro::getType()
{
    return dynamixelPro::TYPE;
}

bool dynamixelPro::init()
{
    const char *log;

    if (!m_enable) return true;

    if (!s_wb)
    {
        string serial;
        int baud;

        ros::param::param<string>("robot/serial", serial, "/dev/ttyUSB0");
        ros::param::param<int>("robot/baud", baud, 57600);

        s_wb = new DynamixelWorkbench();

        if (!s_wb->init(serial.c_str(), baud, &log))
        {
            ROS_ERROR("  failed to initialize %s serial servo: %s", m_path.c_str(), log);
            return false;
        }
    }

    if (!s_wb->ping(m_id, &log) ||
        !s_wb->torqueOn(m_id, &log))
    {
        ROS_ERROR("  failed to initialize %s serial servo: %s", m_path.c_str(), log);
        return false;
    }

    m_pos = s_wb->getItemInfo(m_id, "Present_Position");
    m_goal = s_wb->getItemInfo(m_id, "Goal_Position");

    if (!m_pos ||
        !m_goal ||
        !s_wb->addSyncWriteHandler(m_goal->address, m_goal->data_length) ||
        !s_wb->addSyncReadHandler(m_pos->address, m_pos->data_length))
    {
        ROS_ERROR("  failed to add handlers for %s serial servo", m_path.c_str());
        return false;
    }

    ROS_INFO("  initialized %s %s with id %d", getPath(), getType(), m_id);
    return true;
}

double dynamixelPro::getPos()
{
    uint8_t id = m_id;
    int32_t value = 0;

    if (!s_wb->syncRead(0, &id, 1) ||
        !s_wb->getSyncReadData(0, &id, 1, m_pos->address, m_pos->data_length, &value))
    {
        ROS_ERROR("failed to read %s dynamixelPro servo position", m_name.c_str());
        return 0.0;
    }

    return s_wb->convertValue2Radian(id, value);
}

void dynamixelPro::setPos(double pos)
{
    if (!m_enable) return;

    uint8_t id = m_id;
    int32_t value = s_wb->convertRadian2Value(m_id, pos);

    if (!s_wb->syncWrite(0, &id, 1, &value, 1))
    {
        ROS_ERROR("failed to write %s dynamixelPro servo position", m_name.c_str());
        return;
    }
}

void dynamixelPro::deltaPos(double delta)
{
    if (!m_enable) return;

    double posRad = getPos() + delta;
    uint8_t id = m_id;
    int32_t value = s_wb->convertRadian2Value(m_id, posRad);

    if (!s_wb->syncWrite(0, &id, 1, &value, 1))
    {
        ROS_ERROR("failed to write %s dynamixelPro servo position", m_name.c_str());
        return;
    }
}

void dynamixelPro::deserialize(ros::NodeHandle node)
{
    servo::deserialize(node);

    ros::param::get(getControllerPath("id"), m_id);

    m_pub = node.advertise<std_msgs::Float32>(getPath(), 256);
}

void dynamixelPro::publish()
{
    std_msgs::Float32 msg;
    msg.data = getPos();
    m_pub.publish(msg);
}

controller* dynamixelPro::create(const char* path)
{
    return new dynamixelPro(path);
}
