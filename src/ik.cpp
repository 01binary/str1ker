/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 ik.cpp

 Inverse Kinematics Plugin
 Created 04/03/2023

 Copyright (C) 2023 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include "include/ik.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace std;
using namespace str1ker;

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

const char PLUGIN_NAME[] = "str1ker::ik";
IKPluginRegistrar g_registerIkPlugin;

/*----------------------------------------------------------*\
| IKPlugin implementation
\*----------------------------------------------------------*/

IKPlugin::IKPlugin(): m_node("~")
{
}

bool IKPlugin::initialize(
    const moveit::core::RobotModel& robot_model,
    const std::string& group_name,
    const std::string& base_frame,
    const std::vector<std::string>& tip_frames,
    double search_discretization)
{
    // Load configuration
    KinematicsBase::storeValues(
        robot_model,
        group_name,
        base_frame,
        tip_frames,
        search_discretization
    );

    // Retrieve joints
    m_pModelGroup = robot_model.getJointModelGroup(group_name);

    if (!m_pModelGroup)
    {
        ROS_ERROR_NAMED(PLUGIN_NAME, "failed to retrieve joint model group");
        return false;
    }

    ROS_INFO_NAMED(
        PLUGIN_NAME,
        "found %ld active joints and %ld mimic joints",
        m_pModelGroup->getActiveJointModels().size(),
        m_pModelGroup->getMimicJointModels().size()
    );

    for (vector<string>::const_iterator pos = tip_frames.begin();
         pos != tip_frames.end();
         pos++)
    {
        ROS_INFO_NAMED(PLUGIN_NAME, "found tip %s", pos->c_str());
    }

    const vector<const moveit::core::JointModel*>& jointModels =
        m_pModelGroup->getJointModels();
    const vector<string>& jointNames =
        m_pModelGroup->getJointModelNames();

    for (size_t jointIndex = 0;
        jointIndex < jointModels.size();
        jointIndex++)
    {
        const moveit::core::JointModel* pJointModel = jointModels[jointIndex];

        if (pJointModel->getType() == moveit::core::JointModel::REVOLUTE ||
            pJointModel->getType() == moveit::core::JointModel::PRISMATIC)
        {
            m_groupInfo.joint_names.push_back(jointNames[jointIndex]);

            const vector<moveit_msgs::JointLimits>& jointLimits = pJointModel->getVariableBoundsMsg();
            m_groupInfo.limits.insert(m_groupInfo.limits.end(), jointLimits.begin(), jointLimits.end());
        }
    }

    for (size_t limitIndex = 0;
         limitIndex < m_groupInfo.limits.size();
         limitIndex++)
    {
        ROS_INFO_NAMED(
            PLUGIN_NAME,
            "joint limit %s min %g max %g vel %g",
            jointNames[limitIndex].c_str(),
            m_groupInfo.limits[limitIndex].min_position,
            m_groupInfo.limits[limitIndex].max_position,
            m_groupInfo.limits[limitIndex].max_velocity);
    }

    // Initialize joint states
    m_pState.reset(new robot_state::RobotState(robot_model_));
    m_pState->setToDefaultValues();

    // Initialize RViz
    // m_pVisualTools.reset(new moveit_visual_tools::MoveItVisualTools());

    return true;
}

bool IKPlugin::supportsGroup(
    const moveit::core::JointModelGroup *jmg, std::string *error_text_out) const
{
    return true;
}

const vector<string>& IKPlugin::getJointNames() const
{
    return m_groupInfo.joint_names;
}

const vector<string>& IKPlugin::getLinkNames() const
{
    return m_groupInfo.link_names;
}

bool IKPlugin::getPositionFK(
    const vector<string> &link_names,
    const vector<double> &joint_angles,
    vector<geometry_msgs::Pose> &poses) const
{
    // TODO: first task after getting this compiled
    return false;
}

bool IKPlugin::getPositionIK(
    const geometry_msgs::Pose &ik_pose,
    const vector<double> &ik_seed_state,
    vector<double> &solution,
    moveit_msgs::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const
{
    return searchPositionIK(
        ik_pose,
        ik_seed_state,
        DEFAULT_TIMEOUT,
        solution,
        error_code,
        options);
}

bool IKPlugin::searchPositionIK(
    const geometry_msgs::Pose &ik_pose,
    const vector<double> &ik_seed_state,
    double timeout,
    vector<double> &solution,
    moveit_msgs::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const
{
    const IKCallbackFn solution_callback = NULL;
    vector<double> consistency_limits;
    vector<geometry_msgs::Pose> poses;
    poses.push_back(ik_pose);

    return searchPositionIK(
        poses,
        ik_seed_state,
        timeout,
        consistency_limits,
        solution,
        solution_callback,
        error_code,
        options);
}

bool IKPlugin::searchPositionIK(
    const geometry_msgs::Pose &ik_pose,
    const vector<double> &ik_seed_state,
    double timeout,
    const vector<double> &consistency_limits,
    vector<double> &solution,
    moveit_msgs::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const
{
    const IKCallbackFn solution_callback = NULL;

    return searchPositionIK(
        ik_pose,
        ik_seed_state,
        timeout,
        consistency_limits,
        solution,
        solution_callback,
        error_code,
        options);
}

bool IKPlugin::searchPositionIK(
    const geometry_msgs::Pose &ik_pose,
    const vector<double> &ik_seed_state,
    double timeout,
    vector<double> &solution,
    const IKCallbackFn &solution_callback,
    moveit_msgs::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const
{
    vector<double> consistency_limits;

    return searchPositionIK(
        ik_pose,
        ik_seed_state,
        timeout,
        consistency_limits,
        solution,
        solution_callback,
        error_code,
        options);
}

bool IKPlugin::searchPositionIK(
    const geometry_msgs::Pose &ik_pose,
    const vector<double> &ik_seed_state,
    double timeout,
    const vector<double> &consistency_limits,
    vector<double> &solution,
    const IKCallbackFn &solution_callback,
    moveit_msgs::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const
{
    vector<geometry_msgs::Pose> poses;
    poses.push_back(ik_pose);

    return searchPositionIK(
        poses,
        ik_seed_state,
        timeout,
        consistency_limits,
        solution,
        solution_callback,
        error_code,
        options);
}

bool IKPlugin::searchPositionIK(
    const vector<geometry_msgs::Pose> &ik_poses,
    const vector<double> &ik_seed_state,
    double timeout,
    const vector<double> &consistency_limits,
    vector<double> &solution,
    const IKCallbackFn &solution_callback,
    moveit_msgs::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options,
    const moveit::core::RobotState* context_state) const
{
    // Validate initial state
    if (ik_seed_state.size() >
        m_pModelGroup->getActiveJointModels().size() +
        m_pModelGroup->getMimicJointModels().size())
    {
        ROS_ERROR_NAMED(
            PLUGIN_NAME,
            "Expected state for %ld active joints (and possibly %ld mimic joints), received state for %ld",
            m_pModelGroup->getActiveJointModels().size(),
            m_pModelGroup->getMimicJointModels().size(),
            ik_seed_state.size());

        error_code.val = error_code.NO_IK_SOLUTION;
        return false;
    }

    // Validate consistency limits
    if (!consistency_limits.empty() &&
        consistency_limits.size() != m_pModelGroup->getActiveJointModels().size())
    {
        ROS_ERROR_NAMED(
            PLUGIN_NAME,
            "Consistency limits must be empty or have limit for each of %ld active joints",
            m_pModelGroup->getActiveJointModels().size());

        error_code.val = error_code.NO_IK_SOLUTION;
        return false;
    }

    // Validate poses
    if (tip_frames_.size() != ik_poses.size())
    {
        ROS_ERROR_NAMED(
            PLUGIN_NAME,
            "IK validation failed: found %ld tips and %ld poses (expected %ld poses)",
            tip_frames_.size(),
            ik_poses.size(),
            tip_frames_.size());

        error_code.val = error_code.NO_IK_SOLUTION;
        return false;
    }

    for (vector<geometry_msgs::Pose>::const_iterator pos = ik_poses.begin();
        pos != ik_poses.end();
        pos++)
    {
        ROS_INFO_NAMED(
            PLUGIN_NAME,
            "IK target %g %g %g",
            pos->position.x,
            pos->position.y,
            pos->position.z);
    }

    ros::WallTime startTime = ros::WallTime::now();
    solution.resize(m_pModelGroup->getActiveJointModels().size());

    /*while ((ros::WallTime::now() - startTime).toSec() < timeout)
    {
    }*/

    error_code.val = error_code.NO_IK_SOLUTION;
    return true;
}

IKPluginRegistrar::IKPluginRegistrar()
{
    // PLUGINLIB_EXPORT_CLASS macro does not compile on Noetic
    class_loader::impl::registerPlugin<str1ker::IKPlugin, kinematics::KinematicsBase>(
        "str1ker::IKPlugin", "kinematics::KinematicsBase");
}
