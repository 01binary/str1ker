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

    // Examine joints
    m_pModelGroup = robot_model.getJointModelGroup(group_name);

    if (!m_pModelGroup)
    {
        ROS_ERROR_NAMED(PLUGIN_NAME, "Failed to retrieve joint model group");
        return false;
    }

    for (vector<string>::const_iterator pos = tip_frames.begin();
         pos != tip_frames.end();
         pos++)
    {
        ROS_INFO_NAMED(PLUGIN_NAME, "Tip Frame %s", pos->c_str());
    }

    const vector<const moveit::core::JointModel*>& joints =
        m_pModelGroup->getJointModels();
    const vector<string>& jointNames =
        m_pModelGroup->getJointModelNames();

    for (size_t jointIndex = 0;
        jointIndex < joints.size();
        jointIndex++)
    {
        const moveit::core::JointModel* pJoint = joints[jointIndex];
        bool isRevolute = pJoint->getType() == moveit::core::JointModel::REVOLUTE;
        bool isPrismatic = pJoint->getType() == moveit::core::JointModel::PRISMATIC;
        bool isPassive = pJoint->isPassive();
        bool isMimic = pJoint->getMimic() != NULL;

        if (isRevolute || isPrismatic)
        {
            const moveit_msgs::JointLimits& limitsFirstDof =
                pJoint->getVariableBoundsMsg()[0];

            ROS_INFO_NAMED(
                PLUGIN_NAME,
                "Joint %s %s %s %smin %g max %g vel %g",
                jointNames[jointIndex].c_str(),
                isRevolute ? "revolute" : "prismatic",
                isPassive ? "passive" : "active",
                isMimic ? "mimic " : "",
                limitsFirstDof.min_position,
                limitsFirstDof.max_position,
                limitsFirstDof.max_velocity);
        }
    }

    const vector<const moveit::core::LinkModel*>& links =
        m_pModelGroup->getLinkModels();
    const vector<string>& linkNames =
        m_pModelGroup->getLinkModelNames();

    for (size_t linkIndex = 0;
         linkIndex < links.size();
         linkIndex++)
    {
        const double* origin =
            links[linkIndex]->getJointOriginTransform().data()

        ROS_INFO_NAMED(
            PLUGIN_NAME,
            "Link %s %s origin [ %g %g %g | %g %g %g | %g %g %g ]",
            linkNames[linkIndex].c_str(),
            links[linkIndex]->getVisualMeshFilename().c_str(),
            origin[0],
            origin[1],
            origin[2],
            origin[3],
            origin[4],
            origin[5],
            origin[6],
            origin[7],
            origin[8],
        );
    }

    // Initialize joint states
    m_pState.reset(new robot_state::RobotState(robot_model_));
    m_pState->setToDefaultValues();

    return true;
}

bool IKPlugin::supportsGroup(
    const moveit::core::JointModelGroup *jmg, std::string *error_text_out) const
{
    return true;
}

const vector<string>& IKPlugin::getJointNames() const
{
    return m_pModelGroup->getJointModelNames();
}

const vector<string>& IKPlugin::getLinkNames() const
{
    return m_pModelGroup->getLinkModelNames();
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

    // Validate poses
    if (tip_frames_.size() != ik_poses.size())
    {
        ROS_ERROR_NAMED(
            PLUGIN_NAME,
            "Found %ld tips and %ld poses (expected %ld poses, one for each tip)",
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
    solution.resize(ik_seed_state.size());

    for (size_t jointIndex = 0; jointIndex < ik_seed_state.size(); jointIndex++)
    {
        solution[jointIndex] = ik_seed_state[jointIndex];

        if ((ros::WallTime::now() - startTime).toSec() < timeout) break;
    }

    return true;
}

IKPluginRegistrar::IKPluginRegistrar()
{
    // PLUGINLIB_EXPORT_CLASS macro does not compile on Noetic
    class_loader::impl::registerPlugin<str1ker::IKPlugin, kinematics::KinematicsBase>(
        "str1ker::IKPlugin", "kinematics::KinematicsBase");
}
