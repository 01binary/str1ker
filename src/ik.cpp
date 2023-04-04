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
| IKPlugin implementation
\*----------------------------------------------------------*/

IKPlugin::IKPlugin(): m_node("~")
{
}

bool IKPlugin::initialize(
    const string &robot_description,
    const string &group_name,
    const string &base_name,
    const string &tip_frame,
    double search_discretization)
{
    vector<string> tip_frames;
    tip_frames.push_back(tip_frame);

    return initialize(
        robot_description,
        group_name,
        base_name,
        tip_frames,
        search_discretization);
}

bool IKPlugin::initialize(
    const string &robot_description,
    const string &group_name,
    const string &base_name,
    const vector<string>& tip_frames,
    double search_discretization)
{
    // Load configuration
    KinematicsBase::setValues(
        robot_description,
        group_name,
        base_name,
        tip_frames,
        search_discretization
    );

    // Load model
    rdf_loader::RDFLoader rdfLoader(robot_description);
    const std::shared_ptr<srdf::Model> &srdf = rdfLoader.getSRDF();
    const std::shared_ptr<urdf::ModelInterface>& urdfModel = rdfLoader.getURDF();

    if (!urdfModel || !srdf)
    {
        ROS_ERROR_NAMED("str1ker::ik","failed to load URDF and SDF");
        return false;
    }

    m_pModel.reset(new robot_model::RobotModel(urdfModel, srdf));

    // Retrieve joints
    m_pModelGroup = m_pModel->getJointModelGroup(group_name);

    if (!m_pModelGroup)
    {
        ROS_ERROR_NAMED("str1ker::ik", "failed to retrieve joint model group");
        return false;
    }

    ROS_INFO_NAMED(
        "str1ker::ik",
        "found %ld active joints and %ld mimic joints",
        m_pModelGroup->getActiveJointModels().size(),
        m_pModelGroup->getMimicJointModels().size()
    );

    for (vector<string>::const_iterator pos = tip_frames.begin();
         pos != tip_frames.end();
         pos++)
    {
        ROS_INFO_NAMED("str1ker::ik", "found tip %s", pos->c_str());
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
            "str1ker::ik"
            "joint limit %s min %g max %g vel %g",
            jointNames[limitIndex].c_str(),
            m_groupInfo.limits[limitIndex].min_position,
            m_groupInfo.limits[limitIndex].max_position,
            m_groupInfo.limits[limitIndex].max_velocity);
    }

    // Initialize joint states
    m_pState.reset(new robot_state::RobotState(m_pModel));
    m_pState->setToDefaultValues();

    // Initialize RViz
    m_pVisualTools.reset(new moveit_visual_tools::MoveItVisualTools(
        "/odom","/hrp2_visual_markers", m_pModel));
    m_pVisualTools->loadRobotStatePub("/moveit_whole_body_ik");

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
    if (ik_seed_state.size() != m_pModelGroup->getActiveJointModels().size())
    {
        ROS_ERROR_NAMED(
            "str1ker::ik",
            "Expected initial state for %ld active joints, received state for %ld",
            m_pModelGroup->getActiveJointModels().size(),
            ik_seed_state.size());

        error_code.val = error_code.NO_IK_SOLUTION;
        return false;
    }

    // Validate consistency limits
    if (!consistency_limits.empty() &&
        consistency_limits.size() != m_pModelGroup->getActiveJointModels().size())
    {
        ROS_ERROR_NAMED(
            "str1ker::ik",
            "Consistency limits must be empty or have limit for each of %ld active joints",
            m_pModelGroup->getActiveJointModels().size());

        error_code.val = error_code.NO_IK_SOLUTION;
        return false;
    }

    // Validate poses
    if (tip_frames_.size() != ik_poses.size())
    {
        error_code.val = error_code.NO_IK_SOLUTION;
        ROS_ERROR_NAMED(
            "str1ker::ik",
            "IK validation failed: found %ld tips and %ld poses (expected %ld poses)",
            tip_frames_.size(),
            ik_poses.size(),
            tip_frames_.size());

        return false;
    }

    for (vector<geometry_msgs::Pose>::const_iterator pos = ik_poses.begin();
        pos != ik_poses.end();
        pos++)
    {
        ROS_INFO_NAMED(
            "str1ker::ik",
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
    return false;
}

PLUGINLIB_EXPORT_CLASS(
    str1ker::IKPlugin,
    kinematics::KinematicsBase
);
