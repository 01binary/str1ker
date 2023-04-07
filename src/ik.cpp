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
using namespace kinematics;
using namespace moveit::core;
using namespace moveit_msgs;
using namespace str1ker;

/*----------------------------------------------------------*\
| Definitions
\*----------------------------------------------------------*/

typedef const JointModel* JointModelConstPtr;
typedef const LinkModel* LinkModelConstPtr;

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

const char PLUGIN_NAME[] = "str1ker::ik";
IKPluginRegistrar g_registerIkPlugin;

/*----------------------------------------------------------*\
| IKPlugin implementation
\*----------------------------------------------------------*/

IKPlugin::IKPlugin()
{
}

bool IKPlugin::initialize(
    const RobotModel& robot_model,
    const string& group_name,
    const string& base_frame,
    const vector<string>& tip_frames,
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

    // Validate links and joints
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

    vector<LinkModelConstPtr> tips;
    m_pModelGroup->getEndEffectorTips(tips);

    for (vector<LinkModelConstPtr>::const_iterator posTip = tips.begin();
        posTip != tips.end();
        posTip++)
    {
        ROS_INFO_NAMED(PLUGIN_NAME, "Tip %s", (*posTip)->getName().c_str());
    }

    const vector<pair<string, string>>& chains = m_pModelGroup->getConfig().chains_;

    for (auto posChain = chains.begin();
        posChain != chains.end();
        posChain++)
    {
        ROS_INFO_NAMED(PLUGIN_NAME, "Chain: %s -> %s", posChain->first, posChain->second);
    }

    const vector<JointModelConstPtr>& joints = m_pModelGroup->getJointModels();
    const vector<string>& jointNames = m_pModelGroup->getJointModelNames();

    for (size_t jointIndex = 0;
        jointIndex < joints.size();
        jointIndex++)
    {
        JointModelConstPtr pJoint = joints[jointIndex];
        const JointLimits& limits = pJoint->getVariableBoundsMsg()[0];
        bool isRevolute = pJoint->getType() == JointModel::REVOLUTE;
        bool isPrismatic = pJoint->getType() == JointModel::PRISMATIC;
        bool isPassive = pJoint->isPassive();
        bool isMimic = pJoint->getMimic() != NULL;

        if (isRevolute)
        {
            const Eigen::Vector3d& axis =
                dynamic_cast<const RevoluteJointModel*>(pJoint)->getAxis();

            ROS_INFO_NAMED(
                PLUGIN_NAME,
                "Joint %s %s %s %saxis %g %g %g, min %g max %g vel %g",
                jointNames[jointIndex].c_str(),
                "revolute",
                isPassive ? "passive" : "active",
                isMimic ? "mimic " : "",
                axis.x(),
                axis.y(),
                axis.z(),
                limits.min_position,
                limits.max_position,
                limits.max_velocity);
        }
        else if (isPrismatic)
        {
            const Eigen::Vector3d& axis =
                dynamic_cast<const PrismaticJointModel*>(pJoint)->getAxis();

            ROS_INFO_NAMED(
                PLUGIN_NAME,
                "Joint %s %s %s %saxis %g %g %g, min %g max %g vel %g",
                jointNames[jointIndex].c_str(),
                "prismatic",
                isPassive ? "passive" : "active",
                isMimic ? "mimic " : "",
                axis.x(),
                axis.y(),
                axis.z(),
                limits.min_position,
                limits.max_position,
                limits.max_velocity);
        }
    }

    const vector<LinkModelConstPtr>& links = m_pModelGroup->getLinkModels();
    const vector<string>& linkNames = m_pModelGroup->getLinkModelNames();

    for (size_t linkIndex = 0;
         linkIndex < links.size();
         linkIndex++)
    {
        ROS_INFO_NAMED(PLUGIN_NAME, "Link %s", linkNames[linkIndex].c_str());
    }

    // Initialize joint states
    m_pState.reset(new RobotState(robot_model));
    m_pState->setToDefaultValues();

    return true;
}

bool IKPlugin::supportsGroup(
    const JointModelGroup *jmg, string *error_text_out) const
{
    if (!jmg->isSingleDOFJoints())
    {
        *error_text_out = "IK solver supports only single DOF joints";
        return false;
    }

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
    /* useful for fk and ik
    virtual void 	computeTransform (const double *joint_values, Eigen::Affine3d &transf) const =0
 	Given the joint values for a joint, compute the corresponding transform.
    virtual void 	computeVariablePositions (const Eigen::Affine3d &transform, double *joint_values) const =0
 	Given the transform generated by joint, compute the corresponding joint values.
    */

    return false;
}

bool IKPlugin::getPositionIK(
    const geometry_msgs::Pose &ik_pose,
    const vector<double> &ik_seed_state,
    vector<double> &solution,
    MoveItErrorCodes &error_code,
    const KinematicsQueryOptions &options) const
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
    MoveItErrorCodes &error_code,
    const KinematicsQueryOptions &options) const
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
    MoveItErrorCodes &error_code,
    const KinematicsQueryOptions &options) const
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
    MoveItErrorCodes &error_code,
    const KinematicsQueryOptions &options) const
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
    MoveItErrorCodes &error_code,
    const KinematicsQueryOptions &options) const
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
    MoveItErrorCodes &error_code,
    const KinematicsQueryOptions &options,
    const RobotState* context_state) const
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

    // Apply positions
    //m_pState->setJointGroupPositions(m_pJointModelGroup, solution);
    // Enforce joint limits
    //m_pState->enforceBounds();
    // Get effector state
    //const Eigen::Isometry3d& effectorState = m_pState->getGlobalLinkTransform(m_tipFrames[0].c_str());
    // m_pJoint->getMimicRequests: the joint models whose values would be modified if the value of this joint changed.

    /* useful for fk and ik
    virtual void 	computeTransform (const double *joint_values, Eigen::Affine3d &transf) const =0
 	Given the joint values for a joint, compute the corresponding transform.
virtual void 	computeVariablePositions (const Eigen::Affine3d &transform, double *joint_values) const =0
 	Given the transform generated by joint, compute the corresponding joint values.
    */

    return true;
}

IKPluginRegistrar::IKPluginRegistrar()
{
    // PLUGINLIB_EXPORT_CLASS macro does not compile on Noetic
    class_loader::impl::registerPlugin<str1ker::IKPlugin, kinematics::KinematicsBase>(
        "str1ker::IKPlugin", "kinematics::KinematicsBase");
}
