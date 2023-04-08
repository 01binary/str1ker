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
using namespace robot_state;
using namespace moveit_msgs;
using namespace Eigen;
using namespace str1ker;

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

const char PLUGIN_NAME[] = "str1ker::ik";
IKPluginRegistrar g_registerIkPlugin;

/*----------------------------------------------------------*\
| IKPlugin implementation
\*----------------------------------------------------------*/

IKPlugin::IKPlugin():
    m_pPlanningGroup(NULL)
{
}

bool IKPlugin::initialize(
    const RobotModel& robot_model,
    const string& group_name,
    const string& base_frame,
    const vector<string>& tip_frames,
    double search_discretization)
{
    ROS_INFO_NAMED(PLUGIN_NAME, "Str1ker IK Plugin Initializing");

    // Retrieve planning group
    m_pPlanningGroup = robot_model.getJointModelGroup(group_name);

    if (!m_pPlanningGroup)
    {
        ROS_ERROR_NAMED(PLUGIN_NAME, "Failed to retrieve joint model group");
        return false;
    }


    // Validate chains
    auto planningChains = m_pPlanningGroup->getConfig().chains_;

    if (planningChains.size() != 1)
    {
        ROS_ERROR_NAMED(
            PLUGIN_NAME,
            "Only one chain supported in planning group, found %ld",
            planningChains.size()
        );

        return false;
    }
    else
    {
        auto chain = planningChains[0];
        ROS_INFO_NAMED(
            PLUGIN_NAME,
            "Planning Chain: %s -> %s",
            chain.first.c_str(),
            chain.second.c_str()
        );
    }

    // Validate tips
    if (tip_frames.size() != 1)
    {
        ROS_ERROR_NAMED(
            PLUGIN_NAME,
            "Only one tip frame supported, found %ld",
            tip_frames.size()
        );

        return false;
    }

    // Validate joints
    auto joints = m_pPlanningGroup->getJointModels();
    auto jointNames = m_pPlanningGroup->getJointModelNames();

    for (size_t jointIndex = 0;
        jointIndex < joints.size();
        jointIndex++)
    {
        const JointModel* pJoint = joints[jointIndex];

        if (pJoint->getType() == JointModel::REVOLUTE)
        {
            auto limits = pJoint->getVariableBoundsMsg()[0];
            auto pRevolute = dynamic_cast<const RevoluteJointModel*>(pJoint);
            const Vector3d& axis = pRevolute->getAxis();

            ROS_INFO_NAMED(
                PLUGIN_NAME,
                "Joint %s %s %s %saxis %g %g %g, min %g max %g vel %g",
                jointNames[jointIndex].c_str(),
                "revolute",
                pJoint->isPassive() ? "passive" : "active",
                pJoint->getMimic() != NULL ? "mimic " : "",
                axis.x(),
                axis.y(),
                axis.z(),
                limits.min_position,
                limits.max_position,
                limits.max_velocity
            );
        }
        else if (pJoint->getType() == JointModel::PRISMATIC)
        {
            auto limits = pJoint->getVariableBoundsMsg()[0];
            auto pPrismatic = dynamic_cast<const PrismaticJointModel*>(pJoint);
            const Vector3d& axis = pPrismatic->getAxis();

            ROS_INFO_NAMED(
                PLUGIN_NAME,
                "Joint %s %s %s %saxis %g %g %g, min %g max %g vel %g",
                jointNames[jointIndex].c_str(),
                "prismatic",
                pJoint->isPassive() ? "passive" : "active",
                pJoint->getMimic() != NULL ? "mimic " : "",
                axis.x(),
                axis.y(),
                axis.z(),
                limits.min_position,
                limits.max_position,
                limits.max_velocity
            );
        }
        else if (pJoint->getType() != JointModel::FIXED)
        {
            ROS_WARN_NAMED(
                PLUGIN_NAME,
                "Joint %s not supported",
                jointNames[jointIndex].c_str()
            );
        }
    }

    // Validate links
    auto linkNames = m_pPlanningGroup->getLinkModelNames();

    for (size_t linkIndex = 0;
         linkIndex < linkNames.size();
         linkIndex++)
    {
        ROS_INFO_NAMED(PLUGIN_NAME, "Link %s", linkNames[linkIndex].c_str());
    }

    // Load configuration
    vector<string> chainTips;
    auto chainTip = planningChains[0].second;
    chainTips.push_back(chainTip);

    ROS_INFO_NAMED(
        PLUGIN_NAME,
        "Initializing with base %s and tip %s",
        base_frame.c_str(),
        chainTip.c_str()
    );

    KinematicsBase::storeValues(
        robot_model,
        group_name,
        base_frame,
        chainTips,
        search_discretization
    );

    // Initialize state
    m_pState.reset(new robot_state::RobotState(robot_model_));
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
    return m_pPlanningGroup->getJointModelNames();
}

const vector<string>& IKPlugin::getLinkNames() const
{
    return m_pPlanningGroup->getLinkModelNames();
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
        options
    );
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
        options
    );
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
        options
    );
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
        options
    );
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
        options
    );
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
    const robot_state::RobotState* context_state) const
{
    // Validate initial state
    if (ik_seed_state.size() > m_pPlanningGroup->getJointModels().size())
    {
        ROS_ERROR_NAMED(
            PLUGIN_NAME,
            "Expected state for %ld active joints (and possibly %ld mimic joints), received state for %ld",
            m_pPlanningGroup->getActiveJointModels().size(),
            m_pPlanningGroup->getMimicJointModels().size(),
            ik_seed_state.size()
        );

        error_code.val = error_code.NO_IK_SOLUTION;
        return false;
    }

    // Validate poses
    if (ik_poses.size() != 1 || tip_frames_.size() != ik_poses.size())
    {
        ROS_ERROR_NAMED(
            PLUGIN_NAME,
            "Found %ld tips and %ld poses (expected one pose and one tip)",
            tip_frames_.size(),
            ik_poses.size()
        );

        error_code.val = error_code.NO_IK_SOLUTION;
        return false;
    }

    auto pose = ik_poses[0];
    auto startTime = ros::WallTime::now();
    auto joints = m_pPlanningGroup->getJointModels();

    solution.resize(ik_seed_state.size());

    for (size_t jointIndex = 0; jointIndex < ik_seed_state.size(); jointIndex++)
    {
        auto joint = *joints[jointIndex];

        if (joint.getType() == JointModel::REVOLUTE)
        {
            // Get joint in world frame

            // Get angle between joint and pose
        }
        else
        {
            solution[jointIndex] = ik_seed_state[jointIndex];
        }

        if ((ros::WallTime::now() - startTime).toSec() >= timeout) break;
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
