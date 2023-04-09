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

#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>
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

IKPlugin::IKPlugin() : m_pPlanningGroup(NULL)
{
}

bool IKPlugin::initialize(
    const RobotModel &robot_model,
    const string &group_name,
    const string &base_frame,
    const vector<string> &tip_frames,
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
            planningChains.size());

        return false;
    }
    else
    {
        auto chain = planningChains[0];
        ROS_INFO_NAMED(
            PLUGIN_NAME,
            "Planning Chain: %s -> %s",
            chain.first.c_str(),
            chain.second.c_str());
    }

    // Validate tips
    if (tip_frames.size() != 1)
    {
        ROS_ERROR_NAMED(
            PLUGIN_NAME,
            "Only one tip frame supported, found %ld",
            tip_frames.size());

        return false;
    }

    // Validate joints
    auto joints = m_pPlanningGroup->getJointModels();
    auto jointNames = m_pPlanningGroup->getJointModelNames();

    for (size_t jointIndex = 0;
         jointIndex < joints.size();
         jointIndex++)
    {
        const JointModel *pJoint = joints[jointIndex];

        if (pJoint->getType() == JointModel::REVOLUTE)
        {
            auto limits = pJoint->getVariableBoundsMsg().front();
            auto axis = getJointAxis(pJoint);

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
                limits.max_velocity);

            m_joints.push_back(pJoint);
        }
        else if (pJoint->getType() == JointModel::PRISMATIC)
        {
            auto limits = pJoint->getVariableBoundsMsg()[0];
            auto axis = getJointAxis(pJoint);

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
                limits.max_velocity);

            m_joints.push_back(pJoint);
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
        chainTip.c_str());

    KinematicsBase::storeValues(
        robot_model,
        group_name,
        base_frame,
        chainTips,
        search_discretization);

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

const vector<string> &IKPlugin::getJointNames() const
{
    return m_pPlanningGroup->getJointModelNames();
}

const vector<string> &IKPlugin::getLinkNames() const
{
    return m_pPlanningGroup->getLinkModelNames();
}

bool IKPlugin::getPositionFK(
    const vector<string> &link_names,
    const vector<double> &joint_angles,
    vector<geometry_msgs::Pose> &poses) const
{
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
    const robot_state::RobotState *context_state) const
{
    // Validate initial state
    if (ik_seed_state.size() != m_joints.size())
    {
        ROS_ERROR_NAMED(
            PLUGIN_NAME,
            "Expected state for %ld supported joints, received state for %ld",
            m_joints.size(),
            ik_seed_state.size());

        error_code.val = error_code.NO_IK_SOLUTION;
        return false;
    }
    else
    {
        ROS_INFO_NAMED(
            PLUGIN_NAME,
            "Received seed state for %ld joints",
            ik_seed_state.size()
        );

        for (size_t jointIndex = 0;
            jointIndex < ik_seed_state.size();
            jointIndex++)
        {
            ROS_INFO_NAMED(
                PLUGIN_NAME,
                "  %s (%s): %g",
                m_joints[jointIndex]->getName().c_str(),
                m_joints[jointIndex]->getMimic() ? "mimic" : "active",
                ik_seed_state[jointIndex]
            );
        }
    }

    // Validate poses
    if (ik_poses.size() != 1 || tip_frames_.size() != ik_poses.size())
    {
        ROS_ERROR_NAMED(
            PLUGIN_NAME,
            "Found %ld tips and %ld poses (expected one pose and one tip)",
            tip_frames_.size(),
            ik_poses.size());

        error_code.val = error_code.NO_IK_SOLUTION;
        return false;
    }

    Isometry3d target;
    tf2::fromMsg(ik_poses.front(), target);

    auto startTime = ros::WallTime::now();
    auto activeJoints = m_pPlanningGroup->getActiveJointModels();

    solution.resize(m_joints.size());

    for (size_t jointIndex = 0;
         jointIndex < m_joints.size();
         jointIndex++)
    {
        solution[jointIndex] = ik_seed_state[jointIndex];

        const JointModel& joint = *m_joints[jointIndex];
        const LinkModel& link = *joint.getChildLinkModel();

        if (joint.getMimic() || joint.getType() != JointModel::REVOLUTE)
            continue;

        double jointState = ik_seed_state[jointIndex];
        const Vector3d& axis = getJointAxis(&joint);
        const Isometry3d& world = m_pState->getGlobalLinkTransform(&link);
        const Isometry3d& local = m_pState->getFrameTransform(link.getName());
        const JointLimits& limits = joint.getVariableBoundsMsg().front();

        if (joint.getType() == JointModel::REVOLUTE)
        {
            // Get vector from joint origin to target
            auto delta = target.translation() - world.translation();

            // Get rotation between joint axis and the target
            Matrix3d rotation = Quaterniond::FromTwoVectors(axis, delta)
                .normalized()
                .toRotationMatrix();

            // Compute joint state
            Vector3d angles = rotation
                .eulerAngles(0, 1, 2)
                .cwiseProduct(axis);

            double angle =
                axis.x() > 0.0 ?
                    angles.x() :
                axis.y() > 0.0 ?
                    angles.y() :
                angles.z();

            AngleAxisd axisAngle(M_PI - angle, axis);
            joint.computeVariablePositions(
                Isometry3d(axisAngle),
                &jointState
            );

            ROS_INFO_NAMED(
                PLUGIN_NAME,
                "Calculating IK for Joint\n\t%s:\n\tlocal [%g, %g, %g]\n\tworld [%g %g %g]\n\ttarget [%g %g %g]\n\tangles %g %g %g\n\tseed state %g\n\tjoint state %g\n\tlimits %g -> %g",
                joint.getName().c_str(),

                local.translation().x(),
                local.translation().y(),
                local.translation().z(),

                world.translation().x(),
                world.translation().y(),
                world.translation().z(),

                target.translation().x(),
                target.translation().y(),
                target.translation().z(),

                angles.x(),
                angles.y(),
                angles.z(),

                ik_seed_state[jointIndex],
                jointState,

                limits.min_position,
                limits.max_position
            );
        }

        solution[jointIndex] = clamp(
            jointState, limits.min_position, limits.max_position);

        if ((ros::WallTime::now() - startTime).toSec() >= timeout)
            break;
    }

    // Return solution
    error_code.val = error_code.SUCCESS;

    if(!solution_callback.empty())
        solution_callback(ik_poses.front(), solution, error_code);

    return true;
}

const Vector3d& IKPlugin::getJointAxis(const JointModel* pJoint)
{
    if (pJoint->getType() == JointModel::REVOLUTE)
        return dynamic_cast<const RevoluteJointModel*>(pJoint)->getAxis();
    else
        return dynamic_cast<const PrismaticJointModel*>(pJoint)->getAxis();
}

IKPluginRegistrar::IKPluginRegistrar()
{
    // PLUGINLIB_EXPORT_CLASS macro does not compile on Noetic
    class_loader::impl::registerPlugin<str1ker::IKPlugin, kinematics::KinematicsBase>(
        "str1ker::IKPlugin", "kinematics::KinematicsBase");
}
