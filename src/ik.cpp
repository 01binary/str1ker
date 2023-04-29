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
using namespace visualization_msgs;
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

//
// Constructor
//

IKPlugin::IKPlugin() :
    m_pPlanningGroup(NULL),
    m_node("~"),
    m_pTipLink(NULL),
    m_pMountJoint(NULL),
    m_pShoulderJoint(NULL),
    m_pElbowJoint(NULL),
    m_pWristJoint(NULL),
    m_bVisualize(true)
{
}

//
// Public methods
//

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
    auto chains = m_pPlanningGroup->getConfig().chains_;

    if (chains.size() != 1)
    {
        ROS_ERROR_NAMED(
            PLUGIN_NAME,
            "Only one chain supported in planning group, found %ld",
            chains.size());

        return false;
    }
    
    ROS_INFO_NAMED(
        PLUGIN_NAME,
        "Chain: %s -> %s",
        chains[0].first.c_str(),
        chains[0].second.c_str());

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

            ROS_INFO_NAMED(
                PLUGIN_NAME,
                "Joint %s: %s %sfrom %g to %g",
                jointNames[jointIndex].c_str(),
                "revolute",
                pJoint->getMimic() != NULL ? "mimic " : "",
                limits.min_position,
                limits.max_position);

            m_joints.push_back(pJoint);
        }
        else if (pJoint->getType() == JointModel::PRISMATIC)
        {
            auto limits = pJoint->getVariableBoundsMsg().front();

            ROS_INFO_NAMED(
                PLUGIN_NAME,
                "Joint %s: %s %sfrom %g to %g",
                jointNames[jointIndex].c_str(),
                "prismatic",
                pJoint->getMimic() != NULL ? "mimic " : "",
                limits.min_position,
                limits.max_position);

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
    auto chainTip = chains.front().second;
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

    m_pTipLink = getTipLink();
    m_pMountJoint = getJoint(JointModel::REVOLUTE);
    m_pShoulderJoint = getJoint(JointModel::REVOLUTE, m_pMountJoint);
    m_pElbowJoint = getJoint(JointModel::REVOLUTE, m_pShoulderJoint);
    m_pWristJoint = getJoint(JointModel::REVOLUTE, m_pElbowJoint);

    m_upperArm = getLinkLength(
        m_pShoulderJoint->getChildLinkModel(),
        m_pElbowJoint->getChildLinkModel());
    m_forearm = getLinkLength(
        m_pElbowJoint->getChildLinkModel(),
        m_pWristJoint->getChildLinkModel());
    m_elbowToEffector = getLinkLength(
        m_pElbowJoint->getChildLinkModel(),
        m_pTipLink);
    m_shoulderToEffector = getLinkLength(
        m_pShoulderJoint->getChildLinkModel(),
        m_pTipLink);
    m_shoulderToWrist = getLinkLength(
        m_pShoulderJoint->getChildLinkModel(),
        m_pWristJoint->getChildLinkModel());
    m_wristToEffector = getLinkLength(
        m_pWristJoint->getChildLinkModel(),
        m_pTipLink);

    // Advertise marker publisher
    m_markerPub = m_node.advertise<visualization_msgs::Marker>(
        "visualization_marker",
        10);

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
    if (!validateSeedState(ik_seed_state) || !validateTarget(ik_poses))
    {
        error_code.val = error_code.NO_IK_SOLUTION;
        return false;
    }

    solution = ik_seed_state;

    Vector3d targetWorld = getTarget(ik_poses).translation();
    Vector3d shoulderWorld = m_pState->getGlobalLinkTransform(
        m_pShoulderJoint->getChildLinkModel()).translation();
    Vector3d targetLocal = targetWorld - shoulderWorld;
    double targetNorm = targetLocal.norm();

    double mountAngleRelative = getAngle(targetLocal.x(), targetLocal.y()) - M_PI / 2;
    double mountOffset = getAngle(m_shoulderToEffector.x(), m_shoulderToEffector.y()) - M_PI / 2;
    double mountAngle = mountAngleRelative - mountOffset * 1.5;
    m_pMountJoint->enforcePositionBounds(&mountAngle);

    Isometry3d armRotation = setJointState(
        m_pMountJoint,
        mountAngle < -M_PI
            ? m_pMountJoint->getVariableBounds().front().max_position_
            : mountAngle,
        solution);

    double targetAngle = asin(targetLocal.z() / targetNorm);
    AngleAxisd targetPitch = AngleAxisd(targetAngle, Vector3d::UnitX());
    Vector3d targetWristLocal = armRotation.inverse()
        * targetLocal
        + (targetPitch * -m_wristToEffector);
    Vector3d targetWristWorld = armRotation
        * targetWristLocal
        + shoulderWorld;

    if (m_bVisualize)
    {
        publishLineMarker(0,
            { shoulderWorld, targetWristWorld },
            { 1.0, 0.0, 1.0 });
        publishLineMarker(2,
            { targetWorld, targetWristWorld },
            { 0.0, 1.0, 0.0 });
    }

    double upperArmNorm = m_upperArm.norm();
    double forearmNorm = m_forearm.norm();
    double shoulderEffectorOffset = getAngle(
        m_shoulderToEffector.y(),
        m_shoulderToEffector.z());
    double wristEffectorNorm = m_wristToEffector.norm();
    double targetWristNorm = clamp(
        targetWristLocal.norm(),
        MIN.norm() - wristEffectorNorm,
        MAX.norm() - wristEffectorNorm);
    double shoulderAngle =
        lawOfCosines(upperArmNorm, targetWristNorm, forearmNorm)
        + targetAngle * 1.1
        + shoulderEffectorOffset;

    AngleAxisd shoulderPitch = AngleAxisd(shoulderAngle, Vector3d::UnitX());
    Vector3d elbowAxis = shoulderPitch * Vector3d::UnitY();
    Vector3d elbowLocal = elbowAxis * upperArmNorm;
    Vector3d elbowWorld = shoulderWorld + armRotation * elbowLocal;
    Vector3d elbowToTargetWrist =
        armRotation.inverse() * (targetWristWorld - elbowWorld);

    m_pShoulderJoint->enforcePositionBounds(&shoulderAngle);

    double elbowOffset = getAngle(m_forearm.y(), m_forearm.z());
    double elbowEffectorOffset = getAngle(m_elbowToEffector.y(), m_elbowToEffector.z());
    double elbowAngleRelative = getAngle(elbowToTargetWrist.y(), elbowToTargetWrist.z());
    double elbowAngle = elbowAngleRelative + elbowOffset + elbowEffectorOffset;

    AngleAxisd elbowRotation = AngleAxisd(elbowAngleRelative, Vector3d::UnitX());
    Vector3d wristAxis = elbowRotation * Vector3d::UnitY();
    Vector3d wristLocal = wristAxis * forearmNorm;
    Vector3d wristWorld = elbowWorld + armRotation * wristLocal;

    setJointState(m_pShoulderJoint, shoulderAngle, solution);
    setJointState(m_pElbowJoint, elbowAngle, solution);

    if (m_bVisualize)
    {
        publishLineMarker(3,
            { elbowWorld, shoulderWorld },
            { 1.0, 0.0, 0.0 });
        publishLineMarker(4,
            { elbowWorld, wristWorld },
            { 0.0, 1.0, 1.0 });
    }

    validateSolution(solution);

    error_code.val = error_code.SUCCESS;

    if (!solution_callback.empty())
    {
        solution_callback(ik_poses.front(), solution, error_code);
    }

    return true;
}

//
// Private methods
//

const JointModel* IKPlugin::getJoint(JointModel::JointType type, const JointModel* parent) const
{
    for (auto joint : m_joints)
    {
        if (parent)
        {
            if (joint->getType() == type && joint->getParentLinkModel() == parent->getChildLinkModel())
                return joint;
        }
        else
        {
            if (joint->getType() == type)
                return joint;
        }
    }

    return nullptr;
}

const LinkModel* IKPlugin::getTipLink() const
{
    return m_pPlanningGroup->getLinkModel(tip_frames_.front());
}

bool IKPlugin::validateTarget(const vector<geometry_msgs::Pose>& ik_poses) const
{
    if (ik_poses.size() != 1 || tip_frames_.size() != ik_poses.size())
    {
        ROS_ERROR_NAMED(
            PLUGIN_NAME,
            "Found %ld tips and %ld poses (expected one pose and one tip)",
            tip_frames_.size(),
            ik_poses.size());

        return false;
    }

    return true;
}

Isometry3d IKPlugin::getTarget(const vector<geometry_msgs::Pose>& ik_poses) const
{
    Isometry3d target;
    auto targetPose = ik_poses.back();
    tf2::fromMsg(targetPose, target);

    ROS_DEBUG_NAMED(
        PLUGIN_NAME,
        "IK target %s: %g, %g, %g",
        tip_frames_.front().c_str(),
        targetPose.position.x,
        targetPose.position.y,
        targetPose.position.z
    );

    return target;
}

bool IKPlugin::validateSeedState(const vector<double>& ik_seed_state) const
{
    if (ik_seed_state.size() != m_joints.size())
    {
        ROS_ERROR_NAMED(
            PLUGIN_NAME,
            "Expected seed state for %ld supported joints, received state for %ld",
            m_joints.size(),
            ik_seed_state.size());

        return false;
    }

    ROS_DEBUG_NAMED(
        PLUGIN_NAME,
        "Received seed state for %ld joints",
        ik_seed_state.size()
    );

    for (size_t jointIndex = 0;
        jointIndex < ik_seed_state.size();
        jointIndex++)
    {
        ROS_DEBUG_NAMED(
            PLUGIN_NAME,
            "\t%s (%s): %g",
            m_joints[jointIndex]->getName().c_str(),
            m_joints[jointIndex]->getMimic() ? "mimic" : "active",
            ik_seed_state[jointIndex]
        );
    }

    return true;
}

void IKPlugin::validateSolution(const std::vector<double>& solution) const
{
    const double* next = &solution[0];

    for (const JointModel* joint: m_joints)
    {
        auto limits = joint->getVariableBoundsMsg().front();
        double jointPos = *next++;

        if (jointPos < limits.min_position)
            ROS_WARN("IK constraint %s: %g less than %g", joint->getName().c_str(), jointPos, limits.min_position);
        else if (jointPos > limits.max_position)
            ROS_WARN("IK constraint %s: %g less than %g", joint->getName().c_str(), jointPos, limits.max_position);
    }
}

Vector3d IKPlugin::getLinkLength(const LinkModel* pBaseLink, const LinkModel* pTipLink) const
{
    const Isometry3d& baseLinkPos = m_pState->getGlobalLinkTransform(pBaseLink);
    const Isometry3d& tipLinkPos = m_pState->getGlobalLinkTransform(pTipLink);

    return (tipLinkPos.translation() - baseLinkPos.translation());
}

void IKPlugin::publishLineMarker(int id, vector<Vector3d> points, Vector3d color) const
{
    Marker marker;
    marker.id = id;
    marker.type = Marker::LINE_LIST;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.color.r = color.x();
    marker.color.g = color.y();
    marker.color.b = color.z();
    marker.color.a = 1.0;

    for (Vector3d& point : points)
    {
        geometry_msgs::Point msgPoint;
        msgPoint.x = point.x();
        msgPoint.y = point.y();
        msgPoint.z = point.z();
        marker.points.push_back(msgPoint);
    }

    m_markerPub.publish(marker);
}

//
// Static methods
//

double IKPlugin::toDegrees(double radians)
{
    return radians * 180 / M_PI;
}

double IKPlugin::getAngle(double x, double y)
{
    double angle = atan2(y, x);
    return isnan(angle) ? 0.0 : angle;
}

double IKPlugin::lawOfCosines(double a, double b, double c)
{
    double angle = acos((a * a + b * b - c * c) / (2 * a * b));
    return isnan(angle) ? 0.0 : angle;
}

Isometry3d IKPlugin::setJointState(
    const JointModel* pJoint, double angle, std::vector<double>& states) const
{
    const Vector3d& axis      = getJointAxis(pJoint);
    const JointLimits& limits = pJoint->getVariableBoundsMsg().front();
    double jointState =
        isnan(angle) ? limits.min_position
                     : clamp(angle, limits.min_position, limits.max_position);

    Isometry3d transform = Isometry3d(AngleAxisd(jointState, axis));

    size_t index =
        find(m_joints.begin(), m_joints.end(), pJoint) - m_joints.begin();
    states[index] = jointState;

    ROS_DEBUG_NAMED(PLUGIN_NAME, "IK solution %s: %g [%g] min %g max %g",
        pJoint->getName().c_str(), angle, jointState, limits.min_position,
        limits.max_position);

    if (pJoint->getMimic())
    {
        // Update the master joint
        const JointModel* pMasterJoint = pJoint->getMimic();
        const JointLimits& masterLimits =
            pMasterJoint->getVariableBoundsMsg().front();
        double masterStateRaw =
            (jointState - pJoint->getMimicOffset()) / pJoint->getMimicFactor();
        double masterState = clamp(masterStateRaw, masterLimits.min_position,
            masterLimits.max_position);
        size_t masterIndex =
            find(m_joints.begin(), m_joints.end(), pMasterJoint)
            - m_joints.begin();
        states[masterIndex] = masterState;

        // Update other mimics
        for (auto pMimicJoint : pMasterJoint->getMimicRequests())
        {
            if (pMimicJoint == pJoint)
            {
                continue;
            }

            auto mimicIterator =
                find(m_joints.begin(), m_joints.end(), pMimicJoint);
            if (mimicIterator == m_joints.end())
            {
                continue;
            }

            const JointLimits& mimicLimits =
                pMimicJoint->getVariableBoundsMsg().front();
            size_t mimicIndex = mimicIterator - m_joints.begin();
            double mimicState = masterState * pMimicJoint->getMimicFactor()
                              + pMimicJoint->getMimicOffset();
            states[mimicIndex] = clamp(
                mimicState, mimicLimits.min_position, mimicLimits.max_position);
        }
    }

    return transform;
}

const Vector3d& IKPlugin::getJointAxis(const JointModel* pJoint)
{
    if (pJoint->getType() == JointModel::REVOLUTE)
    {
        return dynamic_cast<const RevoluteJointModel*>(pJoint)->getAxis();
    }
    else
    {
        return dynamic_cast<const PrismaticJointModel*>(pJoint)->getAxis();
    }
}

/*----------------------------------------------------------*\
| IKPluginRegistrar implementation
\*----------------------------------------------------------*/

IKPluginRegistrar::IKPluginRegistrar()
{
    // PLUGINLIB_EXPORT_CLASS macro does not compile on Noetic
    class_loader::impl::registerPlugin<str1ker::IKPlugin, kinematics::KinematicsBase>(
        "str1ker::IKPlugin", "kinematics::KinematicsBase");
}
