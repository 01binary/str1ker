/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 ik.h

 Inverse Kinematics Plugin
 Created 04/03/2023

 Copyright (C) 2023 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <string>
#include <memory>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/KinematicSolverInfo.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| IKPlugin class
\*----------------------------------------------------------*/

class IKPlugin : public kinematics::KinematicsBase
{
private:
    const double DEFAULT_TIMEOUT = 250.0;
    const bool DEBUG = false;

private:
    ros::NodeHandle m_node;
    ros::Publisher m_markerPub;
    robot_state::RobotStatePtr m_pState;
    const robot_model::JointModelGroup* m_pPlanningGroup;
    std::vector<const robot_model::JointModel*> m_joints;
    const robot_model::JointModel* m_pBaseJoint;
    const robot_model::JointModel* m_pShoulderJoint;
    const robot_model::JointModel* m_pElbowJoint;
    const robot_model::JointModel* m_pWristJoint;

public:
    IKPlugin();

public:
    //
    // Initialization
    //

    virtual bool initialize(
        const moveit::core::RobotModel& robot_model,
        const std::string& group_name,
        const std::string& base_frame,
        const std::vector<std::string>& tip_frames,
        double search_discretization) override;

    //
    // Joints, Links, and Groups
    //

    virtual bool supportsGroup(
        const moveit::core::JointModelGroup *jmg,
        std::string *error_text_out = NULL) const;

    virtual const std::vector<std::string>& getJointNames() const;
    virtual const std::vector<std::string>& getLinkNames() const;

    //
    // Forward Kinematics
    //

    virtual bool getPositionFK(
        const std::vector<std::string> &link_names,
        const std::vector<double> &joint_angles,
        std::vector<geometry_msgs::Pose> &poses) const;

    //
    // Inverse Kinematics
    //

    virtual bool getPositionIK(
        const geometry_msgs::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        std::vector<double> &solution,
        moveit_msgs::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    virtual bool searchPositionIK(
        const geometry_msgs::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        double timeout,
        std::vector<double> &solution,
        moveit_msgs::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    virtual bool searchPositionIK(
        const geometry_msgs::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        double timeout,
        const std::vector<double> &consistency_limits,
        std::vector<double> &solution,
        moveit_msgs::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    virtual bool searchPositionIK(
        const geometry_msgs::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        double timeout,
        std::vector<double> &solution,
        const IKCallbackFn &solution_callback,
        moveit_msgs::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    virtual bool searchPositionIK(
        const geometry_msgs::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        double timeout,
        const std::vector<double> &consistency_limits,
        std::vector<double> &solution,
        const IKCallbackFn &solution_callback,
        moveit_msgs::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    virtual bool searchPositionIK(
        const std::vector<geometry_msgs::Pose> &ik_poses,
        const std::vector<double> &ik_seed_state,
        double timeout,
        const std::vector<double> &consistency_limits,
        std::vector<double> &solution,
        const IKCallbackFn &solution_callback,
        moveit_msgs::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions(),
        const moveit::core::RobotState* context_state = NULL) const;

private:
    //
    // Input utilities
    //

    Eigen::Vector3d getGoalPosition(const std::vector<geometry_msgs::Pose>& ik_poses) const;
    Eigen::Vector3d getOrigin() const;

    const robot_model::JointModel* getJoint(
        robot_model::JointModel::JointType type,
        const robot_model::JointModel* parent = nullptr) const;

    static const Eigen::Vector3d& getJointAxis(const robot_model::JointModel* pJoint);

    //
    // Validation utilities
    //

    bool validateSeedState(const std::vector<double>& ik_seed_state) const;
    bool validateTarget(const std::vector<geometry_msgs::Pose>& ik_poses) const;
    void validateSolution(const std::vector<double>& solution) const;

    //
    // Output utilities
    //

    Eigen::Isometry3d setJointState(
        const robot_model::JointModel* pJoint,
        double value,
        std::vector<double>& states) const;

    static inline double clamp(double value, double low, double high)
    {
        assert(low <= high);

        if (value < low)
            value = low;
        else if (value > high)
            value = high;

        return value;
    }

    void publishLineMarker(
        int id,
        std::vector<Eigen::Vector3d> points,
        Eigen::Vector3d color) const;

};

} // namespace str1ker
