/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 ikSolver.h

 Inverse Kinematics Solver
 Created 07/07/2023

 Copyright (C) 2023 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <Eigen/Dense>

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

enum JOINTS
{
  BASE,
  SHOULDER,
  ELBOW,
  WRIST
};

/*----------------------------------------------------------*\
| Functions
\*----------------------------------------------------------*/

Eigen::Matrix4d forwardKinematics(Eigen::MatrixXd angles);
Eigen::MatrixXd inverseKinematics(Eigen::Vector3d position);
Eigen::MatrixXd inverseKinematics(Eigen::Matrix4d positionAndOrientation);

} // namespace str1ker
