/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 inverseKinematicsSolver.cpp

 Inverse Kinematics Solver
 Created 07/07/2023

 Copyright (C) 2023 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include "inverseKinematicsSolver.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace Eigen;
using namespace str1ker;

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const double DRUMSTICK_OFFSET = -0.023;
const double DRUMSTICK_LENGTH = 0.305;
const double FOREARM_LENGTH = 0.48059;
const double UPPERARM_LENGTH = 0.4173;
const double SHOULDER_OFFSET_FORWARD = -0.013;
const double SHOULDER_OFFSET_UP = 0.11518;
const double WRIST_ANGLE = 0.959931;

/*----------------------------------------------------------*\
| Functions
\*----------------------------------------------------------*/

Isometry3d getJointMatrix(int joint, double jointVariable)
{
  switch (joint)
  {
    case BASE:
      return (
        Translation3d(0.0, 0.0, 0.0) *
        AngleAxisd(jointVariable, Vector3d::UnitZ()) *
        // Rotate 90 deg on X axis so that all joint variables are rotations on Z axis
        // (makes the geometry compatible with Denavit-Hartenberg representation)
        AngleAxisd(M_PI / 2.0, Vector3d::UnitX())
      );
    case SHOULDER:
      return (
        Translation3d(SHOULDER_OFFSET_FORWARD, SHOULDER_OFFSET_UP, 0.0) *
        AngleAxisd(jointVariable, Vector3d::UnitZ())
      );
    case ELBOW:
      return (
        Translation3d(UPPERARM_LENGTH, 0.0, 0.0) *
        AngleAxisd(jointVariable, Vector3d::UnitZ())
      );
    case WRIST:
      return (
        Translation3d(FOREARM_LENGTH, 0.0, DRUMSTICK_OFFSET) *
        AngleAxisd(jointVariable, Vector3d::UnitZ()) *
        Translation3d(DRUMSTICK_LENGTH, 0.0, 0.0)
      );
    default:
      return Isometry3d::Identity();
  }
}

inline double pow2(double x)
{
  return x * x;
}

Isometry3d str1ker::forwardKinematics(MatrixXd angles)
{
  Isometry3d endEffector = getJointMatrix(0, angles(0, 0));

  for (int n = 1; n < angles.rows(); n++)
  {
    endEffector = endEffector * getJointMatrix(n, angles(n, 0));
  }

  return endEffector;
}

MatrixXd str1ker::inverseKinematics(Matrix4d positionAndOrientation)
{
  // Normal
  double nx = positionAndOrientation(0, 0);
  double ny = positionAndOrientation(1, 0);
  double nz = positionAndOrientation(2, 0);

  // Orientation
  double ox = positionAndOrientation(0, 1);
  double oy = positionAndOrientation(1, 1);
  double oz = positionAndOrientation(2, 1);

  // Approach
  double ax = positionAndOrientation(0, 2);
  double ay = positionAndOrientation(1, 2);
  double az = positionAndOrientation(2, 2);

  // Position
  double px = positionAndOrientation(0, 3);
  double py = positionAndOrientation(1, 3);
  double pz = positionAndOrientation(2, 3);

  // Base angle
  double baseSin = ax;
  double baseCos = -ay;
  double base = atan2(baseSin, baseCos);

  // Shoulder angle
  double shoulderSin =
    az * 5.511622334052241E-2 -
    nz * 1.410336919654 + oz * 9.467180476060177E-1 +
    pz * 2.396357536544452 - 2.7601246105919E-1;

  double shoulderCos = (
    (
      ax * 2.3E-2 -
      ay * 1.3E-2 -
      nx * 5.88533596571614E-1 +
      ox * 3.950654412659912E-1 +
      px
    ) * -2.396357536544452
  ) / ay;

  double shoulder = atan2(shoulderSin, shoulderCos);
  
  // Elbow angle
  double elbow = -acos(
    (
      (
        nx * oz - nz * ox
      )
      *
      (
        nx * shoulderCos * 8.111596779808571E-1 -
        ox * shoulderCos* 1.158455858812925 +
        nz * baseCos * shoulder * 8.111596779808571E-1 -
        oz * baseCos * shoulder * 1.158455858812925
      ) * 7.071067811865475E-1
    )
    /
    (
      (nx * nx) * pow2(shoulderCos) +
      (ox * ox) * pow2(shoulderCos) +
      (nz * nz) * pow2(baseCos) * pow2(shoulder) +
      (oz * oz) * pow2(baseCos) * pow2(shoulder) +
      nx * nz * baseCos * shoulderCos * shoulder * 2.0 +
      ox * oz * baseCos * shoulderCos * shoulder * 2.0
    )
  );

  MatrixXd angles((int)COUNT, 1);
  angles << base, shoulder, elbow, WRIST_ANGLE;

  return angles;
}

MatrixXd str1ker::inverseKinematics(Vector3d position)
{
  const double ORIGIN_X = 0.0;
  const double ORIGIN_Y = 0.0;
  const double ORIGIN_Z = 0.0;

  // Goal
  double goalX = position(0, 0);
  double goalY = position(1, 0);
  double goalZ = position(2, 0);

  // Base
  double angleToGoal = atan2(goalY - ORIGIN_Y, goalX - ORIGIN_X);
  double distanceToGoalXY = sqrt(pow2(goalX - ORIGIN_X) + pow2(goalY - ORIGIN_Y));
  double angleWristOffset = abs(asin(DRUMSTICK_OFFSET / distanceToGoalXY));
  double baseAngle = angleToGoal - angleWristOffset;

  // Shoulder
  double shoulderX = SHOULDER_OFFSET_FORWARD;
  double shoulderZ = SHOULDER_OFFSET_UP;
  double shoulderToGoalDistance = sqrt(pow2(distanceToGoalXY - shoulderX) + pow2(goalZ - shoulderZ));
  double drumstickOffsetZ = sin(WRIST_ANGLE) * DRUMSTICK_LENGTH;
  double drumstickOffsetX = cos(WRIST_ANGLE) * DRUMSTICK_LENGTH;
  double elbowToGoalAngle = atan2(drumstickOffsetZ, FOREARM_LENGTH + drumstickOffsetX);

  double elbowToGoalDistance =
    sqrt(pow2(FOREARM_LENGTH + drumstickOffsetX) +
    pow2(drumstickOffsetZ));

  double innerShoulderAngle = acos(
    (
      pow2(elbowToGoalDistance) -
      pow2(shoulderToGoalDistance) -
      pow2(UPPERARM_LENGTH)
    )
    /
    (-2.0 * shoulderToGoalDistance * UPPERARM_LENGTH)
  );

  double shoulderToGoalZ = goalZ - shoulderZ;
  double shoulderToGoalX = distanceToGoalXY - shoulderX;
  double shoulderToGoalAngle = atan2(shoulderToGoalZ, shoulderToGoalX);
  double shoulderAngle = shoulderToGoalAngle + innerShoulderAngle;

  // Elbow
  double outerElbowAngle = acos(
    (
      pow2(shoulderToGoalDistance) -
      pow2(elbowToGoalDistance) -
      pow2(UPPERARM_LENGTH)
    )
    /
    (-2.0 * elbowToGoalDistance * UPPERARM_LENGTH)
  );

  double innerElbowAngle = outerElbowAngle - elbowToGoalAngle;
  double elbowAngle = -(M_PI - innerElbowAngle);

  MatrixXd angles((int)COUNT, 1);
  angles << baseAngle, shoulderAngle, elbowAngle, WRIST_ANGLE;

  return angles;
}
