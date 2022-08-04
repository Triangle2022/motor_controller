// Copyright (c) 2020 Robotics and Artificial Intelligence Lab, KAIST
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.

#ifndef RAISIN_RAIBOT__MOTOR_HPP_
#define RAISIN_RAIBOT__MOTOR_HPP_

#include <cmath>
#include <string>
#include <vector>

namespace raisin
{

namespace robot
{

struct Motor
{
  //static constexpr std::string_view product = "RoboDrive ILM85x26 Star Serial";
  static constexpr double resolution = pow(2, 19);  // 524288
  static constexpr double peakCurrent = 28.28;       // A
  static constexpr double contCurrent = 19.8;       // A
  static constexpr double torqueConst = 0.253;      // Nm/A
  static constexpr double torqueScale = 1000.0;
  static constexpr double gearRatio = 1;
  static constexpr double unitToDeg = 360.0 / resolution;
  static constexpr double unitToRad = 2 * M_PI / resolution;
  static constexpr double unitToJointPos = unitToRad / gearRatio;
  static constexpr double unitToMotorPos = 1 / unitToJointPos;
  static constexpr double unitToJointVel = unitToJointPos;
  static constexpr double unitToMotorVel = 1 / unitToJointVel;
  static constexpr double unitToJointToq = contCurrent / torqueScale * torqueConst * gearRatio;
  static constexpr double unitToMotorToq = 1 / unitToJointToq;

  int32_t offset;
  int32_t position;
  int32_t velocity;
  int16_t torque;
  int32_t voltage;
  uint16_t status;
  uint16_t errorCode;
};

using Motors = std::vector<Motor>;

}  // namespace robot

}  // namespace raisin

#endif  // RAISIN_RAIBOT__MOTOR_HPP_
