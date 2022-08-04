// Copyright (c) 2020 Robotics and Artificial Intelligence Lab, KAIST
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.

#ifndef RAISIN_RAIBOT__MOTOR_DRIVE_HPP_
#define RAISIN_RAIBOT__MOTOR_DRIVE_HPP_

#include <string>
#include <sstream>

#include "pdo.hpp"
#include "motor.hpp"

namespace raisin
{

namespace robot
{

struct PDGain
{
  double p;
  double d;

  operator std::string() const
  {
    std::ostringstream out;
    out << "p: " << p << ", d: " << d;
    return out.str();
  }
};

enum OPMODE
{
  PROFILED_POSITION = 1,
  PROFILED_VELOCITY = 3,
  PROFILED_TORQUE = 4,
  HOMING = 6,
  CYCLIC_SYNC_POSITION = 8,
  CYCLIC_SYNC_VELOCITY = 9,
  CYCLIC_SYNC_TORQUE = 10,
};

template<typename T>
class MotorDrive; //declare the template class MotorDrive

// MotorDrive<Elmo>

class Elmo;

template<>
class MotorDrive<Elmo> // setting the type of the Motordirve -> ELMO
{
public:

  MotorDrive();

  // ~MotorDrive();

  MotorDrive(const MotorDrive &) = delete;

  MotorDrive & operator=(const MotorDrive &) = delete;

  MotorDrive(MotorDrive &&) noexcept = default;

  MotorDrive & operator=(MotorDrive &&) noexcept = default;

  void conn(const std::string & ifname);

  void start();

  void cycle();

  void setOpMode(int mode);

  void process();

  bool isReady();

  void setMaxJointTorque(double jointTorque);

  double getMaxJointTorque() const;

  Motors::size_type motorsCount() const noexcept;

  Motors::iterator motorsBegin() noexcept;

  Motors::iterator motorsEnd() noexcept;

  Motor & getMotor(size_t motorId);

  const Motor & getMotor(size_t motorId) const;

  void setMotorOffset(uint8_t motorID, int32_t offset);

  int32_t getMotorPosition(uint8_t motorID);

  int32_t getMotorPositionFromOffset(uint8_t motorID);

  bool setDesiredMotorTorque(uint8_t motorID, double desiredMotorTorque);

  bool setPdTargetAndFeedForwardTorque(
    uint8_t jointId, double targetPos, double targetVel, const PDGain & gain, double ffTorque = 0);

  // bool setPdTargetAndFeedForwardTorque(
  //   uint8_t jointId, double targetPos, double targetVel = 0, double ffTorque = 0);

  void updateMeasuredJointPosition(uint8_t jointId, double & position);

  void updateMeasuredJointVelocity(uint8_t jointId, double & velocity);

  void updateMotorActualTorque(uint8_t jointId, double & torqueActualValue);

  void updateMotorStatusWord(uint8_t jointId, uint16_t & statusWord);

  void updateMotorErrorCode(uint8_t jointId, uint16_t & errorCode);

  /////////////////////////////////This is the custom area////////////////////////////

  void setAcceleration(uint8_t motorID, int32_t acceleration);
  void setDeceleration(uint8_t motorID, int32_t acceleration);
  void setTargetposition(uint8_t motorID, int32_t targetPosition);
  void setPositionVelocity(uint8_t motorID, int32_t positionvelocity);
  void setTargetVelocity(uint8_t motorID, int32_t targetvelocity);

  int32_t getMotorVelocity(uint8_t motorID);
/////////////////////////////////This is the custom area////////////////////////////


private:

  void connect(const std::string & ifname);

  int findSlaves();

  void setPDOMapping();

  void waitUntilOpState();

  void fetchState(uint8_t motorID, const TxPDO * tx);

  void stateTransition(RxPDO * rx, const TxPDO * tx);

  

  void setTorque(uint8_t motorID, uint32_t torque);

  double compensateMotorTorque(double motorTorque);

private:
  int expectedWKC_;
  bool ready_;
  char IOmap_[4096];
  Motors motors_;
  int16_t maxMotorTorque_;
};

}  // namespace robot

}  // namespace raisin

#endif  // RAISIN_RAIBOT__MOTOR_DRIVE_HPP_
