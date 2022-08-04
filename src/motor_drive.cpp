// Copyright (c) 2020 Robotics and Artificial Intelligence Lab, KAIST
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.

#include "motor_drive.hpp"

#include <iostream>

#include <cmath>
#include <chrono>
#include <string>
#include <vector>
#include <sstream>
#include <exception>

//#include "raisim/raisim_message.hpp"
#include "ethercat.h"

namespace raisin
{

namespace robot
{

using namespace std::chrono_literals;

MotorDrive<Elmo>::MotorDrive()
: expectedWKC_(0),
  ready_(false),
  motors_(12),
  maxMotorTorque_(INT16_MAX)
{}

// MotorDrive<Elmo>::~MotorDrive()
// {
//   ec_slave[0].state = EC_STATE_INIT;
//   ec_writestate(0);

//   ec_close();
// }

void MotorDrive<Elmo>::setMaxJointTorque(double jointTorque)
{
  maxMotorTorque_ = jointTorque * Motor::unitToMotorToq;
}

double MotorDrive<Elmo>::getMaxJointTorque() const
{
  return maxMotorTorque_ * Motor::unitToJointToq;
}




void MotorDrive<Elmo>::conn(const std::string & ifname)
{
  connect(ifname);
  findSlaves();
}

void MotorDrive<Elmo>::start()
{
  setPDOMapping();
  waitUntilOpState();
}

void MotorDrive<Elmo>::cycle()
{
  if (isReady()) {
    process();
  }
}

void MotorDrive<Elmo>::connect(const std::string & ifname)
{
  bool ok = ec_init(ifname.c_str());
  //RSFATAL_IF(!ok, "no socket connection on " + ifname + ", execute as root")
}

int MotorDrive<Elmo>::findSlaves()
{
  int slaveCount = ec_config_init(false);
  //RSFATAL_IF(slaveCount < 1, "no slaves found")
  std::cout << slaveCount << " slave " << "founded!" << std::endl;
  return slaveCount;
}

void MotorDrive<Elmo>::setPDOMapping()
{
  ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

  // set PDO mapping
  for (uint8_t i = 1; i <= ec_slavecount; i++) {
    // rxPDO
    {
      uint8_t val = 0;
      ec_SDOwrite(i, rxPDOMapping, 0, false, sizeof(val), &val, EC_TIMEOUTRXM);
      //DRSINFO("rxPDO: init entry number")
    }

    // {
    //   uint32_t val = rxTargetPosition;
    //   ec_SDOwrite(i, rxPDOMapping, 1, false, sizeof(val), &val, EC_TIMEOUTRXM);
    //   //DRSINFO("rxPDO: target position")
    // }

    // {
    //   uint32_t val = rxTargetVelocity;
    //   ec_SDOwrite(i, rxPDOMapping, 2, false, sizeof(val), &val, EC_TIMEOUTRXM);
    //   //DRSINFO("rxPDO: target velocity")
    // }

    {
      uint32_t val = rxTargetTorque;
      ec_SDOwrite(i, rxPDOMapping, 1, false, sizeof(val), &val, EC_TIMEOUTRXM);
     // DRSINFO("rxPDO: target torque")
    }

//     {
//       uint32_t val = rxMaxTorque;
//       ec_SDOwrite(i, rxPDOMapping, 4, false, sizeof(val), &val, EC_TIMEOUTRXM);
//     //  DRSINFO("rxPDO: max torque")
//     }

//     {
//       uint32_t val = rxControlWord;
//       ec_SDOwrite(i, rxPDOMapping, 5, false, sizeof(val), &val, EC_TIMEOUTRXM);
//      // DRSINFO("rxPDO: control word")
//     }

//     {
//       uint32_t val = rxModesOfOperation;
//       ec_SDOwrite(i, rxPDOMapping, 6, false, sizeof(val), &val, EC_TIMEOUTRXM);
//       //DRSINFO("rxPDO: modes of operation")
//     }

// /////////////////////custom area///////////////////
//     {
//       uint32_t val = rxPositionAcceleration;
//       ec_SDOwrite(i, rxPDOMapping, 7, false, sizeof(val), &val, EC_TIMEOUTRXM);
//       //DRSINFO("rxPDO: modes of operation")
//     }

//     {
//       uint32_t val = rxPositionDeceleration;
//       ec_SDOwrite(i, rxPDOMapping, 8, false, sizeof(val), &val, EC_TIMEOUTRXM);
//       //DRSINFO("rxPDO: modes of operation")
//     }

//     {
//       uint32_t val = rxPositionVelocity;
//       ec_SDOwrite(i, rxPDOMapping, 9, false, sizeof(val), &val, EC_TIMEOUTRXM);
//       //DRSINFO("rxPDO: modes of operation")
//     }
// /////////////////////custom area///////////////////

//     {
//       uint8_t val = 6;
//       ec_SDOwrite(i, rxPDOMapping, 0, false, sizeof(val), &val, EC_TIMEOUTRXM);
//       //DRSINFO("rxPDO: number of entry")
//     }

//     {
//       uint8_t val = 0;
//       ec_SDOwrite(i, rxPDOAssign, 0, false, sizeof(val), &val, EC_TIMEOUTRXM);
//       //DRSINFO("rxPDO: init pdo number")
//     }

//     {
//       uint16_t val = rxPDOMapping;
//       ec_SDOwrite(i, rxPDOAssign, 1, false, sizeof(val), &val, EC_TIMEOUTRXM);
//       //DRSINFO("rxPDO: pdo mapping")
//     }

//     {
//       uint8_t val = 1;
//       ec_SDOwrite(i, rxPDOAssign, 0, false, sizeof(val), &val, EC_TIMEOUTRXM);
//      // DRSINFO("rxPDO: number of pdo")
//     }

    // // txPDO
    // {
    //   uint8_t val = 0;
    //   ec_SDOwrite(i, txPDOMapping, 0, false, sizeof(val), &val, EC_TIMEOUTRXM);
    //  // DRSINFO("txPDO: init entry number")
    // }

    // {
    //   uint32_t val = txPositionActualValue;
    //   ec_SDOwrite(i, txPDOMapping, 1, false, sizeof(val), &val, EC_TIMEOUTRXM);
    //  // DRSINFO("txPDO: position actual value")
    // }

    // {
    //   uint32_t val = txTorqueActualValue;
    //   ec_SDOwrite(i, txPDOMapping, 2, false, sizeof(val), &val, EC_TIMEOUTRXM);
    //  // DRSINFO("txPDO: torque actual value")
    // }

    // {
    //   uint32_t val = txStatusWord;
    //   ec_SDOwrite(i, txPDOMapping, 3, false, sizeof(val), &val, EC_TIMEOUTRXM);
    //  // DRSINFO("txPDO: status word")
    // }

    // {
    //   uint32_t val = txModesOfOperationDisplay;
    //   ec_SDOwrite(i, txPDOMapping, 4, false, sizeof(val), &val, EC_TIMEOUTRXM);
    //  // DRSINFO("txPDO: modes of operation display")
    // }

    // {
    //   uint32_t val = txVelocityActualValue;
    //   ec_SDOwrite(i, txPDOMapping, 5, false, sizeof(val), &val, EC_TIMEOUTRXM);
    //  // DRSINFO("txPDO: velocity actual value")
    // }

    // {
    //   uint32_t val = txDcLinkCircuitVoltage;
    //   ec_SDOwrite(i, txPDOMapping, 6, false, sizeof(val), &val, EC_TIMEOUTRXM);
    //  // DRSINFO("rxPDO: dc link circuit voltage")
    // }

    // {
    //   uint32_t val = txErrorCode;
    //   ec_SDOwrite(i, txPDOMapping, 7, false, sizeof(val), &val, EC_TIMEOUTRXM);
    //  // DRSINFO("txPDO: error code")
    // }

    // {
    //   uint8_t val = 7;
    //   ec_SDOwrite(i, txPDOMapping, 0, false, sizeof(val), &val, EC_TIMEOUTRXM);
    //  // DRSINFO("txPDO: number of entry")
    // }

    // {
    //   uint8_t val = 0;
    //   ec_SDOwrite(i, txPDOAssign, 0, false, sizeof(val), &val, EC_TIMEOUTRXM);
    //  // DRSINFO("txPDO: init pdo number")
    // }

    // {
    //   uint16_t val = txPDOMapping;
    //   ec_SDOwrite(i, txPDOAssign, 1, false, sizeof(val), &val, EC_TIMEOUTRXM);
    //  // DRSINFO("txPDO: pdo mapping")
    // }

    // {
    //   uint8_t val = 1;
    //   ec_SDOwrite(i, txPDOAssign, 0, false, sizeof(val), &val, EC_TIMEOUTRXM);
    //  // DRSINFO("txPDO: number of pdo")
    // }
  }

  ec_config_overlap_map(&IOmap_);
  ec_configdc();

  ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
}

void MotorDrive<Elmo>::waitUntilOpState()
{
  expectedWKC_ = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;

  ec_slave[0].state = EC_STATE_OPERATIONAL;
  ec_send_overlap_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);
  ec_writestate(0);

  // wait for all slaves to reach OP state
  uint8_t count = 200;
  uint16_t timeout = 50000;
  do {
    ec_send_overlap_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_statecheck(0, EC_STATE_OPERATIONAL, timeout);
  } while (count-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

  // RSFATAL_IF(
  //   ec_slave[0].state != EC_STATE_OPERATIONAL,
  //   "not all slaves reached operational state")

  if(ec_slave[0].state != EC_STATE_OPERATIONAL)
  {
    std::cout << "not all slaves reached operational state" << std::endl;
  }

  for (uint8_t i = 1; i <= ec_slavecount; i++) {
    auto rx = reinterpret_cast<RxPDO *>(ec_slave[i].outputs);
    // rx->targetPosition = 0;
    // rx->targetVelocity = 0;
    rx->targetTorque = 0;
    // rx->maxTorque = 1000;
    rx->controlWord = 0;
    rx->modesOfOperation = OPMODE::PROFILED_VELOCITY;
    rx->modesOfOperation = OPMODE::PROFILED_TORQUE;
  }
  ready_ = true;
}

void MotorDrive<Elmo>::setOpMode(int mode)
{
  for (uint8_t i = 1; i <= ec_slavecount; i++) {
    auto rx = reinterpret_cast<RxPDO *>(ec_slave[i].outputs);
    rx->modesOfOperation = mode;
  }
  
}

void MotorDrive<Elmo>::process()
{
  ec_send_overlap_processdata();
  int workCount = ec_receive_processdata(EC_TIMEOUTRET);
  if (workCount < expectedWKC_) {
    return;
  }

  for (uint8_t i = 1; i <= ec_slavecount; i++) {
    auto rx = reinterpret_cast<RxPDO *>(ec_slave[i].outputs);
    auto tx = reinterpret_cast<TxPDO *>(ec_slave[i].inputs);

    uint8_t motorID = i - 1;

    fetchState(motorID, tx);

    stateTransition(rx, tx);
  }
}

void MotorDrive<Elmo>::setTorque(uint8_t motorID, uint32_t torque)
{
  uint8_t slaveID = motorID + 1;
  auto rx = reinterpret_cast<RxPDO *>(ec_slave[slaveID].outputs);
  rx->targetTorque = torque;
}

///////////////this is the custom area///////////////

// void MotorDrive<Elmo>::setAcceleration(uint8_t motorID, int32_t acceleration)
// {
//   uint8_t slaveID = motorID + 1;
//   auto rx = reinterpret_cast<RxPDO *>(ec_slave[slaveID].outputs);
//   rx-> PositionAcceleration = acceleration;
// }

// void MotorDrive<Elmo>::setDeceleration(uint8_t motorID, int32_t deceleration)
// {
//   uint8_t slaveID = motorID + 1;
//   auto rx = reinterpret_cast<RxPDO *>(ec_slave[slaveID].outputs);
//   rx-> PositionDeceleration = deceleration;
// }

// void MotorDrive<Elmo>::setTargetposition(uint8_t motorID, int32_t targetPosition)
// {
//   uint8_t slaveID = motorID + 1;
//   auto rx = reinterpret_cast<RxPDO *>(ec_slave[slaveID].outputs);
//   rx->targetPosition = targetPosition;
// }

// void MotorDrive<Elmo>::setPositionVelocity(uint8_t motorID, int32_t positionvelocity)
// {
//   uint8_t slaveID = motorID + 1;
//   auto rx = reinterpret_cast<RxPDO *>(ec_slave[slaveID].outputs);
//   rx->PositionVelocity = positionvelocity;
// }

// void MotorDrive<Elmo>::setTargetVelocity(uint8_t motorID, int32_t targetvelocity)
// {
//   uint8_t slaveID = motorID + 1;
//   auto rx = reinterpret_cast<RxPDO *>(ec_slave[slaveID].outputs);
//   rx->targetVelocity = targetvelocity;
// }


///////////////this is the custom area///////////////

bool MotorDrive<Elmo>::isReady()
{
  return ready_;
}

Motors::iterator MotorDrive<Elmo>::motorsBegin() noexcept
{
  return motors_.begin();
}

Motors::iterator MotorDrive<Elmo>::motorsEnd() noexcept
{
  return motors_.end();
}

Motors::size_type MotorDrive<Elmo>::motorsCount() const noexcept
{
  return motors_.size();
}

Motor & MotorDrive<Elmo>::getMotor(size_t motorId)
{
  return motors_[motorId];
}

const Motor & MotorDrive<Elmo>::getMotor(size_t motorId) const
{
  return motors_[motorId];
}

void MotorDrive<Elmo>::setMotorOffset(uint8_t motorID, int32_t offset)
{
  motors_[motorID].offset = offset;
}

int32_t MotorDrive<Elmo>::getMotorPosition(uint8_t motorID)
{
  return motors_[motorID].position;
}

int32_t MotorDrive<Elmo>::getMotorPositionFromOffset(uint8_t motorID)
{
  return motors_[motorID].position - motors_[motorID].offset;
}

int32_t MotorDrive<Elmo>::getMotorVelocity(uint8_t motorID)
{
  return motors_[motorID].velocity;
}

void MotorDrive<Elmo>::fetchState(uint8_t motorID, const TxPDO * tx)
{
  motors_[motorID].position = tx->positionActualValue;
  motors_[motorID].velocity = tx->velocityActualValue;
  motors_[motorID].torque = tx->torqueActualValue;
  motors_[motorID].voltage = tx->dcLinkCircuitVoltage;
  motors_[motorID].status = tx->statusWord;
  motors_[motorID].errorCode = tx->errorCode;
}

void MotorDrive<Elmo>::stateTransition(RxPDO * rx, const TxPDO * tx)
{
  // drive state machine transisiotns: 0 -> 6 -> 7 -> 15
  if (tx->statusWord >> 2 & 0x01) {
    rx->controlWord = 15;
  } else if (tx->statusWord >> 1 & 0x01) {
    rx->controlWord = 15;
  } else if (tx->statusWord & 0x01) {
    rx->controlWord = 7;
  } else if (tx->statusWord >> 3 & 0x01) {
    rx->controlWord = 128;
  } else {
    rx->controlWord = 6;
  }
}

double MotorDrive<Elmo>::compensateMotorTorque(double motorTorque)
{
  double current = std::abs(motorTorque * Motor::contCurrent / Motor::torqueScale);
  if (current <= 11.5) {
    return motorTorque;
  }
  double torque = 0.2522 * current;  // linear mapping
  double compensatedCurrent = 0.1459 * std::pow(torque, 2) + 3.542 * torque;
  double compensatedMotorTorque = compensatedCurrent / Motor::contCurrent * Motor::torqueScale;
  return std::copysign(compensatedMotorTorque, motorTorque);
}

bool MotorDrive<Elmo>::setDesiredMotorTorque(uint8_t motorID, double desiredMotorTorque)
{
  bool status = isReady();
  //std::cout << status << std::endl;
  if (status == false) {
    return status;
  }

  double motorTorque = desiredMotorTorque;
  //compensateMotorTorque(desiredMotorTorque);

  // if (motorTorque > maxMotorTorque_) {
  //   motorTorque = maxMotorTorque_;
  // } else if (motorTorque < -maxMotorTorque_) {
  //   motorTorque = -maxMotorTorque_;
  // }

  setTorque(motorID, static_cast<uint32_t>(motorTorque));

  return status;
}

bool MotorDrive<Elmo>::setPdTargetAndFeedForwardTorque(
  uint8_t jointId, double targetPos, double targetVel, const PDGain & gain, double ffTorque)
{
  double currentPos = getMotorPositionFromOffset(jointId) * Motor::unitToJointPos;
  double currentVel = getMotorVelocity(jointId) * Motor::unitToJointVel;
  double targetTorque = gain.p * (targetPos - currentPos) +
    gain.d * (targetVel - currentVel) + ffTorque;
  return setDesiredMotorTorque(jointId, targetTorque * Motor::unitToMotorToq);
}

void MotorDrive<Elmo>::updateMeasuredJointPosition(uint8_t jointId, double & position)
{
  position = getMotorPositionFromOffset(jointId) * Motor::unitToJointPos;
}

void MotorDrive<Elmo>::updateMeasuredJointVelocity(uint8_t jointId, double & velocity)
{
  velocity = getMotorVelocity(jointId) * Motor::unitToJointVel;
}

void MotorDrive<Elmo>::updateMotorActualTorque(uint8_t jointId, double & torqueActualValue)
{
  torqueActualValue = motors_[jointId].torque * Motor::unitToJointToq;
}

void MotorDrive<Elmo>::updateMotorStatusWord(uint8_t jointId, uint16_t & statusWord)
{
  statusWord = motors_[jointId].status;
}

void MotorDrive<Elmo>::updateMotorErrorCode(uint8_t jointId, uint16_t & errorCode)
{
  errorCode = motors_[jointId].errorCode;
}


}  // namespace robot

}  // namespace raisin

