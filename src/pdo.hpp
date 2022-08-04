// Copyright (c) 2020 Robotics and Artificial Intelligence Lab, KAIST
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.

#ifndef RAISIN_RAIBOT__PDO_HPP_
#define RAISIN_RAIBOT__PDO_HPP_

namespace raisin
{

namespace robot
{

constexpr uint16_t rxPDOAssign = 0x1c12; //rx -> PDOAssign
constexpr uint16_t rxPDOMapping = 0x1701; //rx -> rxPDOMapping target

constexpr uint32_t rxTargetPosition = 0x607a0020; //rxPDOMapping
constexpr uint32_t rxTargetVelocity = 0x60ff0020; //rxPDOMapping
constexpr uint32_t rxTargetTorque = 0x60710010; //rxPDOMapping
constexpr uint32_t rxMaxTorque = 0x60720010; //rxPDOMapping
constexpr uint32_t rxControlWord = 0x60400010; //rxPDOMapping
constexpr uint32_t rxModesOfOperation = 0x60600008; //rxPDOMapping

//custom code from here *rx
constexpr uint32_t rxPositionAcceleration = 0x60830020; //rxPDOMapping 
constexpr uint32_t rxPositionDeceleration = 0x60840020; //rxPDOMapping
constexpr uint32_t rxPositionVelocity = 0x60810020;

//custom code from here *tx

constexpr uint16_t txPDOAssign = 0x1c13;
constexpr uint16_t txPDOMapping = 0x1b01;

constexpr uint32_t txPositionActualValue = 0x60640020;
constexpr uint32_t txTorqueActualValue = 0x60770010;
constexpr uint32_t txStatusWord = 0x60410010;
constexpr uint32_t txModesOfOperationDisplay = 0x60610008;
constexpr uint32_t txVelocityActualValue = 0x606c0020;
constexpr uint32_t txDcLinkCircuitVoltage = 0x60790020;
constexpr uint32_t txErrorCode = 0x603f0010;

struct __attribute__ ((packed)) RxPDO
{
  int32_t targetPosition;
  int32_t targetVelocity;
  int16_t targetTorque;
  uint16_t maxTorque;
  uint16_t controlWord;
  int8_t modesOfOperation;

  ///////////////////////custom part//////////////////////
  int32_t PositionAcceleration;
  int32_t PositionDeceleration;
  int32_t PositionVelocity;
  ///////////////////////custom part//////////////////////
};

struct __attribute__ ((packed)) TxPDO
{
  int32_t positionActualValue;
  int16_t torqueActualValue;
  uint16_t statusWord;
  int8_t modesOfOperationDisplay;
  int32_t velocityActualValue;
  int32_t dcLinkCircuitVoltage;
  uint16_t errorCode;
};

// TODO(hsyis) : PDO Mapper

}  // namespace robot

}  // namespace raisin

#endif  // RAISIN_RAIBOT__PDO_HPP_
