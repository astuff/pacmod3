// Copyright (c) 2019 AutonomouStuff, LLC
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef PACMOD3__PACMOD3_ROS_MSG_HANDLER_HPP_
#define PACMOD3__PACMOD3_ROS_MSG_HANDLER_HPP_

#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <memory>
#include <string>
#include <vector>

#include "pacmod3/pacmod3_common.hpp"

namespace lc = rclcpp_lifecycle;

namespace pacmod3
{

class LockedData
{
public:
  explicit LockedData(unsigned char data_length);

  std::vector<unsigned char> getData() const;
  void setData(std::vector<unsigned char> && new_data);

private:
  std::vector<unsigned char> _data;
  mutable std::mutex _data_mut;
};

class Pacmod3TxRosMsgHandler
{
public:
  void fillAndPublish(
    const uint32_t & can_id,
    const std::string & frame_id,
    const std::shared_ptr<lc::LifecyclePublisherInterface> & pub,
    const std::shared_ptr<Pacmod3TxMsg> & parser_class);

private:
  void fillSystemRptBool(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::SystemRptBool * new_msg,
    const std::string & frame_id);
  void fillSystemRptInt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::SystemRptInt * new_msg,
    const std::string & frame_id);
  void fillSystemRptFloat(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::SystemRptFloat * new_msg,
    const std::string & frame_id);
  void fillComponentRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::ComponentRpt * new_msg,
    const std::string & frame_id);
  void fillSoftwareVersionRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::SoftwareVersionRpt * new_msg,
    const std::string & frame_id);
  void fillMotorRpt1(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::MotorRpt1 * new_msg,
    const std::string & frame_id);
  void fillMotorRpt2(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::MotorRpt2 * new_msg,
    const std::string & frame_id);
  void fillMotorRpt3(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::MotorRpt3 * new_msg,
    const std::string & frame_id);

  void fillCabinClimateRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::CabinClimateRpt * new_msg,
    const std::string & frame_id);
  void fillSafetyBrakeRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::SafetyBrakeRpt * new_msg,
    const std::string & frame_id);
  void fillSafetyFuncCriticalStopRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::SafetyFuncCriticalStopRpt * new_msg,
    const std::string & frame_id);
  void fillSafetyFuncRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::SafetyFuncRpt * new_msg,
    const std::string & frame_id);
  void fillEStopRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::EStopRpt * new_msg,
    const std::string & frame_id);
  void fillWatchdogRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::WatchdogRpt * new_msg,
    const std::string & frame_id);
  void fillWatchdogRpt2(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::WatchdogRpt2 * new_msg,
    const std::string & frame_id);

// Global
  void fillGlobalRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::GlobalRpt * new_msg,
    const std::string & frame_id);
  void fillGlobalRpt2(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::GlobalRpt2 * new_msg,
    const std::string & frame_id);

// System Aux Reports
  void fillAccelAuxRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::AccelAuxRpt * new_msg,
    const std::string & frame_id);
  void fillBrakeAuxRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::BrakeAuxRpt * new_msg,
    const std::string & frame_id);
  void fillBrakeDecelAuxRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::BrakeDecelAuxRpt * new_msg,
    const std::string & frame_id);
  void fillHeadlightAuxRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::HeadlightAuxRpt * new_msg,
    const std::string & frame_id);
  void fillParkingBrakeAuxRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::ParkingBrakeAuxRpt * new_msg,
    const std::string & frame_id);
  void fillShiftAuxRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::ShiftAuxRpt * new_msg,
    const std::string & frame_id);
  void fillSteerAuxRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::SteerAuxRpt * new_msg,
    const std::string & frame_id);
  void fillTurnAuxRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::TurnAuxRpt * new_msg,
    const std::string & frame_id);
  void fillWiperAuxRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::WiperAuxRpt * new_msg,
    const std::string & frame_id);

// Misc Reports
  void fillAngVelRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::AngVelRpt * new_msg,
    const std::string & frame_id);
  void fillDateTimeRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::DateTimeRpt * new_msg,
    const std::string & frame_id);
  void fillDetectedObjectRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::DetectedObjectRpt * new_msg,
    const std::string & frame_id);
  void fillDoorRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::DoorRpt * new_msg,
    const std::string & frame_id);
  void fillDriveTrainRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::DriveTrainRpt * new_msg,
    const std::string & frame_id);
  void fillEngineRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::EngineRpt * new_msg,
    const std::string & frame_id);
  void fillInteriorLightsRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::InteriorLightsRpt * new_msg,
    const std::string & frame_id);
  void fillLatLonHeadingRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::LatLonHeadingRpt * new_msg,
    const std::string & frame_id);
  void fillLinearAccelRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::LinearAccelRpt * new_msg,
    const std::string & frame_id);
  void fillOccupancyRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::OccupancyRpt * new_msg,
    const std::string & frame_id);
  void fillRearLightsRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::RearLightsRpt * new_msg,
    const std::string & frame_id);
  void fillTirePressureRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::TirePressureRpt * new_msg,
    const std::string & frame_id);
  void fillVehicleDynamicsRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::VehicleDynamicsRpt * new_msg,
    const std::string & frame_id);
  void fillVehicleFaultRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::VehicleFaultRpt * new_msg,
    const std::string & frame_id);
  void fillVehicleSpeedRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::VehicleSpeedRpt * new_msg,
    const std::string & frame_id);
  void fillVinRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::VinRpt * new_msg,
    const std::string & frame_id);
  void fillVinRpt2(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::VinRpt2 * new_msg,
    const std::string & frame_id);
  void fillWheelSpeedRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::WheelSpeedRpt * new_msg,
    const std::string & frame_id);
  void fillYawRateRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::YawRateRpt * new_msg,
    const std::string & frame_id);

  void fillSystemCmdLimitRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::SystemCmdLimitRpt * new_msg,
    const std::string & frame_id);
  void fillSteerCmdLimitRpt(
    const std::shared_ptr<Pacmod3TxMsg> & parser_class,
    pacmod_msgs::msg::SteerCmdLimitRpt * new_msg,
    const std::string & frame_id);
};

class Pacmod3RxRosMsgHandler
{
public:
  static std::vector<uint8_t> unpackAndEncode(
    const uint32_t & can_id, const pacmod_msgs::msg::SystemCmdBool::SharedPtr & msg);
  static std::vector<uint8_t> unpackAndEncode(
    const uint32_t & can_id, const pacmod_msgs::msg::SystemCmdFloat::SharedPtr & msg);
  static std::vector<uint8_t> unpackAndEncode(
    const uint32_t & can_id, const pacmod_msgs::msg::SystemCmdInt::SharedPtr & msg);
  static std::vector<uint8_t> unpackAndEncode(
    const uint32_t & can_id, const pacmod_msgs::msg::SteerSystemCmd::SharedPtr & msg);
  static std::vector<uint8_t> unpackAndEncode(
    const uint32_t & can_id, const pacmod_msgs::msg::GlobalCmd::SharedPtr & msg);
  static std::vector<uint8_t> unpackAndEncode(
    const uint32_t & can_id, const pacmod_msgs::msg::BrakeDecelCmd::SharedPtr & msg);
  static std::vector<uint8_t> unpackAndEncode(
    const uint32_t & can_id, const pacmod_msgs::msg::CabinClimateCmd::SharedPtr & msg);
  static std::vector<uint8_t> unpackAndEncode(
    const uint32_t & can_id, const pacmod_msgs::msg::SafetyBrakeCmd::SharedPtr & msg);
  static std::vector<uint8_t> unpackAndEncode(
    const uint32_t & can_id, const pacmod_msgs::msg::SafetyFuncCmd::SharedPtr & msg);
  static std::vector<uint8_t> unpackAndEncode(
    const uint32_t & can_id, const pacmod_msgs::msg::SupervisoryCtrl::SharedPtr & msg);
  static std::vector<uint8_t> unpackAndEncode(
    const uint32_t & can_id, const pacmod_msgs::msg::NotificationCmd::SharedPtr & msg);
};

}  // namespace pacmod3

#endif  // PACMOD3__PACMOD3_ROS_MSG_HANDLER_HPP_
