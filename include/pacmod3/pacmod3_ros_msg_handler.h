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

#ifndef PACMOD3_PACMOD3_ROS_MSG_HANDLER_H
#define PACMOD3_PACMOD3_ROS_MSG_HANDLER_H

#include <pacmod3/pacmod3_common.h>

#include <string>
#include <vector>
#include <memory>

namespace AS
{
namespace Drivers
{
namespace PACMod3
{
class LockedData
{
public:
  explicit LockedData(unsigned char data_length);

  std::vector<unsigned char> getData() const;
  void setData(std::vector<unsigned char> new_data);

private:
  std::vector<unsigned char> _data;
  mutable std::mutex _data_mut;
};

class Pacmod3TxRosMsgHandler
{
public:
  void fillAndPublish(const uint32_t& can_id,
                      const std::string& frame_id,
                      const ros::Publisher& pub,
                      const std::shared_ptr<Pacmod3TxMsg>& parser_class);

private:
// General Reports
  void fillSystemRptBool(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::SystemRptBool * new_msg,
      const std::string& frame_id);
  void fillSystemRptInt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::SystemRptInt * new_msg,
      const std::string& frame_id);
  void fillSystemRptFloat(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::SystemRptFloat * new_msg,
      const std::string& frame_id);
  void fillComponentRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::ComponentRpt * new_msg,
      const std::string& frame_id);
  void fillSoftwareVersionRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::SoftwareVersionRpt * new_msg,
      const std::string& frame_id);
  void fillMotorRpt1(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::MotorRpt1 * new_msg,
      const std::string& frame_id);
  void fillMotorRpt2(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::MotorRpt2 * new_msg,
      const std::string& frame_id);
  void fillMotorRpt3(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::MotorRpt3 * new_msg,
      const std::string& frame_id);

  void fillCabinClimateRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::CabinClimateRpt * new_msg,
      const std::string& frame_id);
  void fillSafetyBrakeRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::SafetyBrakeRpt * new_msg,
      const std::string& frame_id);
  void fillSafetyFuncCriticalStopRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::SafetyFuncCriticalStopRpt * new_msg,
      const std::string& frame_id);
  void fillSafetyFuncRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::SafetyFuncRpt * new_msg,
      const std::string& frame_id);
  void fillEStopRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::EStopRpt * new_msg,
      const std::string& frame_id);
  void fillWatchdogRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::WatchdogRpt * new_msg,
      const std::string& frame_id);
  void fillWatchdogRpt2(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::WatchdogRpt2 * new_msg,
      const std::string& frame_id);

// Global
  void fillGlobalRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::GlobalRpt * new_msg,
      const std::string& frame_id);
  void fillGlobalRpt2(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::GlobalRpt2 * new_msg,
      const std::string& frame_id);

// System Aux Reports
  void fillAccelAuxRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::AccelAuxRpt * new_msg,
      const std::string& frame_id);
  void fillBrakeAuxRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::BrakeAuxRpt * new_msg,
      const std::string& frame_id);
  void fillBrakeDeccelAuxRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::BrakeDeccelAuxRpt * new_msg,
      const std::string& frame_id);
  void fillHeadlightAuxRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::HeadlightAuxRpt * new_msg,
      const std::string& frame_id);
  void fillParkingBrakeAuxRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::ParkingBrakeAuxRpt * new_msg,
      const std::string& frame_id);
  void fillShiftAuxRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::ShiftAuxRpt * new_msg,
      const std::string& frame_id);
  void fillSteerAuxRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::SteerAuxRpt * new_msg,
      const std::string& frame_id);
  void fillTurnAuxRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::TurnAuxRpt * new_msg,
      const std::string& frame_id);
  void fillWiperAuxRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::WiperAuxRpt * new_msg,
      const std::string& frame_id);

// Misc Reports
  void fillAngVelRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::AngVelRpt * new_msg,
      const std::string& frame_id);
  void fillDateTimeRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::DateTimeRpt * new_msg,
      const std::string& frame_id);
  void fillDetectedObjectRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::DetectedObjectRpt * new_msg,
      const std::string& frame_id);
  void fillDoorRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::DoorRpt * new_msg,
      const std::string& frame_id);
  void fillDriveTrainRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::DriveTrainRpt * new_msg,
      const std::string& frame_id);
  void fillEngineRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::EngineRpt * new_msg,
      const std::string& frame_id);
  void fillInteriorLightsRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::InteriorLightsRpt * new_msg,
      const std::string& frame_id);
  void fillLatLonHeadingRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::LatLonHeadingRpt * new_msg,
      const std::string& frame_id);
  void fillLinearAccelRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::LinearAccelRpt * new_msg,
      const std::string& frame_id);
  void fillOccupancyRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::OccupancyRpt * new_msg,
      const std::string& frame_id);
  void fillOverrideCfgRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::OverrideCfgRpt * new_msg,
      const std::string& frame_id);
  void fillRearLightsRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::RearLightsRpt * new_msg,
      const std::string& frame_id);
  void fillTirePressureRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::TirePressureRpt * new_msg,
      const std::string& frame_id);
  void fillVehDynamicsRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::VehDynamicsRpt * new_msg,
      const std::string& frame_id);
  void fillVehicleSpeedRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::VehicleSpeedRpt * new_msg,
      const std::string& frame_id);
  void fillVinRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::VinRpt * new_msg,
      const std::string& frame_id);
  void fillWheelSpeedRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::WheelSpeedRpt * new_msg,
      const std::string& frame_id);
  void fillYawRateRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::YawRateRpt * new_msg,
      const std::string& frame_id);

  void fillSystemCmdLimitRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::SystemCmdLimitRpt * new_msg,
      const std::string& frame_id);
  void fillSteerCmdLimitRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod3::SteerCmdLimitRpt * new_msg,
      const std::string& frame_id);
};

class Pacmod3RxRosMsgHandler
{
public:
  static std::vector<uint8_t> unpackAndEncode(
      const uint32_t& can_id, const pacmod3::SystemCmdBool::ConstPtr& msg);
  static std::vector<uint8_t> unpackAndEncode(
      const uint32_t& can_id, const pacmod3::SystemCmdFloat::ConstPtr& msg);
  static std::vector<uint8_t> unpackAndEncode(
      const uint32_t& can_id, const pacmod3::SystemCmdInt::ConstPtr& msg);
  static std::vector<uint8_t> unpackAndEncode(
      const uint32_t& can_id, const pacmod3::SteerSystemCmd::ConstPtr& msg);
  static std::vector<uint8_t> unpackAndEncode(
      const uint32_t& can_id, const pacmod3::GlobalCmd::ConstPtr& msg);
  static std::vector<uint8_t> unpackAndEncode(
      const uint32_t& can_id, const pacmod3::BrakeDeccelCmd::ConstPtr& msg);
  static std::vector<uint8_t> unpackAndEncode(
      const uint32_t& can_id, const pacmod3::CabinClimateCmd::ConstPtr& msg);
  static std::vector<uint8_t> unpackAndEncode(
      const uint32_t& can_id, const pacmod3::SafetyBrakeCmd::ConstPtr& msg);
  static std::vector<uint8_t> unpackAndEncode(
      const uint32_t& can_id, const pacmod3::SafetyFuncCmd::ConstPtr& msg);
  static std::vector<uint8_t> unpackAndEncode(
      const uint32_t& can_id, const pacmod3::SupervisoryCtrl::ConstPtr& msg);
  static std::vector<uint8_t> unpackAndEncode(
      const uint32_t& can_id, const pacmod3::NotificationCmd::ConstPtr& msg);
};
}  // namespace PACMod3
}  // namespace Drivers
}  // namespace AS

#endif  // PACMOD3_PACMOD3_ROS_MSG_HANDLER_H
