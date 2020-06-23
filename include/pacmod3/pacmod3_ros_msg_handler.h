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
      pacmod_msgs::SystemRptBool * new_msg,
      const std::string& frame_id);
  void fillSystemRptInt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::SystemRptInt * new_msg,
      const std::string& frame_id);
  void fillSystemRptFloat(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::SystemRptFloat * new_msg,
      const std::string& frame_id);
  void fillComponentRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::ComponentRpt * new_msg,
      const std::string& frame_id);
  void fillSoftwareVersionRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::SoftwareVersionRpt * new_msg,
      const std::string& frame_id);
  void fillMotorRpt1(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::MotorRpt1 * new_msg,
      const std::string& frame_id);
  void fillMotorRpt2(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::MotorRpt2 * new_msg,
      const std::string& frame_id);
  void fillMotorRpt3(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::MotorRpt3 * new_msg,
      const std::string& frame_id);

  void fillCabinClimateRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::CabinClimateRpt * new_msg,
      const std::string& frame_id);
  void fillSafetyBrakeRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::SafetyBrakeRpt * new_msg,
      const std::string& frame_id);
  void fillSafetyFuncRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::SafetyFuncRpt * new_msg,
      const std::string& frame_id);
  void fillEStopRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::EStopRpt * new_msg,
      const std::string& frame_id);
  void fillWatchdogRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::WatchdogRpt * new_msg,
      const std::string& frame_id);

// Global
  void fillGlobalRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::GlobalRpt * new_msg,
      const std::string& frame_id);
  void fillGlobalRpt2(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::GlobalRpt2 * new_msg,
      const std::string& frame_id);

// System Aux Reports
  void fillAccelAuxRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::AccelAuxRpt * new_msg,
      const std::string& frame_id);
  void fillBrakeAuxRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::BrakeAuxRpt * new_msg,
      const std::string& frame_id);
  void fillBrakeDeccelAuxRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::BrakeDeccelAuxRpt * new_msg,
      const std::string& frame_id);
  void fillHeadlightAuxRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::HeadlightAuxRpt * new_msg,
      const std::string& frame_id);
  void fillShiftAuxRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::ShiftAuxRpt * new_msg,
      const std::string& frame_id);
  void fillSteerAuxRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::SteerAuxRpt * new_msg,
      const std::string& frame_id);
  void fillTurnAuxRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::TurnAuxRpt * new_msg,
      const std::string& frame_id);
  void fillWiperAuxRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::WiperAuxRpt * new_msg,
      const std::string& frame_id);

// Misc Reports
  void fillAngVelRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::AngVelRpt * new_msg,
      const std::string& frame_id);
  void fillDateTimeRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::DateTimeRpt * new_msg,
      const std::string& frame_id);
  void fillDetectedObjectRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::DetectedObjectRpt * new_msg,
      const std::string& frame_id);
  void fillDoorRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::DoorRpt * new_msg,
      const std::string& frame_id);
  void fillDriveTrainRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::DriveTrainRpt * new_msg,
      const std::string& frame_id);
  void fillEngineRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::EngineRpt * new_msg,
      const std::string& frame_id);
  void fillInteriorLightsRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::InteriorLightsRpt * new_msg,
      const std::string& frame_id);
  void fillLatLonHeadingRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::LatLonHeadingRpt * new_msg,
      const std::string& frame_id);
  void fillLinearAccelRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::LinearAccelRpt * new_msg,
      const std::string& frame_id);
  void fillOccupancyRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::OccupancyRpt * new_msg,
      const std::string& frame_id);
  void fillRearLightsRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::RearLightsRpt * new_msg,
      const std::string& frame_id);
  void fillVehicleSpeedRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::VehicleSpeedRpt * new_msg,
      const std::string& frame_id);
  void fillVinRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::VinRpt * new_msg,
      const std::string& frame_id);
  void fillWheelSpeedRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::WheelSpeedRpt * new_msg,
      const std::string& frame_id);
  void fillYawRateRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::YawRateRpt * new_msg,
      const std::string& frame_id);

  void fillSystemCmdLimitRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::SystemCmdLimitRpt * new_msg,
      const std::string& frame_id);
  void fillSteerCmdLimitRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::SteerCmdLimitRpt * new_msg,
      const std::string& frame_id);

// Extra Messages
  void fillHydraulicsAuxRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::HydraulicsAuxRpt * new_msg,
      const std::string& frame_id);
  void fillWorklightsRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::WorklightsRpt * new_msg,
      const std::string& frame_id);
// Optional Debug Messages
  void fillFaultDebugRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::FaultDebugRpt * new_msg,
      const std::string& frame_id);
  void fillJoystickRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::JoystickRpt * new_msg,
      const std::string& frame_id);
  void fillMFAButtonsRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::MFAButtonsRpt * new_msg,
      const std::string& frame_id);

};

class Pacmod3RxRosMsgHandler
{
public:
  static std::vector<uint8_t> unpackAndEncode(const uint32_t& can_id, const pacmod_msgs::SystemCmdBool::ConstPtr& msg);
  static std::vector<uint8_t> unpackAndEncode(const uint32_t& can_id, const pacmod_msgs::SystemCmdFloat::ConstPtr& msg);
  static std::vector<uint8_t> unpackAndEncode(const uint32_t& can_id, const pacmod_msgs::SystemCmdInt::ConstPtr& msg);
  static std::vector<uint8_t> unpackAndEncode(const uint32_t& can_id, const pacmod_msgs::SteerSystemCmd::ConstPtr& msg);
  static std::vector<uint8_t> unpackAndEncode(const uint32_t& can_id, const pacmod_msgs::GlobalCmd::ConstPtr& msg);
  static std::vector<uint8_t> unpackAndEncode(const uint32_t& can_id, const pacmod_msgs::BrakeDeccelCmd::ConstPtr& msg);
  static std::vector<uint8_t> unpackAndEncode(const uint32_t& can_id, const pacmod_msgs::CabinClimateCmd::ConstPtr& msg);
  static std::vector<uint8_t> unpackAndEncode(const uint32_t& can_id, const pacmod_msgs::SafetyBrakeCmd::ConstPtr& msg);
  static std::vector<uint8_t> unpackAndEncode(const uint32_t& can_id, const pacmod_msgs::SafetyFuncCmd::ConstPtr& msg);
  static std::vector<uint8_t> unpackAndEncode(const uint32_t& can_id, const pacmod_msgs::SupervisoryCtrl::ConstPtr& msg);
  static std::vector<uint8_t> unpackAndEncode(const uint32_t& can_id, const pacmod_msgs::NotificationCmd::ConstPtr& msg);

  static std::vector<uint8_t> unpackAndEncode(const uint32_t& can_id, const pacmod_msgs::HydraulicsCmd::ConstPtr& msg);

};
}  // namespace PACMod3
}  // namespace Drivers
}  // namespace AS

#endif  // PACMOD3_PACMOD3_ROS_MSG_HANDLER_H
