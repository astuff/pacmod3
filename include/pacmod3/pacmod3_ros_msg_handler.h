#ifndef PACMOD3_PACMOD3_ROS_MSG_HANDLER_H
#define PACMOD3_PACMOD3_ROS_MSG_HANDLER_H

/*
* Unpublished Copyright (c) 2009-2019 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

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
  LockedData();

  std::vector<unsigned char> getData() const;
  void setData(std::vector<unsigned char> new_data);

private:
  std::vector<unsigned char> _data;
  mutable std::mutex _data_mut;
};

class Pacmod3TxRosMsgHandler
{
public:
  void fillAndPublish(const int64_t& can_id,
                      const std::string& frame_id,
                      const ros::Publisher& pub,
                      Pacmod3TxMsg * parser_class);

private:
  void fillSystemRptBool(
      Pacmod3TxMsg * parser_class, pacmod_msgs::SystemRptBool * new_msg, const std::string& frame_id);
  void fillSystemRptInt(
      Pacmod3TxMsg * parser_class, pacmod_msgs::SystemRptInt * new_msg, const std::string& frame_id);
  void fillSystemRptFloat(
      Pacmod3TxMsg * parser_class, pacmod_msgs::SystemRptFloat * new_msg, const std::string& frame_id);
  void fillGlobalRpt(
      Pacmod3TxMsg * parser_class, pacmod_msgs::GlobalRpt * new_msg, const std::string& frame_id);
  void fillComponentRpt(
      Pacmod3TxMsg * parser_class, pacmod_msgs::ComponentRpt * new_msg, const std::string& frame_id);
  void fillAccelAuxRpt(
      Pacmod3TxMsg * parser_class, pacmod_msgs::AccelAuxRpt * new_msg, const std::string& frame_id);
  void fillBrakeAuxRpt(
      Pacmod3TxMsg * parser_class, pacmod_msgs::BrakeAuxRpt * new_msg, const std::string& frame_id);
  void fillDateTimeRpt(
      Pacmod3TxMsg * parser_class, pacmod_msgs::DateTimeRpt * new_msg, const std::string& frame_id);
  void fillDetectedObjectRpt(
      Pacmod3TxMsg * parser_class, pacmod_msgs::DetectedObjectRpt * new_msg, const std::string& frame_id);
  void fillDoorRpt(
      Pacmod3TxMsg * parser_class, pacmod_msgs::DoorRpt * new_msg, const std::string& frame_id);
  void fillHeadlightAuxRpt(
      Pacmod3TxMsg * parser_class, pacmod_msgs::HeadlightAuxRpt * new_msg, const std::string& frame_id);
  void fillInteriorLightsRpt(
      Pacmod3TxMsg * parser_class, pacmod_msgs::InteriorLightsRpt * new_msg, const std::string& frame_id);
  void fillLatLonHeadingRpt(
      Pacmod3TxMsg * parser_class, pacmod_msgs::LatLonHeadingRpt * new_msg, const std::string& frame_id);
  void fillMotorRpt1(
      Pacmod3TxMsg * parser_class, pacmod_msgs::MotorRpt1 * new_msg, const std::string& frame_id);
  void fillMotorRpt2(
      Pacmod3TxMsg * parser_class, pacmod_msgs::MotorRpt2 * new_msg, const std::string& frame_id);
  void fillMotorRpt3(
      Pacmod3TxMsg * parser_class, pacmod_msgs::MotorRpt3 * new_msg, const std::string& frame_id);
  void fillOccupancyRpt(
      Pacmod3TxMsg * parser_class, pacmod_msgs::OccupancyRpt * new_msg, const std::string& frame_id);
  void fillRearLightsRpt(
      Pacmod3TxMsg * parser_class, pacmod_msgs::RearLightsRpt * new_msg, const std::string& frame_id);
  void fillShiftAuxRpt(
      Pacmod3TxMsg * parser_class, pacmod_msgs::ShiftAuxRpt * new_msg, const std::string& frame_id);
  void fillSteerAuxRpt(
      Pacmod3TxMsg * parser_class, pacmod_msgs::SteerAuxRpt * new_msg, const std::string& frame_id);
  void fillSteeringPIDRpt1(
      Pacmod3TxMsg * parser_class, pacmod_msgs::SteeringPIDRpt1 * new_msg, const std::string& frame_id);
  void fillSteeringPIDRpt2(
      Pacmod3TxMsg * parser_class, pacmod_msgs::SteeringPIDRpt2 * new_msg, const std::string& frame_id);
  void fillSteeringPIDRpt3(
      Pacmod3TxMsg * parser_class, pacmod_msgs::SteeringPIDRpt3 * new_msg, const std::string& frame_id);
  void fillSteeringPIDRpt4(
      Pacmod3TxMsg * parser_class, pacmod_msgs::SteeringPIDRpt4 * new_msg, const std::string& frame_id);
  void fillTurnAuxRpt(
      Pacmod3TxMsg * parser_class, pacmod_msgs::TurnAuxRpt * new_msg, const std::string& frame_id);
  void fillVehicleSpecificRpt1(
      Pacmod3TxMsg * parser_class, pacmod_msgs::VehicleSpecificRpt1 * new_msg, const std::string& frame_id);
  void fillVehicleDynamicsRpt(
      Pacmod3TxMsg * parser_class, pacmod_msgs::VehicleDynamicsRpt * new_msg, const std::string& frame_id);
  void fillVehicleSpeedRpt(
      Pacmod3TxMsg * parser_class, pacmod_msgs::VehicleSpeedRpt * new_msg, const std::string& frame_id);
  void fillVinRpt(
      Pacmod3TxMsg * parser_class, pacmod_msgs::VinRpt * new_msg, const std::string& frame_id);
  void fillWheelSpeedRpt(
      Pacmod3TxMsg * parser_class, pacmod_msgs::WheelSpeedRpt * new_msg, const std::string& frame_id);
  void fillWiperAuxRpt(
      Pacmod3TxMsg * parser_class, pacmod_msgs::WiperAuxRpt * new_msg, const std::string& frame_id);
  void fillYawRateRpt(
      Pacmod3TxMsg * parser_class, pacmod_msgs::YawRateRpt * new_msg, const std::string& frame_id);
};

class Pacmod3RxRosMsgHandler
{
public:
  static std::vector<uint8_t> unpackAndEncode(const int64_t& can_id, const pacmod_msgs::SystemCmdBool::ConstPtr& msg);
  static std::vector<uint8_t> unpackAndEncode(const int64_t& can_id, const pacmod_msgs::SystemCmdFloat::ConstPtr& msg);
  static std::vector<uint8_t> unpackAndEncode(const int64_t& can_id, const pacmod_msgs::SystemCmdInt::ConstPtr& msg);
  static std::vector<uint8_t> unpackAndEncode(const int64_t& can_id, const pacmod_msgs::SteerSystemCmd::ConstPtr& msg);
};
}  // namespace PACMod3
}  // namespace Drivers
}  // namespace AS

#endif  // PACMOD3_PACMOD3_ROS_MSG_HANDLER_H
