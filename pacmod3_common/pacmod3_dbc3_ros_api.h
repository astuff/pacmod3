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

#ifndef PACMOD3_DBC3_ROS_API_H
#define PACMOD3_DBC3_ROS_API_H

#include "pacmod3_dbc_ros_api.h"

#include <string>
#include <vector>
#include <memory>
#include <mutex>

#define USE_ROS1

#ifdef USE_ROS1

// #include <pacmod3/pacmod3_core.h>
#include <ros/ros.h>

#include <can_msgs/Frame.h>
#include <pacmod3_msgs/SystemCmdBool.h>
#include <pacmod3_msgs/SystemCmdFloat.h>
#include <pacmod3_msgs/SystemCmdInt.h>
#include <pacmod3_msgs/GlobalRpt.h>
#include <pacmod3_msgs/AccelAuxRpt.h>
#include <pacmod3_msgs/AllSystemStatuses.h>
#include <pacmod3_msgs/BrakeAuxRpt.h>
#include <pacmod3_msgs/ComponentRpt.h>
#include <pacmod3_msgs/DateTimeRpt.h>
#include <pacmod3_msgs/DetectedObjectRpt.h>
#include <pacmod3_msgs/DoorRpt.h>
#include <pacmod3_msgs/EngineRpt.h>
#include <pacmod3_msgs/HeadlightAuxRpt.h>
#include <pacmod3_msgs/InteriorLightsRpt.h>
#include <pacmod3_msgs/LatLonHeadingRpt.h>
#include <pacmod3_msgs/MotorRpt1.h>
#include <pacmod3_msgs/MotorRpt2.h>
#include <pacmod3_msgs/MotorRpt3.h>
#include <pacmod3_msgs/OccupancyRpt.h>
#include <pacmod3_msgs/RearLightsRpt.h>
#include <pacmod3_msgs/ShiftAuxRpt.h>
#include <pacmod3_msgs/SteeringAuxRpt.h>
#include <pacmod3_msgs/SteeringCmd.h>
#include <pacmod3_msgs/SystemRptBool.h>
#include <pacmod3_msgs/SystemRptFloat.h>
#include <pacmod3_msgs/SystemRptInt.h>
#include <pacmod3_msgs/TurnAuxRpt.h>
#include <pacmod3_msgs/VehicleDynamicsRpt.h>
#include <pacmod3_msgs/VehicleSpeedRpt.h>
#include <pacmod3_msgs/VinRpt.h>
#include <pacmod3_msgs/WheelSpeedRpt.h>
#include <pacmod3_msgs/WiperAuxRpt.h>
#include <pacmod3_msgs/YawRateRpt.h>

namespace pm_msgs = pacmod3_msgs;
namespace can_msgs = can_msgs;

#endif  // USE_ROS1

#ifdef USE_ROS2

// #include <pacmod3/pacmod3_core.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <pacmod3_msgs/msg/accel_aux_rpt.hpp>
#include <pacmod3_msgs/msg/all_system_statuses.hpp>
#include <pacmod3_msgs/msg/brake_aux_rpt.hpp>
#include <pacmod3_msgs/msg/component_rpt.hpp>
#include <pacmod3_msgs/msg/date_time_rpt.hpp>
#include <pacmod3_msgs/msg/detected_object_rpt.hpp>
#include <pacmod3_msgs/msg/door_rpt.hpp>
#include <pacmod3_msgs/msg/engine_rpt.hpp>
#include <pacmod3_msgs/msg/global_rpt.hpp>
#include <pacmod3_msgs/msg/headlight_aux_rpt.hpp>
#include <pacmod3_msgs/msg/interior_lights_rpt.hpp>
#include <pacmod3_msgs/msg/lat_lon_heading_rpt.hpp>
#include <pacmod3_msgs/msg/motor_rpt1.hpp>
#include <pacmod3_msgs/msg/motor_rpt2.hpp>
#include <pacmod3_msgs/msg/motor_rpt3.hpp>
#include <pacmod3_msgs/msg/occupancy_rpt.hpp>
#include <pacmod3_msgs/msg/rear_lights_rpt.hpp>
#include <pacmod3_msgs/msg/shift_aux_rpt.hpp>
#include <pacmod3_msgs/msg/steering_aux_rpt.hpp>
#include <pacmod3_msgs/msg/steering_cmd.hpp>
#include <pacmod3_msgs/msg/system_cmd_bool.hpp>
#include <pacmod3_msgs/msg/system_cmd_float.hpp>
#include <pacmod3_msgs/msg/system_cmd_int.hpp>
#include <pacmod3_msgs/msg/system_rpt_bool.hpp>
#include <pacmod3_msgs/msg/system_rpt_float.hpp>
#include <pacmod3_msgs/msg/system_rpt_int.hpp>
#include <pacmod3_msgs/msg/turn_aux_rpt.hpp>
#include <pacmod3_msgs/msg/vehicle_dynamics_rpt.hpp>
#include <pacmod3_msgs/msg/vehicle_speed_rpt.hpp>
#include <pacmod3_msgs/msg/vin_rpt.hpp>
#include <pacmod3_msgs/msg/wheel_speed_rpt.hpp>
#include <pacmod3_msgs/msg/wiper_aux_rpt.hpp>
#include <pacmod3_msgs/msg/yaw_rate_rpt.hpp>

namespace pm_msgs = pacmod3_msgs::msg;
namespace can_msgs = can_msgs::msg;

#endif  // USE_ROS2

// namespace pacmod3_common
namespace pacmod3
{

class Dbc3Api : public DbcApi
{
public:
  std::shared_ptr<void> ParseSystemRptBool(const can_msgs::Frame& can_msg) override;
  // std::shared_ptr<void> ParseSystemRptInt(const can_msgs::Frame& can_msg) override;


//   void fillSystemRptFloat(
//       const std::shared_ptr<Pacmod3TxMsg>& parser_class,
//       pm_msgs::SystemRptFloat * new_msg,
//       const std::string& frame_id);
//   void fillGlobalRpt(
//       const std::shared_ptr<Pacmod3TxMsg>& parser_class,
//       pm_msgs::GlobalRpt * new_msg,
//       const std::string& frame_id);
//   void fillComponentRpt(
//       const std::shared_ptr<Pacmod3TxMsg>& parser_class,
//       pm_msgs::ComponentRpt * new_msg,
//       const std::string& frame_id);
//   void fillAccelAuxRpt(
//       const std::shared_ptr<Pacmod3TxMsg>& parser_class,
//       pm_msgs::AccelAuxRpt * new_msg,
//       const std::string& frame_id);
//   void fillBrakeAuxRpt(
//       const std::shared_ptr<Pacmod3TxMsg>& parser_class,
//       pm_msgs::BrakeAuxRpt * new_msg,
//       const std::string& frame_id);
//   void fillDateTimeRpt(
//       const std::shared_ptr<Pacmod3TxMsg>& parser_class,
//       pm_msgs::DateTimeRpt * new_msg,
//       const std::string& frame_id);
//   void fillDetectedObjectRpt(
//       const std::shared_ptr<Pacmod3TxMsg>& parser_class,
//       pm_msgs::DetectedObjectRpt * new_msg,
//       const std::string& frame_id);
//   void fillDoorRpt(
//       const std::shared_ptr<Pacmod3TxMsg>& parser_class,
//       pm_msgs::DoorRpt * new_msg,
//       const std::string& frame_id);
//   void fillEngineRpt(
//       const std::shared_ptr<Pacmod3TxMsg>& parser_class,
//       pm_msgs::EngineRpt * new_msg,
//       const std::string& frame_id);
//   void fillHeadlightAuxRpt(
//       const std::shared_ptr<Pacmod3TxMsg>& parser_class,
//       pm_msgs::HeadlightAuxRpt * new_msg,
//       const std::string& frame_id);
//   void fillInteriorLightsRpt(
//       const std::shared_ptr<Pacmod3TxMsg>& parser_class,
//       pm_msgs::InteriorLightsRpt * new_msg,
//       const std::string& frame_id);
//   void fillLatLonHeadingRpt(
//       const std::shared_ptr<Pacmod3TxMsg>& parser_class,
//       pm_msgs::LatLonHeadingRpt * new_msg,
//       const std::string& frame_id);
//   void fillMotorRpt1(
//       const std::shared_ptr<Pacmod3TxMsg>& parser_class,
//       pm_msgs::MotorRpt1 * new_msg,
//       const std::string& frame_id);
//   void fillMotorRpt2(
//       const std::shared_ptr<Pacmod3TxMsg>& parser_class,
//       pm_msgs::MotorRpt2 * new_msg,
//       const std::string& frame_id);
//   void fillMotorRpt3(
//       const std::shared_ptr<Pacmod3TxMsg>& parser_class,
//       pm_msgs::MotorRpt3 * new_msg,
//       const std::string& frame_id);
//   void fillOccupancyRpt(
//       const std::shared_ptr<Pacmod3TxMsg>& parser_class,
//       pm_msgs::OccupancyRpt * new_msg,
//       const std::string& frame_id);
//   void fillRearLightsRpt(
//       const std::shared_ptr<Pacmod3TxMsg>& parser_class,
//       pm_msgs::RearLightsRpt * new_msg,
//       const std::string& frame_id);
//   void fillShiftAuxRpt(
//       const std::shared_ptr<Pacmod3TxMsg>& parser_class,
//       pm_msgs::ShiftAuxRpt * new_msg,
//       const std::string& frame_id);
//   void fillSteeringAuxRpt(
//       const std::shared_ptr<Pacmod3TxMsg>& parser_class,
//       pm_msgs::SteeringAuxRpt * new_msg,
//       const std::string& frame_id);
//   void fillTurnAuxRpt(
//       const std::shared_ptr<Pacmod3TxMsg>& parser_class,
//       pm_msgs::TurnAuxRpt * new_msg,
//       const std::string& frame_id);
//   void fillVehicleDynamicsRpt(
//       const std::shared_ptr<Pacmod3TxMsg>& parser_class,
//       pm_msgs::VehicleDynamicsRpt * new_msg,
//       const std::string& frame_id);
//   void fillVehicleSpeedRpt(
//       const std::shared_ptr<Pacmod3TxMsg>& parser_class,
//       pm_msgs::VehicleSpeedRpt * new_msg,
//       const std::string& frame_id);
//   void fillVinRpt(
//       const std::shared_ptr<Pacmod3TxMsg>& parser_class,
//       pm_msgs::VinRpt * new_msg,
//       const std::string& frame_id);
//   void fillWheelSpeedRpt(
//       const std::shared_ptr<Pacmod3TxMsg>& parser_class,
//       pm_msgs::WheelSpeedRpt * new_msg,
//       const std::string& frame_id);
//   void fillWiperAuxRpt(
//       const std::shared_ptr<Pacmod3TxMsg>& parser_class,
//       pm_msgs::WiperAuxRpt * new_msg,
//       const std::string& frame_id);
//   void fillYawRateRpt(
//       const std::shared_ptr<Pacmod3TxMsg>& parser_class,
//       pm_msgs::YawRateRpt * new_msg,
//       const std::string& frame_id);


  can_msgs::Frame EncodeSystemCmdBool(const pm_msgs::SystemCmdBool& msg) override;

  // static std::vector<uint8_t>
  //   unpackAndEncode(const uint32_t& can_id, const pm_msgs::SystemCmdBool::ConstPtr& msg);
  // static std::vector<uint8_t>
  //   unpackAndEncode(const uint32_t& can_id, const pm_msgs::SystemCmdFloat::ConstPtr& msg);
  // static std::vector<uint8_t> unpackAndEncode(const uint32_t& can_id, const pm_msgs::SystemCmdInt::ConstPtr& msg);
  // static std::vector<uint8_t> unpackAndEncode(const uint32_t& can_id, const pm_msgs::SteeringCmd::ConstPtr& msg);
};
}  // namespace pacmod3

#endif  // PACMOD3_DBC3_ROS_API_H
