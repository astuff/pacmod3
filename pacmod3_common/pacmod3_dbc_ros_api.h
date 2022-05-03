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

#ifndef PACMOD3_DBC_ROS_API_H
#define PACMOD3_DBC_ROS_API_H

#include "pacmod3_can_ids.h"

#include <string>
#include <vector>
#include <memory>
#include <mutex>

#ifndef ROS_VERSION
  #define ROS_VERSION 1
#endif

#if ROS_VERSION==1

#include <ros/ros.h>

#include <can_msgs/Frame.h>

#include <pacmod3_msgs/AccelAuxRpt.h>
#include <pacmod3_msgs/AllSystemStatuses.h>
#include <pacmod3_msgs/AngVelRpt.h>
#include <pacmod3_msgs/BrakeAuxRpt.h>
#include <pacmod3_msgs/ComponentRpt.h>
#include <pacmod3_msgs/DateTimeRpt.h>
#include <pacmod3_msgs/DetectedObjectRpt.h>
#include <pacmod3_msgs/DoorRpt.h>
#include <pacmod3_msgs/EngineRpt.h>
#include <pacmod3_msgs/GlobalCmd.h>
#include <pacmod3_msgs/GlobalRpt.h>
#include <pacmod3_msgs/HeadlightAuxRpt.h>
#include <pacmod3_msgs/InteriorLightsRpt.h>
#include <pacmod3_msgs/LatLonHeadingRpt.h>
#include <pacmod3_msgs/LinearAccelRpt.h>
#include <pacmod3_msgs/MotorRpt1.h>
#include <pacmod3_msgs/MotorRpt2.h>
#include <pacmod3_msgs/MotorRpt3.h>
#include <pacmod3_msgs/NotificationCmd.h>
#include <pacmod3_msgs/OccupancyRpt.h>
#include <pacmod3_msgs/RearLightsRpt.h>
#include <pacmod3_msgs/ShiftAuxRpt.h>
#include <pacmod3_msgs/SteeringAuxRpt.h>
#include <pacmod3_msgs/SteeringCmd.h>
#include <pacmod3_msgs/SystemCmdBool.h>
#include <pacmod3_msgs/SystemCmdFloat.h>
#include <pacmod3_msgs/SystemCmdInt.h>
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
namespace cn_msgs = can_msgs;

#endif  // ROS1

#if ROS_VERSION==2

#include <rclcpp/rclcpp.hpp>

#include <can_msgs/msg/frame.hpp>

#include <pacmod3_msgs/msg/accel_aux_rpt.hpp>
#include <pacmod3_msgs/msg/all_system_statuses.hpp>
#include <pacmod3_msgs/msg/ang_vel_rpt.hpp>
#include <pacmod3_msgs/msg/brake_aux_rpt.hpp>
#include <pacmod3_msgs/msg/component_rpt.hpp>
#include <pacmod3_msgs/msg/date_time_rpt.hpp>
#include <pacmod3_msgs/msg/detected_object_rpt.hpp>
#include <pacmod3_msgs/msg/door_rpt.hpp>
#include <pacmod3_msgs/msg/engine_rpt.hpp>
#include <pacmod3_msgs/msg/global_cmd.hpp>
#include <pacmod3_msgs/msg/global_rpt.hpp>
#include <pacmod3_msgs/msg/headlight_aux_rpt.hpp>
#include <pacmod3_msgs/msg/interior_lights_rpt.hpp>
#include <pacmod3_msgs/msg/lat_lon_heading_rpt.hpp>
#include <pacmod3_msgs/msg/linear_accel_rpt.hpp>
#include <pacmod3_msgs/msg/motor_rpt1.hpp>
#include <pacmod3_msgs/msg/motor_rpt2.hpp>
#include <pacmod3_msgs/msg/motor_rpt3.hpp>
#include <pacmod3_msgs/msg/notification_cmd.hpp>
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
namespace cn_msgs = can_msgs::msg;

#endif  // ROS2

// namespace pacmod3_common
namespace pacmod3
{

class DbcApi
{
public:
  // Parsing functions take in a ROS CAN msg and return a pointer to a ROS pacmod msg
  virtual std::shared_ptr<void> ParseAccelAuxRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseAngVelRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseBrakeAuxRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseComponentRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseDateTimeRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseDetectedObjectRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseDoorRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseEngineRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseGlobalRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseHeadlightAuxRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseInteriorLightsRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseLatLonHeadingRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseLinearAccelRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseMotorRpt1(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseMotorRpt2(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseMotorRpt3(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseOccupancyRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseRearLightsRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseShiftAuxRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseSteeringAuxRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseSystemRptBool(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseSystemRptFloat(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseSystemRptInt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseTurnAuxRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseVehicleDynamicsRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseVehicleSpeedRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseVinRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseWheelSpeedRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseWiperAuxRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseYawRateRpt(const cn_msgs::Frame& can_msg) = 0;

  // Encoding functions take in a ROS pacmod msg and return and ROS CAN frame msg.
  virtual cn_msgs::Frame EncodeCmd(const pm_msgs::GlobalCmd& msg) = 0;
  virtual cn_msgs::Frame EncodeCmd(const pm_msgs::NotificationCmd& msg) = 0;
  virtual cn_msgs::Frame EncodeCmd(const pm_msgs::SteeringCmd& msg) = 0;
  virtual cn_msgs::Frame EncodeCmd(const pm_msgs::SystemCmdBool& msg) = 0;
  virtual cn_msgs::Frame EncodeCmd(const pm_msgs::SystemCmdFloat& msg) = 0;
  virtual cn_msgs::Frame EncodeCmd(const pm_msgs::SystemCmdInt& msg) = 0;

  void SetDbcVersion(uint32_t dbc_major_version);
  uint32_t GetDbcVersion();
  void PrintParseError(const std::string& msg_type);
  void PrintEncodeError(const std::string& msg_type);

private:
  uint32_t dbc_major_version_ = 0;
};
}  // namespace pacmod3

#endif  // PACMOD3_DBC_ROS_API_H
