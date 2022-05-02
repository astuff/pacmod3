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


// TODO: Remove this include:
#include "pacmod3/pacmod3_common.hpp"

#include "pacmod3_dbc_ros_api.h"
#include "pacmod3_dbc3_ros_api.h"
#include "pacmod3_dbc4_ros_api.h"

#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <memory>
#include <string>
#include <vector>


#include <can_msgs/msg/frame.hpp>

// #include <pacmod3_msgs/msg/accel_aux_rpt.hpp>
// #include <pacmod3_msgs/msg/all_system_statuses.hpp>
// #include <pacmod3_msgs/msg/ang_vel_rpt.hpp>
// #include <pacmod3_msgs/msg/brake_aux_rpt.hpp>
// #include <pacmod3_msgs/msg/component_rpt.hpp>
// #include <pacmod3_msgs/msg/date_time_rpt.hpp>
// #include <pacmod3_msgs/msg/detected_object_rpt.hpp>
// #include <pacmod3_msgs/msg/door_rpt.hpp>
// #include <pacmod3_msgs/msg/engine_rpt.hpp>
// #include <pacmod3_msgs/msg/global_cmd.hpp>
// #include <pacmod3_msgs/msg/global_rpt.hpp>
// #include <pacmod3_msgs/msg/headlight_aux_rpt.hpp>
// #include <pacmod3_msgs/msg/interior_lights_rpt.hpp>
// #include <pacmod3_msgs/msg/lat_lon_heading_rpt.hpp>
// #include <pacmod3_msgs/msg/linear_accel_rpt.hpp>
// #include <pacmod3_msgs/msg/motor_rpt1.hpp>
// #include <pacmod3_msgs/msg/motor_rpt2.hpp>
// #include <pacmod3_msgs/msg/motor_rpt3.hpp>
// #include <pacmod3_msgs/msg/notification_cmd.hpp>
// #include <pacmod3_msgs/msg/occupancy_rpt.hpp>
// #include <pacmod3_msgs/msg/rear_lights_rpt.hpp>
// #include <pacmod3_msgs/msg/shift_aux_rpt.hpp>
// #include <pacmod3_msgs/msg/steering_aux_rpt.hpp>
// #include <pacmod3_msgs/msg/steering_cmd.hpp>
// #include <pacmod3_msgs/msg/system_cmd_bool.hpp>
// #include <pacmod3_msgs/msg/system_cmd_float.hpp>
// #include <pacmod3_msgs/msg/system_cmd_int.hpp>
// #include <pacmod3_msgs/msg/system_rpt_bool.hpp>
// #include <pacmod3_msgs/msg/system_rpt_float.hpp>
// #include <pacmod3_msgs/msg/system_rpt_int.hpp>
// #include <pacmod3_msgs/msg/turn_aux_rpt.hpp>
// #include <pacmod3_msgs/msg/vehicle_dynamics_rpt.hpp>
// #include <pacmod3_msgs/msg/vehicle_speed_rpt.hpp>
// #include <pacmod3_msgs/msg/vin_rpt.hpp>
// #include <pacmod3_msgs/msg/wheel_speed_rpt.hpp>
// #include <pacmod3_msgs/msg/wiper_aux_rpt.hpp>
// #include <pacmod3_msgs/msg/yaw_rate_rpt.hpp>


namespace lc = rclcpp_lifecycle;

namespace pacmod3
{

class LockedData
{
public:
  explicit LockedData(unsigned char data_length);

  std::vector<unsigned char> getData() const;
  void setData(const std::vector<unsigned char>& new_data);

private:
  std::vector<unsigned char> _data;
  mutable std::mutex _data_mut;
};

class Pacmod3TxRosMsgHandler
{
public:
  Pacmod3TxRosMsgHandler(uint32_t dbc_major_version);

  template <class RosMsgType>
  void ParseAndPublishType(
    const can_msgs::msg::Frame& can_msg,
    const std::shared_ptr<lc::LifecyclePublisherInterface> & pub);

  void ParseAndPublish(
    const can_msgs::msg::Frame& can_msg,
    const std::shared_ptr<lc::LifecyclePublisherInterface> & pub);

  // std::vector<uint8_t> Encode(
  //   const uint32_t & can_id, const pacmod3_msgs::msg::SystemCmdBool::SharedPtr & msg);

  // Function for packing/encoding data into CAN messages. It's located in the header to force
  // the compiler to instatiate all template types required by the includer.
  template <class RosMsgType>
  can_msgs::msg::Frame Encode(uint32_t can_id, const RosMsgType& msg)
  {
    can_msgs::msg::Frame new_frame;
    new_frame = msg_api_->EncodeCmd(*msg);
    new_frame.id = can_id;
    return new_frame;
  }

private:

  // pacmod3::Dbc12Api msg_parser_;
  // pacmod3::Dbc12Api msg_api_;
  std::unique_ptr<DbcApi> msg_api_;

  // // List of functions for parsing CAN frames into ROS msgs
  // std::map<uint32_t, std::function<std::shared_ptr<void>(const std::shared_ptr<Pacmod3TxMsg>&, const std::string&)>> parse_functions;

  // // List of functions for
  // std::map<uint32_t, std::function<void(const uint32_t&,
  // const std::string& ,const std::shared_ptr<lc::LifecyclePublisherInterface>& ,const std::shared_ptr<Pacmod3TxMsg>&)>> pub_functions;

  // List of functions for parsing CAN frames into ROS msgs
  std::map<uint32_t, std::function<std::shared_ptr<void>(const cn_msgs::Frame&)>> parse_functions;

  // List of function for matching publishers with the type of message they are publishing
  std::map<uint32_t, std::function<void(const cn_msgs::Frame&, const std::shared_ptr<lc::LifecyclePublisherInterface>& )>> pub_functions;

};

// class Pacmod3RxRosMsgHandler
// {
// public:
//   static std::vector<uint8_t> unpackAndEncode(
//     const uint32_t & can_id, const pacmod3_msgs::msg::SystemCmdBool::SharedPtr & msg);
//   static std::vector<uint8_t> unpackAndEncode(
//     const uint32_t & can_id, const pacmod3_msgs::msg::SystemCmdFloat::SharedPtr & msg);
//   static std::vector<uint8_t> unpackAndEncode(
//     const uint32_t & can_id, const pacmod3_msgs::msg::SystemCmdInt::SharedPtr & msg);
//   static std::vector<uint8_t> unpackAndEncode(
//     const uint32_t & can_id, const pacmod3_msgs::msg::SteeringCmd::SharedPtr & msg);
// };

}  // namespace pacmod3

#endif  // PACMOD3__PACMOD3_ROS_MSG_HANDLER_HPP_
