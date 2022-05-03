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

class Pacmod3RosMsgHandler
{
public:
  Pacmod3RosMsgHandler(uint32_t dbc_major_version);

  template <class RosMsgType>
  void ParseAndPublishType(
    const can_msgs::msg::Frame& can_msg,
    const std::shared_ptr<lc::LifecyclePublisherInterface> & pub);

  void ParseAndPublish(
    const can_msgs::msg::Frame& can_msg,
    const std::shared_ptr<lc::LifecyclePublisherInterface> & pub);

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
  std::unique_ptr<DbcApi> msg_api_;

  // List of functions for parsing CAN frames into ROS msgs
  std::map<uint32_t, std::function<std::shared_ptr<void>(const can_msgs::msg::Frame&)>> parse_functions;

  // List of function for matching publishers with the type of message they are publishing
  std::map<uint32_t, std::function<void(const can_msgs::msg::Frame&, const std::shared_ptr<lc::LifecyclePublisherInterface>& )>> pub_functions;
};

}  // namespace pacmod3

#endif  // PACMOD3__PACMOD3_ROS_MSG_HANDLER_HPP_
