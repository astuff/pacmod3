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


#include "pacmod3/pacmod3_common.hpp"
#include "pacmod3_dbc12_ros_api.h"

#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <memory>
#include <string>
#include <vector>



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
  Pacmod3TxRosMsgHandler();

  template <class RosMsgType>
  void ParseAndPublishType(
    const uint32_t & can_id,
    const std::string & frame_id,
    const std::shared_ptr<lc::LifecyclePublisherInterface> & pub,
    const std::shared_ptr<Pacmod3TxMsg> & parser_class);

  void ParseAndPublish(
    const uint32_t & can_id,
    const std::string & frame_id,
    const std::shared_ptr<lc::LifecyclePublisherInterface> & pub,
    const std::shared_ptr<Pacmod3TxMsg> & parser_class);

  std::vector<uint8_t> Encode(
    const uint32_t & can_id, const pacmod3_msgs::msg::SystemCmdBool::SharedPtr & msg);

private:

  pacmod3::Dbc12Api msg_parser_;
  pacmod3::Dbc12Api msg_api_;

  // List of functions for parsing CAN frames into ROS msgs
  std::map<uint32_t, std::function<std::shared_ptr<void>(const std::shared_ptr<Pacmod3TxMsg>&, const std::string&)>> parse_functions;

  // List of functions for
  std::map<uint32_t, std::function<void(const uint32_t&,
  const std::string& ,const std::shared_ptr<lc::LifecyclePublisherInterface>& ,const std::shared_ptr<Pacmod3TxMsg>&)>> pub_functions;
};

class Pacmod3RxRosMsgHandler
{
public:
  static std::vector<uint8_t> unpackAndEncode(
    const uint32_t & can_id, const pacmod3_msgs::msg::SystemCmdBool::SharedPtr & msg);
  static std::vector<uint8_t> unpackAndEncode(
    const uint32_t & can_id, const pacmod3_msgs::msg::SystemCmdFloat::SharedPtr & msg);
  static std::vector<uint8_t> unpackAndEncode(
    const uint32_t & can_id, const pacmod3_msgs::msg::SystemCmdInt::SharedPtr & msg);
  static std::vector<uint8_t> unpackAndEncode(
    const uint32_t & can_id, const pacmod3_msgs::msg::SteeringCmd::SharedPtr & msg);
};

}  // namespace pacmod3

#endif  // PACMOD3__PACMOD3_ROS_MSG_HANDLER_HPP_
