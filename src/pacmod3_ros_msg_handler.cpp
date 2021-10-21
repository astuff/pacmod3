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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <vector>
#include <string>
#include <memory>

#include "pacmod3/pacmod3_ros_msg_handler.hpp"

namespace lc = rclcpp_lifecycle;

namespace pacmod3
{

LockedData::LockedData(unsigned char data_length)
: _data(),
  _data_mut()
{
  _data.assign(data_length, 0);
}

std::vector<unsigned char> LockedData::getData() const
{
  std::lock_guard<std::mutex> lck(_data_mut);
  return _data;
}

void LockedData::setData(std::vector<unsigned char> && new_data)
{
  std::lock_guard<std::mutex> lck(_data_mut);
  _data = new_data;
}

Pacmod3TxRosMsgHandler::Pacmod3TxRosMsgHandler()
{
  parse_functions[HornRptMsg::CAN_ID] = std::bind(&Dbc12Api::ParseSystemRptBool, msg_parser_, std::placeholders::_1, std::placeholders::_2);

  pub_functions[HornRptMsg::CAN_ID] = std::bind(&Pacmod3TxRosMsgHandler::ParseAndPublishType<pacmod3_msgs::msg::SystemRptBool>, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
}

template <class RosMsgType>
void Pacmod3TxRosMsgHandler::ParseAndPublishType(
  const uint32_t & can_id,
  const std::string & frame_id,
  const std::shared_ptr<lc::LifecyclePublisherInterface> & pub,
  const std::shared_ptr<Pacmod3TxMsg> & parser_class)
{
  // Call generic fill function from common hybrid lib, cast void pointer return
  if (parse_functions.count(can_id))
  {
    std::shared_ptr<RosMsgType> ros_msg = std::static_pointer_cast<RosMsgType>(parse_functions[can_id](parser_class, frame_id));

    ros_msg->header.frame_id = frame_id;
    // ros_msg.header.stamp = 0.0;

    // setup publisher
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<RosMsgType>>(pub);

    dc_pub->publish(*ros_msg);
  }
}

void Pacmod3TxRosMsgHandler::ParseAndPublish(
  const uint32_t & can_id,
  const std::string & frame_id,
  const std::shared_ptr<lc::LifecyclePublisherInterface> & pub,
  const std::shared_ptr<Pacmod3TxMsg> & parser_class)
{
  if (pub_functions.count(can_id))
  {
    pub_functions[can_id](can_id, frame_id, pub, parser_class);
  }
}


std::vector<uint8_t> Pacmod3TxRosMsgHandler::Encode(
  const uint32_t & can_id,
  const pacmod3_msgs::msg::SystemCmdBool::SharedPtr & msg)
{
  return msg_api_.EncodeSystemCmdBool(*msg);
}


// Command messages
std::vector<uint8_t> Pacmod3RxRosMsgHandler::unpackAndEncode(
  const uint32_t & can_id,
  const pacmod3_msgs::msg::SystemCmdBool::SharedPtr & msg)
{
  // Note: The clear_faults field has been removed in later DBC versions.
  // It is omitted here.

  // TODO(icolwell-as): should clear_faults be added back in here from global_cmd?

  // if (can_id == HornCmdMsg::CAN_ID) {
  //   HornCmdMsg encoder;
  //   encoder.encode(
  //     msg->enable,
  //     msg->ignore_overrides,
  //     msg->clear_override,
  //     false,
  //     msg->command);
  //   return encoder.data;
  // } else if (can_id == ParkingBrakeCmdMsg::CAN_ID) {
  //   ParkingBrakeCmdMsg encoder;
  //   encoder.encode(
  //     msg->enable,
  //     msg->ignore_overrides,
  //     msg->clear_override,
  //     false,
  //     msg->command);
  //   return encoder.data;
  // } else if (can_id == HazardLightCmdMsg::CAN_ID) {
  //   HazardLightCmdMsg encoder;
  //   encoder.encode(
  //     msg->enable,
  //     msg->ignore_overrides,
  //     msg->clear_override,
  //     false,
  //     msg->command);
  //   return encoder.data;
  // } else {
  //   std::vector<uint8_t> bad_id;
  //   bad_id.assign(8, 0);
  //   return bad_id;
  // }
}

std::vector<uint8_t> Pacmod3RxRosMsgHandler::unpackAndEncode(
  const uint32_t & can_id,
  const pacmod3_msgs::msg::SystemCmdFloat::SharedPtr & msg)
{
  // Note: The clear_faults field has been removed in later DBC versions.
  // It is omitted here.

  // TODO(icolwell-as): should clear_faults be added back in here from global_cmd?

  if (can_id == AccelCmdMsg::CAN_ID) {
    AccelCmdMsg encoder;
    encoder.encode(
      msg->enable,
      msg->ignore_overrides,
      msg->clear_override,
      false,
      msg->command);
    return encoder.data;
  } else if (can_id == BrakeCmdMsg::CAN_ID) {
    BrakeCmdMsg encoder;
    encoder.encode(
      msg->enable,
      msg->ignore_overrides,
      msg->clear_override,
      false,
      msg->command);
    return encoder.data;
  } else {
    std::vector<uint8_t> bad_id;
    bad_id.assign(8, 0);
    return bad_id;
  }
}

std::vector<uint8_t> Pacmod3RxRosMsgHandler::unpackAndEncode(
  const uint32_t & can_id,
  const pacmod3_msgs::msg::SystemCmdInt::SharedPtr & msg)
{
  // Note: The clear_faults field has been removed in later DBC versions.
  // It is omitted here.

  // TODO(icolwell-as): should clear_faults be added back in here from global_cmd?

  if (can_id == CruiseControlButtonsCmdMsg::CAN_ID) {
    CruiseControlButtonsCmdMsg encoder;
    encoder.encode(
      msg->enable,
      msg->ignore_overrides,
      msg->clear_override,
      false,
      msg->command);
    return encoder.data;
  } else if (can_id == DashControlsLeftCmdMsg::CAN_ID) {
    DashControlsLeftCmdMsg encoder;
    encoder.encode(
      msg->enable,
      msg->ignore_overrides,
      msg->clear_override,
      false,
      msg->command);
    return encoder.data;
  } else if (can_id == DashControlsRightCmdMsg::CAN_ID) {
    DashControlsRightCmdMsg encoder;
    encoder.encode(
      msg->enable,
      msg->ignore_overrides,
      msg->clear_override,
      false,
      msg->command);
    return encoder.data;
  } else if (can_id == HeadlightCmdMsg::CAN_ID) {
    HeadlightCmdMsg encoder;
    encoder.encode(
      msg->enable,
      msg->ignore_overrides,
      msg->clear_override,
      false,
      msg->command);
    return encoder.data;
  } else if (can_id == MediaControlsCmdMsg::CAN_ID) {
    MediaControlsCmdMsg encoder;
    encoder.encode(
      msg->enable,
      msg->ignore_overrides,
      msg->clear_override,
      false,
      msg->command);
    return encoder.data;
  } else if (can_id == RearPassDoorCmdMsg::CAN_ID) {
    RearPassDoorCmdMsg encoder;
    encoder.encode(
      msg->enable,
      msg->ignore_overrides,
      msg->clear_override,
      false,
      msg->command);
    return encoder.data;
  } else if (can_id == ShiftCmdMsg::CAN_ID) {
    ShiftCmdMsg encoder;
    encoder.encode(
      msg->enable,
      msg->ignore_overrides,
      msg->clear_override,
      false,
      msg->command);
    return encoder.data;
  } else if (can_id == TurnSignalCmdMsg::CAN_ID) {
    TurnSignalCmdMsg encoder;
    encoder.encode(
      msg->enable,
      msg->ignore_overrides,
      msg->clear_override,
      false,
      msg->command);
    return encoder.data;
  } else if (can_id == WiperCmdMsg::CAN_ID) {
    WiperCmdMsg encoder;
    encoder.encode(
      msg->enable,
      msg->ignore_overrides,
      msg->clear_override,
      false,
      msg->command);
    return encoder.data;
  } else {
    std::vector<uint8_t> bad_id;
    bad_id.assign(8, 0);
    return bad_id;
  }
}

std::vector<uint8_t> Pacmod3RxRosMsgHandler::unpackAndEncode(
  const uint32_t & can_id,
  const pacmod3_msgs::msg::SteeringCmd::SharedPtr & msg)
{
  // Note: The clear_faults field has been removed in later DBC versions.
  // It is omitted here.

  // TODO(icolwell-as): should clear_faults be added back in here from global_cmd?

  if (can_id == SteeringCmdMsg::CAN_ID) {
    SteeringCmdMsg encoder;
    encoder.encode(
      msg->enable,
      msg->ignore_overrides,
      msg->clear_override,
      false,
      msg->command,
      msg->rotation_rate);
    return encoder.data;
  } else {
    std::vector<uint8_t> bad_id;
    bad_id.assign(8, 0);
    return bad_id;
  }
}

}  // namespace pacmod3
