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

#include <pacmod3/pacmod3_ros_msg_handler.h>

#include <vector>
#include <string>
#include <memory>


namespace pacmod3
{

LockedData::LockedData(unsigned char data_length) :
  _data(),
  _data_mut()
{
  _data.assign(data_length, 0);
}

std::vector<unsigned char> LockedData::getData() const
{
  std::lock_guard<std::mutex> lck(_data_mut);
  return _data;
}

void LockedData::setData(std::vector<unsigned char> new_data)
{
  std::lock_guard<std::mutex> lck(_data_mut);
  _data = new_data;
}

Pacmod3RosMsgHandler::Pacmod3RosMsgHandler(uint32_t dbc_major_version)
{
  switch (dbc_major_version)
  {
    case (3):
      msg_api_ = std::make_unique<Dbc3Api>();
      break;
    case (4):
    default:
      msg_api_ = std::make_unique<Dbc4Api>();
      break;
  }

  // Bool Reports
  parse_functions[HornRptMsg::CAN_ID] =
  parse_functions[ParkingBrakeRptMsg::CAN_ID] =
  parse_functions[MarkerLampRptMsg::CAN_ID] =
  parse_functions[SprayerRptMsg::CAN_ID] =
  parse_functions[HazardLightRptMsg::CAN_ID] = std::bind(&DbcApi::ParseSystemRptBool, std::ref(*msg_api_), std::placeholders::_1);
  pub_functions[HornRptMsg::CAN_ID] =
  pub_functions[ParkingBrakeRptMsg::CAN_ID] =
  pub_functions[MarkerLampRptMsg::CAN_ID] =
  pub_functions[SprayerRptMsg::CAN_ID] =
  pub_functions[HazardLightRptMsg::CAN_ID] = std::bind(&Pacmod3RosMsgHandler::ParseAndPublishType<pacmod3_msgs::SystemRptBool>, this, std::placeholders::_1, std::placeholders::_2);

  // Int Reports
  parse_functions[CruiseControlButtonsRptMsg::CAN_ID] =
  parse_functions[DashControlsLeftRptMsg::CAN_ID] =
  parse_functions[DashControlsRightRptMsg::CAN_ID] =
  parse_functions[TurnSignalRptMsg::CAN_ID] =
  parse_functions[RearPassDoorRptMsg::CAN_ID] =
  parse_functions[ShiftRptMsg::CAN_ID] =
  parse_functions[HeadlightRptMsg::CAN_ID] =
  parse_functions[MediaControlsRptMsg::CAN_ID] =
  parse_functions[WiperRptMsg::CAN_ID] =
  parse_functions[EngineBrakeRptMsg::CAN_ID] = std::bind(&DbcApi::ParseSystemRptInt, std::ref(*msg_api_), std::placeholders::_1);
  pub_functions[CruiseControlButtonsRptMsg::CAN_ID] =
  pub_functions[DashControlsLeftRptMsg::CAN_ID] =
  pub_functions[DashControlsRightRptMsg::CAN_ID] =
  pub_functions[TurnSignalRptMsg::CAN_ID] =
  pub_functions[RearPassDoorRptMsg::CAN_ID] =
  pub_functions[ShiftRptMsg::CAN_ID] =
  pub_functions[HeadlightRptMsg::CAN_ID] =
  pub_functions[MediaControlsRptMsg::CAN_ID] =
  pub_functions[WiperRptMsg::CAN_ID] =
  pub_functions[EngineBrakeRptMsg::CAN_ID] = std::bind(&Pacmod3RosMsgHandler::ParseAndPublishType<pacmod3_msgs::SystemRptInt>, this, std::placeholders::_1, std::placeholders::_2);

  // Float Reports
  parse_functions[AccelRptMsg::CAN_ID] =
  parse_functions[BrakeRptMsg::CAN_ID] =
  parse_functions[SteerRptMsg::CAN_ID] = std::bind(&DbcApi::ParseSystemRptFloat, std::ref(*msg_api_), std::placeholders::_1);
  pub_functions[AccelRptMsg::CAN_ID] =
  pub_functions[BrakeRptMsg::CAN_ID] =
  pub_functions[SteerRptMsg::CAN_ID] = std::bind(&Pacmod3RosMsgHandler::ParseAndPublishType<pacmod3_msgs::SystemRptFloat>, this, std::placeholders::_1, std::placeholders::_2);
}

template <class RosMsgType>
void Pacmod3RosMsgHandler::ParseAndPublishType(const can_msgs::Frame& can_msg, const ros::Publisher& pub)
{
  // Call generic fill function from common hybrid lib, cast void pointer return
  if (parse_functions.count(can_msg.id))
  {
    std::shared_ptr<RosMsgType> ros_msg = std::static_pointer_cast<RosMsgType>(parse_functions[can_msg.id](can_msg));

    ros_msg->header.frame_id = "pacmod3";
    ros_msg->header.stamp = can_msg.header.stamp;

    pub.publish(*ros_msg);
  }
}

void Pacmod3RosMsgHandler::ParseAndPublish(const can_msgs::Frame& can_msg, const ros::Publisher& pub)
{
  if (pub_functions.count(can_msg.id))
  {
    pub_functions[can_msg.id](can_msg, pub);
  }
}

// Command messages
can_msgs::Frame Pacmod3RosMsgHandler::Encode(
    const uint32_t& can_id, const pacmod3_msgs::SystemCmdBool::ConstPtr& msg)
{
  // TODO: Check for valid CAN ID. Is this check even necessary? Could possibly change based on DBC?

  can_msgs::Frame new_frame;
  new_frame = msg_api_->EncodeSystemCmdBool(*msg);
  new_frame.id = can_id;

  // return new_frame.data;
  return new_frame;
}

can_msgs::Frame Pacmod3RosMsgHandler::Encode(
    const uint32_t& can_id, const pacmod3_msgs::SystemCmdInt::ConstPtr& msg)
{
  // TODO: Check for valid CAN ID. Is this check even necessary? Could possibly change based on DBC?

  can_msgs::Frame new_frame;
  new_frame = msg_api_->EncodeSystemCmdInt(*msg);
  new_frame.id = can_id;

  // return new_frame.data;
  return new_frame;
}

can_msgs::Frame Pacmod3RosMsgHandler::Encode(
    const uint32_t& can_id, const pacmod3_msgs::SystemCmdFloat::ConstPtr& msg)
{
  // TODO: Check for valid CAN ID. Is this check even necessary? Could possibly change based on DBC?

  can_msgs::Frame new_frame;
  new_frame = msg_api_->EncodeSystemCmdFloat(*msg);
  new_frame.id = can_id;

  // return new_frame.data;
  return new_frame;
}






// std::vector<uint8_t> Pacmod3RosMsgHandler::unpackAndEncode(
//     const uint32_t& can_id, const pacmod3_msgs::SystemCmdFloat::ConstPtr& msg)
// {
//   if (can_id == AccelCmdMsg::CAN_ID)
//   {
//     AccelCmdMsg encoder;
//     encoder.encode(msg->enable,
//                    msg->ignore_overrides,
//                    msg->clear_override,
//                    msg->command);
//     return encoder.data;
//   }
//   else if (can_id == BrakeCmdMsg::CAN_ID)
//   {
//     BrakeCmdMsg encoder;
//     encoder.encode(msg->enable,
//                    msg->ignore_overrides,
//                    msg->clear_override,
//                    msg->command);
//     return encoder.data;
//   }
//   else
//   {
//     std::vector<uint8_t> bad_id;
//     bad_id.assign(8, 0);
//     ROS_ERROR("A float system command matching the provided CAN ID could not be found.");
//     return bad_id;
//   }
// }

// std::vector<uint8_t> Pacmod3RosMsgHandler::unpackAndEncode(
//     const uint32_t& can_id, const pacmod3_msgs::SystemCmdInt::ConstPtr& msg)
// {
//   if (can_id == CruiseControlButtonsCmdMsg::CAN_ID)
//   {
//     CruiseControlButtonsCmdMsg encoder;
//     encoder.encode(msg->enable,
//                    msg->ignore_overrides,
//                    msg->clear_override,
//                    msg->command);
//     return encoder.data;
//   }
//   else if (can_id == DashControlsLeftCmdMsg::CAN_ID)
//   {
//     DashControlsLeftCmdMsg encoder;
//     encoder.encode(msg->enable,
//                    msg->ignore_overrides,
//                    msg->clear_override,
//                    msg->command);
//     return encoder.data;
//   }
//   else if (can_id == DashControlsRightCmdMsg::CAN_ID)
//   {
//     DashControlsRightCmdMsg encoder;
//     encoder.encode(msg->enable,
//                    msg->ignore_overrides,
//                    msg->clear_override,
//                    msg->command);
//     return encoder.data;
//   }
//   else if (can_id == HeadlightCmdMsg::CAN_ID)
//   {
//     HeadlightCmdMsg encoder;
//     encoder.encode(msg->enable,
//                    msg->ignore_overrides,
//                    msg->clear_override,
//                    msg->command);
//     return encoder.data;
//   }
//   else if (can_id == MediaControlsCmdMsg::CAN_ID)
//   {
//     MediaControlsCmdMsg encoder;
//     encoder.encode(msg->enable,
//                    msg->ignore_overrides,
//                    msg->clear_override,
//                    msg->command);
//     return encoder.data;
//   }
//   else if (can_id == ShiftCmdMsg::CAN_ID)
//   {
//     ShiftCmdMsg encoder;
//     encoder.encode(msg->enable,
//                    msg->ignore_overrides,
//                    msg->clear_override,
//                    msg->command);
//     return encoder.data;
//   }
//   else if (can_id == TurnSignalCmdMsg::CAN_ID)
//   {
//     TurnSignalCmdMsg encoder;
//     encoder.encode(msg->enable,
//                    msg->ignore_overrides,
//                    msg->clear_override,
//                    msg->command);
//     return encoder.data;
//   }
//   else if (can_id == RearPassDoorCmdMsg::CAN_ID)
//   {
//     RearPassDoorCmdMsg encoder;
//     encoder.encode(msg->enable,
//                    msg->ignore_overrides,
//                    msg->clear_override,
//                    msg->command);
//     return encoder.data;
//   }
//   else if (can_id == WiperCmdMsg::CAN_ID)
//   {
//     WiperCmdMsg encoder;
//     encoder.encode(msg->enable,
//                    msg->ignore_overrides,
//                    msg->clear_override,
//                    msg->command);
//     return encoder.data;
//   }
//   else if (can_id == EngineBrakeCmdMsg::CAN_ID)
//   {
//     EngineBrakeCmdMsg encoder;
//     encoder.encode(msg->enable,
//                    msg->ignore_overrides,
//                    msg->clear_override,
//                    msg->command);
//     return encoder.data;
//   }
//   else
//   {
//     std::vector<uint8_t> bad_id;
//     bad_id.assign(8, 0);
//     ROS_ERROR("An enum system command matching the provided CAN ID could not be found.");
//     return bad_id;
//   }
// }

std::vector<uint8_t> Pacmod3RosMsgHandler::unpackAndEncode(
    const uint32_t& can_id, const pacmod3_msgs::SteeringCmd::ConstPtr& msg)
{
  if (can_id == SteerCmdMsg::CAN_ID)
  {
    SteerCmdMsg encoder;
    encoder.encode(msg->enable,
                   msg->ignore_overrides,
                   msg->clear_override,
                   msg->command,
                   msg->rotation_rate);
    return encoder.data;
  }
  else
  {
    std::vector<uint8_t> bad_id;
    bad_id.assign(8, 0);
    ROS_ERROR("A steering system command matching the provided CAN ID could not be found.");
    return bad_id;
  }
}

}  // namespace pacmod3

