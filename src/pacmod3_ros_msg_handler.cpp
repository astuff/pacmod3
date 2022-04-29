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
  ROS_INFO("Initialized API for DBC version %d", msg_api_->GetDbcVersion());

  // Bool Reports
  parse_functions[HORN_RPT_CANID] =
  parse_functions[PARKING_BRAKE_RPT_CANID] =
  parse_functions[MARKER_LAMP_RPT_CANID] =
  parse_functions[SPRAYER_RPT_CANID] =
  parse_functions[HAZARD_LIGHTS_RPT_CANID] = std::bind(&DbcApi::ParseSystemRptBool, std::ref(*msg_api_), std::placeholders::_1);
  pub_functions[HORN_RPT_CANID] =
  pub_functions[PARKING_BRAKE_RPT_CANID] =
  pub_functions[MARKER_LAMP_RPT_CANID] =
  pub_functions[SPRAYER_RPT_CANID] =
  pub_functions[HAZARD_LIGHTS_RPT_CANID] = std::bind(&Pacmod3RosMsgHandler::ParseAndPublishType<pacmod3_msgs::SystemRptBool>, this, std::placeholders::_1, std::placeholders::_2);

  // Int Reports
  parse_functions[CRUISE_CONTROL_BUTTONS_RPT_CANID] =
  parse_functions[TURN_RPT_CANID] =
  parse_functions[REAR_PASS_DOOR_RPT_CANID] =
  parse_functions[SHIFT_RPT_CANID] =
  parse_functions[HEADLIGHT_RPT_CANID] =
  parse_functions[MEDIA_CONTROLS_RPT_CANID] =
  parse_functions[WIPER_RPT_CANID] =
  parse_functions[ENGINE_BRAKE_RPT_CANID] = std::bind(&DbcApi::ParseSystemRptInt, std::ref(*msg_api_), std::placeholders::_1);
  pub_functions[CRUISE_CONTROL_BUTTONS_RPT_CANID] =
  pub_functions[TURN_RPT_CANID] =
  pub_functions[REAR_PASS_DOOR_RPT_CANID] =
  pub_functions[SHIFT_RPT_CANID] =
  pub_functions[HEADLIGHT_RPT_CANID] =
  pub_functions[MEDIA_CONTROLS_RPT_CANID] =
  pub_functions[WIPER_RPT_CANID] =
  pub_functions[ENGINE_BRAKE_RPT_CANID] = std::bind(&Pacmod3RosMsgHandler::ParseAndPublishType<pacmod3_msgs::SystemRptInt>, this, std::placeholders::_1, std::placeholders::_2);

  // Float Reports
  parse_functions[ACCEL_RPT_CANID] =
  parse_functions[BRAKE_RPT_CANID] =
  parse_functions[STEERING_RPT_CANID] = std::bind(&DbcApi::ParseSystemRptFloat, std::ref(*msg_api_), std::placeholders::_1);
  pub_functions[ACCEL_RPT_CANID] =
  pub_functions[BRAKE_RPT_CANID] =
  pub_functions[STEERING_RPT_CANID] = std::bind(&Pacmod3RosMsgHandler::ParseAndPublishType<pacmod3_msgs::SystemRptFloat>, this, std::placeholders::_1, std::placeholders::_2);

  // Other Reports
  parse_functions[ENGINE_RPT_CANID] = std::bind(&DbcApi::ParseEngineRpt, std::ref(*msg_api_), std::placeholders::_1);
  pub_functions[ENGINE_RPT_CANID] = std::bind(&Pacmod3RosMsgHandler::ParseAndPublishType<pacmod3_msgs::EngineRpt>, this, std::placeholders::_1, std::placeholders::_2);
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

}  // namespace pacmod3

