// Copyright (c) 2022 AutonomouStuff, LLC
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

// LockedData
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

// Pacmod3RosMsgHandler
Pacmod3RosMsgHandler::Pacmod3RosMsgHandler(uint32_t dbc_major_version)
{
  switch (dbc_major_version)
  {
    case (3):
      msg_api_ = std::make_unique<pacmod3_common::Dbc3Api>();
      break;
    case (4):
      msg_api_ = std::make_unique<pacmod3_common::Dbc4Api>();
      break;
    case (5):
      msg_api_ = std::make_unique<pacmod3_common::Dbc5Api>();
      break;
    case (6):
      msg_api_ = std::make_unique<pacmod3_common::Dbc6Api>();
      break;
    case (7):
      msg_api_ = std::make_unique<pacmod3_common::Dbc7Api>();
      break;
    case (8):
      msg_api_ = std::make_unique<pacmod3_common::Dbc8Api>();
      break;
    case (9):
      msg_api_ = std::make_unique<pacmod3_common::Dbc9Api>();
      break;
    case (10):
      msg_api_ = std::make_unique<pacmod3_common::Dbc10Api>();
      break;
    case (11):
      msg_api_ = std::make_unique<pacmod3_common::Dbc11Api>();
      break;
    case (12):
    default:
      msg_api_ = std::make_unique<pacmod3_common::Dbc12Api>();
      break;
  }
  ROS_INFO("Initialized API for DBC version %d", msg_api_->GetDbcVersion());

  // Bool Reports
  parse_functions[HORN_RPT_CANID] =
  parse_functions[PARKING_BRAKE_RPT_CANID] =
  parse_functions[MARKER_LAMP_RPT_CANID] =
  parse_functions[SPRAYER_RPT_CANID] =
  parse_functions[HAZARD_LIGHTS_RPT_CANID] = std::bind(&pacmod3_common::DbcApi::ParseSystemRptBool, std::ref(*msg_api_), std::placeholders::_1);
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
  parse_functions[ENGINE_BRAKE_RPT_CANID] = std::bind(&pacmod3_common::DbcApi::ParseSystemRptInt, std::ref(*msg_api_), std::placeholders::_1);
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
  parse_functions[STEERING_RPT_CANID] = std::bind(&pacmod3_common::DbcApi::ParseSystemRptFloat, std::ref(*msg_api_), std::placeholders::_1);
  pub_functions[ACCEL_RPT_CANID] =
  pub_functions[BRAKE_RPT_CANID] =
  pub_functions[STEERING_RPT_CANID] = std::bind(&Pacmod3RosMsgHandler::ParseAndPublishType<pacmod3_msgs::SystemRptFloat>, this, std::placeholders::_1, std::placeholders::_2);

  // Aux Reports
  parse_functions[ACCEL_AUX_RPT_CANID] = std::bind(&pacmod3_common::DbcApi::ParseAccelAuxRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[BRAKE_AUX_RPT_CANID] = std::bind(&pacmod3_common::DbcApi::ParseBrakeAuxRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[HEADLIGHT_AUX_RPT_CANID] = std::bind(&pacmod3_common::DbcApi::ParseHeadlightAuxRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[SHIFT_AUX_RPT_CANID] = std::bind(&pacmod3_common::DbcApi::ParseShiftAuxRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[STEERING_AUX_RPT_CANID] = std::bind(&pacmod3_common::DbcApi::ParseSteeringAuxRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[TURN_AUX_RPT_CANID] = std::bind(&pacmod3_common::DbcApi::ParseTurnAuxRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[WIPER_AUX_RPT_CANID] = std::bind(&pacmod3_common::DbcApi::ParseWiperAuxRpt, std::ref(*msg_api_), std::placeholders::_1);
  pub_functions[ACCEL_AUX_RPT_CANID] = std::bind(&Pacmod3RosMsgHandler::ParseAndPublishType<pacmod3_msgs::AccelAuxRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[BRAKE_AUX_RPT_CANID] = std::bind(&Pacmod3RosMsgHandler::ParseAndPublishType<pacmod3_msgs::BrakeAuxRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[HEADLIGHT_AUX_RPT_CANID] = std::bind(&Pacmod3RosMsgHandler::ParseAndPublishType<pacmod3_msgs::HeadlightAuxRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[SHIFT_AUX_RPT_CANID] = std::bind(&Pacmod3RosMsgHandler::ParseAndPublishType<pacmod3_msgs::ShiftAuxRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[STEERING_AUX_RPT_CANID] = std::bind(&Pacmod3RosMsgHandler::ParseAndPublishType<pacmod3_msgs::SteeringAuxRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[TURN_AUX_RPT_CANID] = std::bind(&Pacmod3RosMsgHandler::ParseAndPublishType<pacmod3_msgs::TurnAuxRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[WIPER_AUX_RPT_CANID] = std::bind(&Pacmod3RosMsgHandler::ParseAndPublishType<pacmod3_msgs::WiperAuxRpt>, this, std::placeholders::_1, std::placeholders::_2);

  // Component Reports
  parse_functions[COMPONENT_RPT_00_CANID] = std::bind(&pacmod3_common::DbcApi::ParseComponentRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[COMPONENT_RPT_01_CANID] = std::bind(&pacmod3_common::DbcApi::ParseComponentRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[COMPONENT_RPT_02_CANID] = std::bind(&pacmod3_common::DbcApi::ParseComponentRpt, std::ref(*msg_api_), std::placeholders::_1);
  pub_functions[COMPONENT_RPT_00_CANID] = std::bind(&Pacmod3RosMsgHandler::ParseAndPublishType<pacmod3_msgs::ComponentRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[COMPONENT_RPT_01_CANID] = std::bind(&Pacmod3RosMsgHandler::ParseAndPublishType<pacmod3_msgs::ComponentRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[COMPONENT_RPT_02_CANID] = std::bind(&Pacmod3RosMsgHandler::ParseAndPublishType<pacmod3_msgs::ComponentRpt>, this, std::placeholders::_1, std::placeholders::_2);
  
  // Other Reports
  parse_functions[ENGINE_RPT_CANID] = std::bind(&pacmod3_common::DbcApi::ParseEngineRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[ESTOP_RPT_CANID] = std::bind(&pacmod3_common::DbcApi::ParseEStopRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[GLOBAL_RPT_CANID] = std::bind(&pacmod3_common::DbcApi::ParseGlobalRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[GLOBAL_RPT_2_CANID] = std::bind(&pacmod3_common::DbcApi::ParseGlobalRpt2, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[WHEEL_SPEED_RPT_CANID] = std::bind(&pacmod3_common::DbcApi::ParseWheelSpeedRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[VEHICLE_SPEED_RPT_CANID] = std::bind(&pacmod3_common::DbcApi::ParseVehicleSpeedRpt, std::ref(*msg_api_), std::placeholders::_1);
  pub_functions[ENGINE_RPT_CANID] = std::bind(&Pacmod3RosMsgHandler::ParseAndPublishType<pacmod3_msgs::EngineRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[ESTOP_RPT_CANID] = std::bind(&Pacmod3RosMsgHandler::ParseAndPublishType<pacmod3_msgs::EStopRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[GLOBAL_RPT_CANID] = std::bind(&Pacmod3RosMsgHandler::ParseAndPublishType<pacmod3_msgs::GlobalRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[GLOBAL_RPT_2_CANID] = std::bind(&Pacmod3RosMsgHandler::ParseAndPublishType<pacmod3_msgs::GlobalRpt2>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[WHEEL_SPEED_RPT_CANID] = std::bind(&Pacmod3RosMsgHandler::ParseAndPublishType<pacmod3_msgs::WheelSpeedRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[VEHICLE_SPEED_RPT_CANID] = std::bind(&Pacmod3RosMsgHandler::ParseAndPublishType<pacmod3_msgs::VehicleSpeedRpt>, this, std::placeholders::_1, std::placeholders::_2);
}

void Pacmod3RosMsgHandler::ParseAndPublish(const can_msgs::Frame& can_msg, const ros::Publisher& pub)
{
  if (pub_functions.count(can_msg.id))
  {
    pub_functions[can_msg.id](can_msg, pub);
  }
}

}  // namespace pacmod3

