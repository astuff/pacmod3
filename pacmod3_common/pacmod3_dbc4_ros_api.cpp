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

#include <pacmod3_dbc4_ros_api.h>
#include <autogen/pacmod4.h>

#include <vector>
#include <string>
#include <memory>


namespace pacmod3
{
Dbc4Api::Dbc4Api()
{
  SetDbcVersion(4);
}

std::shared_ptr<void> Dbc4Api::ParseAngVelRpt(const cn_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::AngVelRpt> new_msg( new pm_msgs::AngVelRpt() );

  ANG_VEL_RPT_t parsed_rpt;
  Unpack_ANG_VEL_RPT_pacmod4(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->pitch_new_data_rx = parsed_rpt.PITCH_NEW_DATA_RX;
  new_msg->roll_new_data_rx = parsed_rpt.ROLL_NEW_DATA_RX;
  new_msg->yaw_new_data_rx = parsed_rpt.YAW_NEW_DATA_RX;

  new_msg->pitch_valid = parsed_rpt.PITCH_VALID;
  new_msg->roll_valid = parsed_rpt.ROLL_VALID;
  new_msg->yaw_valid = parsed_rpt.YAW_VALID;

  new_msg->pitch_vel = parsed_rpt.PITCH_VEL_phys;
  new_msg->roll_vel = parsed_rpt.ROLL_VEL_phys;
  new_msg->yaw_vel = parsed_rpt.YAW_VEL_phys;

  return new_msg;
}

std::shared_ptr<void> Dbc4Api::ParseComponentRpt(const cn_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::ComponentRpt> new_msg( new pm_msgs::ComponentRpt() );

  COMPONENT_RPT_00_t parsed_rpt;
  Unpack_COMPONENT_RPT_00_pacmod4(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->component_type = parsed_rpt.COMPONENT_TYPE;
  new_msg->accel = parsed_rpt.ACCEL;
  new_msg->brake = parsed_rpt.BRAKE;
  new_msg->cruise_control_buttons = parsed_rpt.CRUISE_CONTROL_BUTTONS;
  new_msg->dash_controls_left = parsed_rpt.DASH_CONTROLS_LEFT;
  new_msg->dash_controls_right = parsed_rpt.DASH_CONTROLS_RIGHT;
  new_msg->hazard_lights = parsed_rpt.HAZARD_LIGHTS;
  new_msg->headlight = parsed_rpt.HEADLIGHT;
  new_msg->horn = parsed_rpt.HORN;
  new_msg->media_controls = parsed_rpt.MEDIA_CONTROLS;
  new_msg->parking_brake = parsed_rpt.PARKING_BRAKE;
  new_msg->shift = parsed_rpt.SHIFT;
  new_msg->steering = parsed_rpt.STEERING;
  new_msg->turn = parsed_rpt.TURN;
  new_msg->wiper = parsed_rpt.WIPER;
  new_msg->watchdog = parsed_rpt.WATCHDOG;

  // Following fields not present in dbc4
  new_msg->brake_deccel = 0;
  new_msg->cabin_climate = 0;
  new_msg->cabin_fan_speed = 0;
  new_msg->cabin_temp = 0;
  new_msg->engine_brake = 0;
  new_msg->marker_lamp = 0;
  new_msg->rear_pass_door = 0;
  new_msg->sprayer = 0;

  new_msg->counter = parsed_rpt.COUNTER;
  new_msg->complement = parsed_rpt.COMPLEMENT;
  new_msg->config_fault = parsed_rpt.CONFIG_FAULT;
  new_msg->can_timeout_fault = parsed_rpt.CAN_TIMEOUT_FAULT;

  // Following fields not present in dbc4
  new_msg->internal_supply_voltage_fault = 0;
  new_msg->supervisory_timeout = 0;
  new_msg->supervisory_sanity_fault = 0;
  new_msg->watchdog_sanity_fault = 0;
  new_msg->watchdog_system_present_fault = 0;

  return new_msg;
}

std::shared_ptr<void> Dbc4Api::ParseLinearAccelRpt(const cn_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::LinearAccelRpt> new_msg( new pm_msgs::LinearAccelRpt() );

  LINEAR_ACCEL_RPT_t parsed_rpt;
  Unpack_LINEAR_ACCEL_RPT_pacmod4(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->lateral_new_data_rx = parsed_rpt.LATERAL_NEW_DATA_RX;
  new_msg->longitudinal_new_data_rx = parsed_rpt.LONGITUDNAL_NEW_DATA_RX;
  new_msg->vertical_new_data_rx = parsed_rpt.VERTICAL_NEW_DATA_RX;

  new_msg->lateral_valid = parsed_rpt.LATERAL_VALID;
  new_msg->longitudinal_valid = parsed_rpt.LONGITUDNAL_VALID;
  new_msg->vertical_valid = parsed_rpt.VERTICAL_VALID;

  new_msg->lateral_accel = parsed_rpt.LATERAL_ACCEL_phys;
  new_msg->longitudinal_accel = parsed_rpt.LONGITUDNAL_ACCEL_phys;
  new_msg->vertical_accel = parsed_rpt.VERTICAL_ACCEL_phys;

  return new_msg;
}

std::shared_ptr<void> Dbc4Api::ParseSystemRptBool(const cn_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::SystemRptBool> new_msg( new pm_msgs::SystemRptBool() );

  HORN_RPT_t parsed_rpt;
  Unpack_HORN_RPT_pacmod4(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->enabled = parsed_rpt.ENABLED;
  new_msg->override_active = parsed_rpt.OVERRIDE_ACTIVE;
  new_msg->command_output_fault = parsed_rpt.COMMAND_OUTPUT_FAULT;
  new_msg->input_output_fault = parsed_rpt.INPUT_OUTPUT_FAULT;
  new_msg->output_reported_fault = parsed_rpt.OUTPUT_REPORTED_FAULT;
  new_msg->pacmod_fault = parsed_rpt.PACMOD_FAULT;
  new_msg->vehicle_fault = parsed_rpt.VEHICLE_FAULT;
  new_msg->command_timeout = parsed_rpt.COMMAND_TIMEOUT;
  new_msg->manual_input = parsed_rpt.MANUAL_INPUT;
  new_msg->command = parsed_rpt.COMMANDED_VALUE;
  new_msg->output = parsed_rpt.OUTPUT_VALUE;

  return new_msg;
}

std::shared_ptr<void> Dbc4Api::ParseSystemRptFloat(const cn_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::SystemRptFloat> new_msg( new pm_msgs::SystemRptFloat() );

  ACCEL_RPT_t parsed_rpt;
  Unpack_ACCEL_RPT_pacmod4(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->enabled = parsed_rpt.ENABLED;
  new_msg->override_active = parsed_rpt.OVERRIDE_ACTIVE;
  new_msg->command_output_fault = parsed_rpt.COMMAND_OUTPUT_FAULT;
  new_msg->input_output_fault = parsed_rpt.INPUT_OUTPUT_FAULT;
  new_msg->output_reported_fault = parsed_rpt.OUTPUT_REPORTED_FAULT;
  new_msg->pacmod_fault = parsed_rpt.PACMOD_FAULT;
  new_msg->vehicle_fault = parsed_rpt.VEHICLE_FAULT;
  new_msg->command_timeout = parsed_rpt.COMMAND_TIMEOUT;
  new_msg->manual_input = parsed_rpt.MANUAL_INPUT_phys;
  new_msg->command = parsed_rpt.COMMANDED_VALUE_phys;
  new_msg->output = parsed_rpt.OUTPUT_VALUE_phys;

  return new_msg;
}

std::shared_ptr<void> Dbc4Api::ParseSystemRptInt(const cn_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::SystemRptInt> new_msg( new pm_msgs::SystemRptInt() );

  SHIFT_RPT_t parsed_rpt;
  Unpack_SHIFT_RPT_pacmod4(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->enabled = parsed_rpt.ENABLED;
  new_msg->override_active = parsed_rpt.OVERRIDE_ACTIVE;
  new_msg->command_output_fault = parsed_rpt.COMMAND_OUTPUT_FAULT;
  new_msg->input_output_fault = parsed_rpt.INPUT_OUTPUT_FAULT;
  new_msg->output_reported_fault = parsed_rpt.OUTPUT_REPORTED_FAULT;
  new_msg->pacmod_fault = parsed_rpt.PACMOD_FAULT;
  new_msg->vehicle_fault = parsed_rpt.VEHICLE_FAULT;
  new_msg->command_timeout = parsed_rpt.COMMAND_TIMEOUT;
  new_msg->manual_input = parsed_rpt.MANUAL_INPUT;
  new_msg->command = parsed_rpt.COMMANDED_VALUE;
  new_msg->output = parsed_rpt.OUTPUT_VALUE;

  return new_msg;
}

// Message Encoding

cn_msgs::Frame Dbc4Api::EncodeCmd(const pm_msgs::GlobalCmd& msg)
{
  cn_msgs::Frame packed_frame;

  GLOBAL_CMD_t unpacked_cmd;
  unpacked_cmd.CLEAR_FAULTS = msg.clear_faults;

  uint8_t unused_ide;
  Pack_GLOBAL_CMD_pacmod4(&unpacked_cmd, static_cast<uint8_t*>(&packed_frame.data[0]), static_cast<uint8_t*>(&packed_frame.dlc), &unused_ide);

  return packed_frame;
}

cn_msgs::Frame Dbc4Api::EncodeCmd(const pm_msgs::NotificationCmd& msg)
{
  cn_msgs::Frame packed_frame;

  NOTIFICATION_CMD_t unpacked_cmd;
  unpacked_cmd.BUZZER_MUTE = msg.buzzer_mute;
  unpacked_cmd.UNDERDASH_LIGHTS_WHITE = msg.underdash_lights_white;

  uint8_t unused_ide;
  Pack_NOTIFICATION_CMD_pacmod4(&unpacked_cmd, static_cast<uint8_t*>(&packed_frame.data[0]), static_cast<uint8_t*>(&packed_frame.dlc), &unused_ide);

  return packed_frame;
}

cn_msgs::Frame Dbc4Api::EncodeCmd(const pm_msgs::SystemCmdBool& msg)
{
  cn_msgs::Frame packed_frame;

  HORN_CMD_t unpacked_cmd;
  unpacked_cmd.ENABLE = msg.enable;
  unpacked_cmd.IGNORE_OVERRIDES = msg.ignore_overrides;
  unpacked_cmd.CLEAR_OVERRIDE = msg.clear_override;
  unpacked_cmd.HORN_CMD = msg.command;

  uint8_t unused_ide;
  Pack_HORN_CMD_pacmod4(&unpacked_cmd, static_cast<uint8_t*>(&packed_frame.data[0]), static_cast<uint8_t*>(&packed_frame.dlc), &unused_ide);

  return packed_frame;
}

cn_msgs::Frame Dbc4Api::EncodeCmd(const pm_msgs::SystemCmdInt& msg)
{
  cn_msgs::Frame packed_frame;

  SHIFT_CMD_t unpacked_cmd;
  unpacked_cmd.ENABLE = msg.enable;
  unpacked_cmd.IGNORE_OVERRIDES = msg.ignore_overrides;
  unpacked_cmd.CLEAR_OVERRIDE = msg.clear_override;
  unpacked_cmd.SHIFT_CMD = msg.command;

  uint8_t unused_ide;
  Pack_SHIFT_CMD_pacmod4(&unpacked_cmd, static_cast<uint8_t*>(&packed_frame.data[0]), static_cast<uint8_t*>(&packed_frame.dlc), &unused_ide);

  return packed_frame;
}

cn_msgs::Frame Dbc4Api::EncodeCmd(const pm_msgs::SystemCmdFloat& msg)
{
  cn_msgs::Frame packed_frame;

  ACCEL_CMD_t unpacked_cmd;
  unpacked_cmd.ENABLE = msg.enable;
  unpacked_cmd.IGNORE_OVERRIDES = msg.ignore_overrides;
  unpacked_cmd.CLEAR_OVERRIDE = msg.clear_override;
  unpacked_cmd.ACCEL_CMD_phys = msg.command;

  uint8_t unused_ide;
  Pack_ACCEL_CMD_pacmod4(&unpacked_cmd, static_cast<uint8_t*>(&packed_frame.data[0]), static_cast<uint8_t*>(&packed_frame.dlc), &unused_ide);

  return packed_frame;
}

}  // namespace pacmod3
