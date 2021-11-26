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

std::shared_ptr<void> Dbc4Api::ParseSystemRptBool(const can_msgs::Frame& can_msg)
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

std::shared_ptr<void> Dbc4Api::ParseSystemRptInt(const can_msgs::Frame& can_msg)
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

std::shared_ptr<void> Dbc4Api::ParseSystemRptFloat(const can_msgs::Frame& can_msg)
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

can_msgs::Frame Dbc4Api::EncodeSystemCmdBool(const pm_msgs::SystemCmdBool& msg)
{
  can_msgs::Frame packed_frame;

  HORN_CMD_t unpacked_cmd;
  unpacked_cmd.ENABLE = msg.enable;
  unpacked_cmd.IGNORE_OVERRIDES = msg.ignore_overrides;
  unpacked_cmd.CLEAR_OVERRIDE = msg.clear_override;
  unpacked_cmd.HORN_CMD = msg.command;

  uint8_t unused_ide;
  Pack_HORN_CMD_pacmod4(&unpacked_cmd, static_cast<uint8_t*>(&packed_frame.data[0]), static_cast<uint8_t*>(&packed_frame.dlc), &unused_ide);

  return packed_frame;
}

can_msgs::Frame Dbc4Api::EncodeSystemCmdInt(const pm_msgs::SystemCmdInt& msg)
{
  can_msgs::Frame packed_frame;

  SHIFT_CMD_t unpacked_cmd;
  unpacked_cmd.ENABLE = msg.enable;
  unpacked_cmd.IGNORE_OVERRIDES = msg.ignore_overrides;
  unpacked_cmd.CLEAR_OVERRIDE = msg.clear_override;
  unpacked_cmd.SHIFT_CMD = msg.command;

  uint8_t unused_ide;
  Pack_SHIFT_CMD_pacmod4(&unpacked_cmd, static_cast<uint8_t*>(&packed_frame.data[0]), static_cast<uint8_t*>(&packed_frame.dlc), &unused_ide);

  return packed_frame;
}

can_msgs::Frame Dbc4Api::EncodeSystemCmdFloat(const pm_msgs::SystemCmdFloat& msg)
{
  can_msgs::Frame packed_frame;

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

