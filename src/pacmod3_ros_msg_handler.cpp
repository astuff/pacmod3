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

namespace AS
{
namespace Drivers
{
namespace PACMod3
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

void Pacmod3TxRosMsgHandler::fillAndPublish(
    const uint32_t& can_id,
    const std::string& frame_id,
    const ros::Publisher& pub,
    const std::shared_ptr<Pacmod3TxMsg>& parser_class)
{
  if (can_id == HornRptMsg::CAN_ID ||
      can_id == ParkingBrakeRptMsg::CAN_ID ||
      can_id == MarkerLampRptMsg::CAN_ID ||
      can_id == SprayerRptMsg::CAN_ID ||
      can_id == HazardLightRptMsg::CAN_ID)
  {
    pacmod_msgs::SystemRptBool new_msg;
    fillSystemRptBool(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == CruiseControlButtonsRptMsg::CAN_ID ||
           can_id == DashControlsLeftRptMsg::CAN_ID ||
           can_id == DashControlsRightRptMsg::CAN_ID ||
           can_id == TurnSignalRptMsg::CAN_ID ||
           can_id == RearPassDoorRptMsg::CAN_ID ||
           can_id == ShiftRptMsg::CAN_ID ||
           can_id == HeadlightRptMsg::CAN_ID ||
           can_id == MediaControlsRptMsg::CAN_ID ||
           can_id == WiperRptMsg::CAN_ID ||
           can_id == EngineBrakeRptMsg::CAN_ID)
  {
    pacmod_msgs::SystemRptInt new_msg;
    fillSystemRptInt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == AccelRptMsg::CAN_ID ||
           can_id == BrakeRptMsg::CAN_ID ||
           can_id == SteerRptMsg::CAN_ID)
  {
    pacmod_msgs::SystemRptFloat new_msg;
    fillSystemRptFloat(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == GlobalRptMsg::CAN_ID)
  {
    pacmod_msgs::GlobalRpt new_msg;
    fillGlobalRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == GlobalRpt2Msg::CAN_ID)
  {
    pacmod_msgs::GlobalRpt2 new_msg;
    fillGlobalRpt2(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == ComponentRptMsg00::CAN_ID ||
           can_id == ComponentRptMsg01::CAN_ID ||
           can_id == ComponentRptMsg02::CAN_ID ||
           can_id == ComponentRptMsg03::CAN_ID)
  {
    pacmod_msgs::ComponentRpt new_msg;
    fillComponentRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == BrakeMotorRpt1Msg::CAN_ID ||
           can_id == SteerMotorRpt1Msg::CAN_ID)
  {
    pacmod_msgs::MotorRpt1 new_msg;
    fillMotorRpt1(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == BrakeMotorRpt2Msg::CAN_ID ||
           can_id == SteerMotorRpt2Msg::CAN_ID)
  {
    pacmod_msgs::MotorRpt2 new_msg;
    fillMotorRpt2(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == BrakeMotorRpt3Msg::CAN_ID ||
           can_id == SteerMotorRpt3Msg::CAN_ID)
  {
    pacmod_msgs::MotorRpt3 new_msg;
    fillMotorRpt3(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == AccelAuxRptMsg::CAN_ID)
  {
    pacmod_msgs::AccelAuxRpt new_msg;
    fillAccelAuxRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == BrakeAuxRptMsg::CAN_ID)
  {
    pacmod_msgs::BrakeAuxRpt new_msg;
    fillBrakeAuxRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == AngVelRptMsg::CAN_ID)
  {
    pacmod_msgs::AngVelRpt new_msg;
    fillAngVelRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == DateTimeRptMsg::CAN_ID)
  {
    pacmod_msgs::DateTimeRpt new_msg;
    fillDateTimeRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == DoorRptMsg::CAN_ID)
  {
    pacmod_msgs::DoorRpt new_msg;
    fillDoorRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == EngineRptMsg::CAN_ID)
  {
    pacmod_msgs::EngineRpt new_msg;
    fillEngineRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == HeadlightAuxRptMsg::CAN_ID)
  {
    pacmod_msgs::HeadlightAuxRpt new_msg;
    fillHeadlightAuxRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == InteriorLightsRptMsg::CAN_ID)
  {
    pacmod_msgs::InteriorLightsRpt new_msg;
    fillInteriorLightsRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == LatLonHeadingRptMsg::CAN_ID)
  {
    pacmod_msgs::LatLonHeadingRpt new_msg;
    fillLatLonHeadingRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == OccupancyRptMsg::CAN_ID)
  {
    pacmod_msgs::OccupancyRpt new_msg;
    fillOccupancyRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == RearLightsRptMsg::CAN_ID)
  {
    pacmod_msgs::RearLightsRpt new_msg;
    fillRearLightsRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == ShiftAuxRptMsg::CAN_ID)
  {
    pacmod_msgs::ShiftAuxRpt new_msg;
    fillShiftAuxRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == SteerAuxRptMsg::CAN_ID)
  {
    pacmod_msgs::SteerAuxRpt new_msg;
    fillSteerAuxRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == TurnAuxRptMsg::CAN_ID)
  {
    pacmod_msgs::TurnAuxRpt new_msg;
    fillTurnAuxRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == YawRateRptMsg::CAN_ID)
  {
    pacmod_msgs::YawRateRpt new_msg;
    fillYawRateRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == VehicleSpeedRptMsg::CAN_ID)
  {
    pacmod_msgs::VehicleSpeedRpt new_msg;
    fillVehicleSpeedRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == VinRptMsg::CAN_ID)
  {
    pacmod_msgs::VinRpt new_msg;
    fillVinRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == WheelSpeedRptMsg::CAN_ID)
  {
    pacmod_msgs::WheelSpeedRpt new_msg;
    fillWheelSpeedRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == WiperAuxRptMsg::CAN_ID)
  {
    pacmod_msgs::WiperAuxRpt new_msg;
    fillWiperAuxRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == DetectedObjectRptMsg::CAN_ID)
  {
    pacmod_msgs::DetectedObjectRpt new_msg;
    fillDetectedObjectRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
}

// General Reports
void Pacmod3TxRosMsgHandler::fillSystemRptBool(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::SystemRptBool * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<SystemRptBoolMsg>(parser_class);

  new_msg->enabled = dc_parser->enabled;
  new_msg->override_active = dc_parser->override_active;
  new_msg->command_output_fault = dc_parser->command_output_fault;
  new_msg->input_output_fault = dc_parser->input_output_fault;
  new_msg->output_reported_fault = dc_parser->output_reported_fault;
  new_msg->pacmod_fault = dc_parser->pacmod_fault;
  new_msg->vehicle_fault = dc_parser->vehicle_fault;
  new_msg->command_timeout = dc_parser->command_timeout;
  new_msg->manual_input = dc_parser->manual_input;
  new_msg->command = dc_parser->command;
  new_msg->output = dc_parser->output;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillSystemRptInt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::SystemRptInt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<SystemRptIntMsg>(parser_class);

  new_msg->enabled = dc_parser->enabled;
  new_msg->override_active = dc_parser->override_active;
  new_msg->command_output_fault = dc_parser->command_output_fault;
  new_msg->input_output_fault = dc_parser->input_output_fault;
  new_msg->output_reported_fault = dc_parser->output_reported_fault;
  new_msg->pacmod_fault = dc_parser->pacmod_fault;
  new_msg->vehicle_fault = dc_parser->vehicle_fault;
  new_msg->command_timeout = dc_parser->command_timeout;

  new_msg->manual_input = dc_parser->manual_input;
  new_msg->command = dc_parser->command;
  new_msg->output = dc_parser->output;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillSystemRptFloat(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::SystemRptFloat * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<SystemRptFloatMsg>(parser_class);

  new_msg->enabled = dc_parser->enabled;
  new_msg->override_active = dc_parser->override_active;
  new_msg->command_output_fault = dc_parser->command_output_fault;
  new_msg->input_output_fault = dc_parser->input_output_fault;
  new_msg->output_reported_fault = dc_parser->output_reported_fault;
  new_msg->pacmod_fault = dc_parser->pacmod_fault;
  new_msg->vehicle_fault = dc_parser->vehicle_fault;
  new_msg->command_timeout = dc_parser->command_timeout;

  new_msg->manual_input = dc_parser->manual_input;
  new_msg->command = dc_parser->command;
  new_msg->output = dc_parser->output;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillComponentRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::ComponentRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<ComponentRptMsg>(parser_class);

  new_msg->component_type = dc_parser->component_type;

  new_msg->accel = dc_parser->accel;
  new_msg->brake = dc_parser->brake;
  new_msg->cruise_control_buttons = dc_parser->cruise_control_buttons;
  new_msg->dash_controls_left = dc_parser->dash_controls_left;
  new_msg->dash_controls_right = dc_parser->dash_controls_right;
  new_msg->hazard_lights = dc_parser->hazard_lights;
  new_msg->headlight = dc_parser->headlight;
  new_msg->horn = dc_parser->horn;
  new_msg->media_controls = dc_parser->media_controls;
  new_msg->parking_brake = dc_parser->parking_brake;
  new_msg->shift = dc_parser->shift;
  new_msg->sprayer = dc_parser->sprayer;
  new_msg->steering = dc_parser->steering;
  new_msg->turn = dc_parser->turn;
  new_msg->wiper = dc_parser->wiper;
  new_msg->watchdog = dc_parser->watchdog;
  new_msg->brake_deccel = dc_parser->brake_deccel;
  new_msg->rear_pass_door = dc_parser->rear_pass_door;
  new_msg->engine_brake = dc_parser->engine_brake;
  new_msg->marker_lamp = dc_parser->marker_lamp;
  new_msg->cabin_climate = dc_parser->cabin_climate;
  new_msg->cabin_fan_speed = dc_parser->cabin_fan_speed;
  new_msg->cabin_temp = dc_parser->cabin_temp;

  new_msg->counter = dc_parser->counter;
  new_msg->complement = dc_parser->complement;
  new_msg->config_fault = dc_parser->config_fault;
  new_msg->can_timeout_fault = dc_parser->can_timeout_fault;
  new_msg->internal_supply_voltage_fault = dc_parser->internal_supply_voltage_fault;
  new_msg->supervisory_timeout = dc_parser->supervisory_timeout;
  new_msg->supervisory_sanity_fault = dc_parser->supervisory_sanity_fault;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillMotorRpt1(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::MotorRpt1 * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<MotorRpt1Msg>(parser_class);

  new_msg->current = dc_parser->current;
  new_msg->position = dc_parser->position;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillMotorRpt2(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::MotorRpt2 * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<MotorRpt2Msg>(parser_class);

  new_msg->encoder_temp = dc_parser->encoder_temp;
  new_msg->motor_temp = dc_parser->motor_temp;
  new_msg->angular_speed = dc_parser->angular_speed;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillMotorRpt3(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::MotorRpt3 * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<MotorRpt3Msg>(parser_class);

  new_msg->torque_output = dc_parser->torque_output;
  new_msg->torque_input = dc_parser->torque_input;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillCabinClimateRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::CabinClimateRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<CabinClimateRptMsg>(parser_class);

  new_msg->man_ac_off_on = dc_parser->man_ac_off_on;
  new_msg->man_max_ac_off_on = dc_parser->man_max_ac_off_on;
  new_msg->man_defrost_off_on = dc_parser->man_defrost_off_on;
  new_msg->man_max_defrost_off_on = dc_parser->man_max_defrost_off_on;
  new_msg->man_dir_up_off_on = dc_parser->man_dir_up_off_on;
  new_msg->man_dir_down_off_on = dc_parser->man_dir_down_off_on;

  new_msg->cmd_ac_off_on = dc_parser->man_ac_off_on;
  new_msg->cmd_max_ac_off_on = dc_parser->man_max_ac_off_on;
  new_msg->cmd_defrost_off_on = dc_parser->man_defrost_off_on;
  new_msg->cmd_max_defrost_off_on = dc_parser->man_max_defrost_off_on;
  new_msg->cmd_dir_up_off_on = dc_parser->man_dir_up_off_on;
  new_msg->cmd_dir_down_off_on = dc_parser->man_dir_down_off_on;

  new_msg->out_ac_off_on = dc_parser->man_ac_off_on;
  new_msg->out_max_ac_off_on = dc_parser->man_max_ac_off_on;
  new_msg->out_defrost_off_on = dc_parser->man_defrost_off_on;
  new_msg->out_max_defrost_off_on = dc_parser->man_max_defrost_off_on;
  new_msg->out_dir_up_off_on = dc_parser->man_dir_up_off_on;
  new_msg->out_dir_down_off_on = dc_parser->man_dir_down_off_on;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();

}

void Pacmod3TxRosMsgHandler::fillSafetyBrakeRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::SafetyBrakeRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<SafetyBrakeRptMsg>(parser_class);

  new_msg->commanded_val = dc_parser->commanded_val;
  new_msg->output_val = dc_parser->output_val;
  new_msg->reported_fault = dc_parser->reported_fault;
  new_msg->cmd_reported_fault = dc_parser->cmd_reported_fault;
  new_msg->cmd_timeout = dc_parser->cmd_timeout;
  new_msg->cmd_permitted = dc_parser->cmd_permitted;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();

}

void Pacmod3TxRosMsgHandler::fillSafetyFuncRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::SafetyFuncRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<SafetyFuncRptMsg>(parser_class);

  // new_msg->torque_output = dc_parser->torque_output;
  // new_msg->torque_input = dc_parser->torque_input;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();

}


// Global
void Pacmod3TxRosMsgHandler::fillGlobalRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::GlobalRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<GlobalRptMsg>(parser_class);

  new_msg->enabled = dc_parser->enabled;
  new_msg->override_active = dc_parser->override_active;
  new_msg->user_can_timeout = dc_parser->user_can_timeout;
  new_msg->steering_can_timeout = dc_parser->steering_can_timeout;
  new_msg->brake_can_timeout = dc_parser->brake_can_timeout;
  new_msg->subsystem_can_timeout = dc_parser->subsystem_can_timeout;
  new_msg->vehicle_can_timeout = dc_parser->vehicle_can_timeout;
  new_msg->pacmod_sys_fault_active = dc_parser->pacmod_sys_fault_active;
  new_msg->supervisory_enable_required = dc_parser->supervisory_enable_required;  
  new_msg->config_fault_active = dc_parser->config_fault_active;
  new_msg->user_can_read_errors = dc_parser->user_can_read_errors;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillGlobalRpt2(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::GlobalRpt2 * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<GlobalRpt2Msg>(parser_class);

  new_msg->system_enabled = dc_parser->system_enabled;
  new_msg->system_override_active = dc_parser->system_override_active;
  new_msg->system_fault_active = dc_parser->system_fault_active;
  new_msg->supervisory_enable_required = dc_parser->supervisory_enable_required;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

// System Aux Reports
void Pacmod3TxRosMsgHandler::fillAccelAuxRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::AccelAuxRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<AccelAuxRptMsg>(parser_class);

  new_msg->operator_interaction = dc_parser->operator_interaction;
  new_msg->operator_interaction_avail = dc_parser->operator_interaction_avail;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillBrakeAuxRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::BrakeAuxRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<BrakeAuxRptMsg>(parser_class);

  new_msg->brake_pressure = dc_parser->brake_pressure;
  new_msg->operator_interaction = dc_parser->operator_interaction;
  new_msg->brake_on_off = dc_parser->brake_on_off;
  
  new_msg->brake_pressure_avail = dc_parser->brake_pressure_avail;
  new_msg->operator_interaction_avail = dc_parser->operator_interaction_avail;
  new_msg->brake_on_off_avail = dc_parser->brake_on_off_avail;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillBrakeDeccelAuxRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::BrakeDeccelAuxRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<BrakeDeccelAuxRptMsg>(parser_class);

  // new_msg->brake_pressure = dc_parser->brake_pressure;
  // new_msg->operator_interaction = dc_parser->operator_interaction;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();

}

void Pacmod3TxRosMsgHandler::fillHeadlightAuxRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::HeadlightAuxRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<HeadlightAuxRptMsg>(parser_class);

  new_msg->headlights_on = dc_parser->headlights_on;
  new_msg->headlights_on_bright = dc_parser->headlights_on_bright;
  new_msg->fog_lights_on = dc_parser->fog_lights_on;
  new_msg->headlights_mode = dc_parser->headlights_mode;
  
  new_msg->headlights_on_avail = dc_parser->headlights_on_avail;
  new_msg->headlights_on_bright_avail = dc_parser->headlights_on_bright_avail;
  new_msg->fog_lights_on_avail = dc_parser->fog_lights_on_avail;
  new_msg->headlights_mode_avail = dc_parser->headlights_mode_avail;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillShiftAuxRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::ShiftAuxRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<ShiftAuxRptMsg>(parser_class);

  new_msg->between_gears = dc_parser->between_gears;
  new_msg->stay_in_neutral_mode = dc_parser->stay_in_neutral_mode;
  new_msg->brake_interlock_active = dc_parser->brake_interlock_active;
  new_msg->speed_interlock_active = dc_parser->speed_interlock_active;
  new_msg->write_to_config = dc_parser->write_to_config;
  
  new_msg->between_gears_avail = dc_parser->between_gears_avail;
  new_msg->stay_in_neutral_mode_avail = dc_parser->stay_in_neutral_mode_avail;
  new_msg->brake_interlock_active_avail = dc_parser->brake_interlock_active_avail;
  new_msg->speed_interlock_active_avail = dc_parser->speed_interlock_active_avail;
  new_msg->write_to_config_is_valid = dc_parser->write_to_config_is_valid;
  new_msg->gear_number_avail = dc_parser->gear_number_avail;
  new_msg->gear_number = dc_parser->gear_number;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillSteerAuxRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::SteerAuxRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<SteerAuxRptMsg>(parser_class);

  new_msg->steering_torque = dc_parser->steering_torque;
  new_msg->rotation_rate = dc_parser->rotation_rate;
  new_msg->operator_interaction = dc_parser->operator_interaction;
  new_msg->rotation_rate_sign = dc_parser->rotation_rate_sign;
  new_msg->vehicle_angle_calib_status = dc_parser->vehicle_angle_calib_status;

  new_msg->steering_torque_avail = dc_parser->steering_torque_avail;
  new_msg->rotation_rate_avail = dc_parser->rotation_rate_avail;
  new_msg->operator_interaction_avail = dc_parser->operator_interaction_avail;
  new_msg->rotation_rate_sign_avail = dc_parser->rotation_rate_sign_avail;
  new_msg->vehicle_angle_calib_status_avail = dc_parser->vehicle_angle_calib_status_avail;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillTurnAuxRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::TurnAuxRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<TurnAuxRptMsg>(parser_class);

  new_msg->driver_blinker_bulb_on = dc_parser->driver_blinker_bulb_on;
  new_msg->passenger_blinker_bulb_on = dc_parser->passenger_blinker_bulb_on;
  
  new_msg->driver_blinker_bulb_on_avail = dc_parser->driver_blinker_bulb_on_avail;
  new_msg->passenger_blinker_bulb_on_avail = dc_parser->passenger_blinker_bulb_on_avail;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillWiperAuxRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::WiperAuxRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<WiperAuxRptMsg>(parser_class);

  new_msg->front_wiping = dc_parser->front_wiping;
  new_msg->front_spraying = dc_parser->front_spraying;
  new_msg->rear_wiping = dc_parser->rear_wiping;
  new_msg->rear_spraying = dc_parser->rear_spraying;
  new_msg->spray_near_empty = dc_parser->spray_near_empty;
  new_msg->spray_empty = dc_parser->spray_empty;
  
  new_msg->front_wiping_avail = dc_parser->front_wiping_avail;
  new_msg->front_spraying_avail = dc_parser->front_spraying_avail;
  new_msg->rear_wiping_avail = dc_parser->rear_wiping_avail;
  new_msg->rear_spraying_avail = dc_parser->rear_spraying_avail;
  new_msg->spray_near_empty_avail = dc_parser->spray_near_empty_avail;
  new_msg->spray_empty_avail = dc_parser->spray_empty_avail;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

// Misc Reports
void Pacmod3TxRosMsgHandler::fillAngVelRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::AngVelRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<AngVelRptMsg>(parser_class);

  new_msg->pitch_new_data_rx = dc_parser->pitch_new_data_rx;
  new_msg->roll_new_data_rx = dc_parser->roll_new_data_rx;
  new_msg->yaw_new_data_rx = dc_parser->yaw_new_data_rx;
  new_msg->pitch_valid = dc_parser->pitch_valid;
  new_msg->roll_valid = dc_parser->roll_valid;
  new_msg->yaw_valid = dc_parser->yaw_valid;
  new_msg->pitch_vel = dc_parser->pitch_vel;
  new_msg->roll_vel = dc_parser->roll_vel;
  new_msg->yaw_vel = dc_parser->yaw_vel;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillDateTimeRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::DateTimeRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<DateTimeRptMsg>(parser_class);

  new_msg->year = dc_parser->year;
  new_msg->month = dc_parser->month;
  new_msg->day = dc_parser->day;
  new_msg->hour = dc_parser->hour;
  new_msg->minute = dc_parser->minute;
  new_msg->second = dc_parser->second;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillDetectedObjectRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::DetectedObjectRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<DetectedObjectRptMsg>(parser_class);

  new_msg->front_object_distance_low_res = dc_parser->front_object_distance_low_res;
  new_msg->front_object_distance_high_res = dc_parser->front_object_distance_high_res;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillDoorRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::DoorRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<DoorRptMsg>(parser_class);

  new_msg->driver_door_open = dc_parser->driver_door_open;
  new_msg->passenger_door_open = dc_parser->passenger_door_open;
  new_msg->rear_driver_door_open = dc_parser->rear_driver_door_open;
  new_msg->rear_passenger_door_open = dc_parser->rear_passenger_door_open;
  new_msg->hood_open = dc_parser->hood_open;
  new_msg->trunk_open = dc_parser->trunk_open;
  new_msg->fuel_door_open = dc_parser->fuel_door_open;
  
  new_msg->driver_door_open_avail = dc_parser->driver_door_open_avail;
  new_msg->passenger_door_open_avail = dc_parser->passenger_door_open_avail;
  new_msg->rear_driver_door_open_avail = dc_parser->rear_driver_door_open_avail;
  new_msg->rear_passenger_door_open_avail = dc_parser->rear_passenger_door_open_avail;
  new_msg->hood_open_avail = dc_parser->hood_open_avail;
  new_msg->trunk_open_avail = dc_parser->trunk_open_avail;
  new_msg->fuel_door_open_avail = dc_parser->fuel_door_open_avail;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void fillDriveTrainRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::DriveTrainRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<DriveTrainRptMsg>(parser_class);

  // new_msg->driver_door_open = dc_parser->driver_door_open;
  // new_msg->passenger_door_open = dc_parser->passenger_door_open;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillEngineRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::EngineRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<EngineRptMsg>(parser_class);

  new_msg->engine_speed = dc_parser->engine_speed;
  new_msg->engine_torque = dc_parser->engine_torque;
  new_msg->engine_coolant_temp = dc_parser->engine_coolant_temp;
  
  new_msg->engine_speed_avail = dc_parser->engine_speed_avail;
  new_msg->engine_torque_avail = dc_parser->engine_torque_avail;
  new_msg->engine_coolant_temp_avail = dc_parser->engine_coolant_temp_avail;
  new_msg->fuel_level_avail = dc_parser->fuel_level_avail;
  new_msg->fuel_level = dc_parser->fuel_level;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillInteriorLightsRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::InteriorLightsRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<InteriorLightsRptMsg>(parser_class);

  new_msg->front_dome_lights_on = dc_parser->front_dome_lights_on;
  new_msg->rear_dome_lights_on = dc_parser->rear_dome_lights_on;
  new_msg->mood_lights_on = dc_parser->mood_lights_on;
  new_msg->ambient_light_sensor = dc_parser->ambient_light_sensor;
  new_msg->dim_level = dc_parser->dim_level;
  
  new_msg->front_dome_lights_on_avail = dc_parser->front_dome_lights_on_avail;
  new_msg->rear_dome_lights_on_avail = dc_parser->rear_dome_lights_on_avail;
  new_msg->mood_lights_on_avail = dc_parser->mood_lights_on_avail;
  new_msg->dim_level_avail = dc_parser->dim_level_avail;
  new_msg->ambient_light_sensor_avail = dc_parser->ambient_light_sensor_avail;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillLatLonHeadingRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::LatLonHeadingRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<LatLonHeadingRptMsg>(parser_class);

  new_msg->latitude_degrees = dc_parser->latitude_degrees;
  new_msg->latitude_minutes = dc_parser->latitude_minutes;
  new_msg->latitude_seconds = dc_parser->latitude_seconds;
  new_msg->longitude_degrees = dc_parser->longitude_degrees;
  new_msg->longitude_minutes = dc_parser->longitude_minutes;
  new_msg->longitude_seconds = dc_parser->longitude_seconds;
  new_msg->heading = dc_parser->heading;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void fillLinearAccelRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::LinearAccelRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<LinearAccelRptMsg>(parser_class);

  // new_msg->driver_door_open = dc_parser->driver_door_open;
  // new_msg->passenger_door_open = dc_parser->passenger_door_open;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillOccupancyRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::OccupancyRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<OccupancyRptMsg>(parser_class);

  new_msg->driver_seat_occupied = dc_parser->driver_seat_occupied;
  new_msg->passenger_seat_occupied = dc_parser->passenger_seat_occupied;
  new_msg->rear_seat_occupied = dc_parser->rear_seat_occupied;
  new_msg->driver_seatbelt_buckled = dc_parser->driver_seatbelt_buckled;
  new_msg->passenger_seatbelt_buckled = dc_parser->passenger_seatbelt_buckled;
  new_msg->driver_rear_seatbelt_buckled = dc_parser->driver_rear_seatbelt_buckled;
  new_msg->pass_rear_seatbelt_buckled = dc_parser->pass_rear_seatbelt_buckled;
  new_msg->center_rear_seatbelt_buckled = dc_parser->center_rear_seatbelt_buckled;
  
  new_msg->driver_seat_occupied_avail = dc_parser->driver_seat_occupied_avail;
  new_msg->passenger_seat_occupied_avail = dc_parser->passenger_seat_occupied_avail;
  new_msg->rear_seat_occupied_avail = dc_parser->rear_seat_occupied_avail;
  new_msg->driver_seatbelt_buckled_avail = dc_parser->driver_seatbelt_buckled_avail;
  new_msg->passenger_seatbelt_buckled_avail = dc_parser->passenger_seatbelt_buckled_avail;
  new_msg->driver_rear_seatbelt_buckled_avail = dc_parser->driver_rear_seatbelt_buckled_avail;
  new_msg->pass_rear_seatbelt_buckled_avail = dc_parser->pass_rear_seatbelt_buckled_avail;
  new_msg->center_rear_seatbelt_buckled_avail = dc_parser->center_rear_seatbelt_buckled_avail;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillRearLightsRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::RearLightsRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<RearLightsRptMsg>(parser_class);

  new_msg->brake_lights_on = dc_parser->brake_lights_on;
  new_msg->reverse_lights_on = dc_parser->reverse_lights_on;

  new_msg->brake_lights_on_avail = dc_parser->brake_lights_on_avail;
  new_msg->reverse_lights_on_avail = dc_parser->reverse_lights_on_avail;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillVehicleSpeedRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::VehicleSpeedRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<VehicleSpeedRptMsg>(parser_class);

  new_msg->vehicle_speed = dc_parser->vehicle_speed;
  new_msg->vehicle_speed_valid = dc_parser->vehicle_speed_valid;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillVinRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::VinRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<VinRptMsg>(parser_class);

  new_msg->mfg_code = dc_parser->mfg_code;
  new_msg->mfg = dc_parser->mfg;
  new_msg->model_year_code = dc_parser->model_year_code;
  new_msg->model_year = dc_parser->model_year;
  new_msg->serial = dc_parser->serial;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillWheelSpeedRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::WheelSpeedRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<WheelSpeedRptMsg>(parser_class);

  new_msg->front_left_wheel_speed = dc_parser->front_left_wheel_speed;
  new_msg->front_right_wheel_speed = dc_parser->front_right_wheel_speed;
  new_msg->rear_left_wheel_speed = dc_parser->rear_left_wheel_speed;
  new_msg->rear_right_wheel_speed = dc_parser->rear_right_wheel_speed;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillYawRateRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::YawRateRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<YawRateRptMsg>(parser_class);

  new_msg->yaw_rate = dc_parser->yaw_rate;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void fillSystemCmdLimitRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::SystemCmdLimitRpt * new_msg,
    const std::string& frame_id)
{
    auto dc_parser = std::dynamic_pointer_cast<SystemCmdLimitRptMsg>(parser_class);

  // new_msg->driver_door_open = dc_parser->driver_door_open;
  // new_msg->passenger_door_open = dc_parser->passenger_door_open;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void fillSteerCmdLimitRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::SteerCmdLimitRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<SteerCmdLimitRptMsg>(parser_class);

  // new_msg->driver_door_open = dc_parser->driver_door_open;
  // new_msg->passenger_door_open = dc_parser->passenger_door_open;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

// Command messages
std::vector<uint8_t> Pacmod3RxRosMsgHandler::unpackAndEncode(
    const uint32_t& can_id, const pacmod_msgs::SystemCmdBool::ConstPtr& msg)
{
  if (can_id == HornCmdMsg::CAN_ID)
  {
    HornCmdMsg encoder;
    encoder.encode(msg->enable,
                   msg->ignore_overrides,
                   msg->clear_override,
                   msg->command);
    return encoder.data;
  }
  else if (can_id == ParkingBrakeCmdMsg::CAN_ID)
  {
    ParkingBrakeCmdMsg encoder;
    encoder.encode(msg->enable,
                   msg->ignore_overrides,
                   msg->clear_override,
                   msg->command);
    return encoder.data;
  }
  else if (can_id == MarkerLampCmdMsg::CAN_ID)
  {
    MarkerLampCmdMsg encoder;
    encoder.encode(msg->enable,
                   msg->ignore_overrides,
                   msg->clear_override,
                   msg->command);
    return encoder.data;
  }
  else if (can_id == SprayerCmdMsg::CAN_ID)
  {
    SprayerCmdMsg encoder;
    encoder.encode(msg->enable,
                   msg->ignore_overrides,
                   msg->clear_override,
                   msg->command);
    return encoder.data;
  }
  else if (can_id == HazardLightCmdMsg::CAN_ID)
  {
    HazardLightCmdMsg encoder;
    encoder.encode(msg->enable,
                   msg->ignore_overrides,
                   msg->clear_override,
                   msg->command);
    return encoder.data;
  }
  else
  {
    std::vector<uint8_t> bad_id;
    bad_id.assign(8, 0);
    ROS_ERROR("A bool system command matching the provided CAN ID could not be found.");
    return bad_id;
  }
}

std::vector<uint8_t> Pacmod3RxRosMsgHandler::unpackAndEncode(
    const uint32_t& can_id, const pacmod_msgs::SystemCmdFloat::ConstPtr& msg)
{
  if (can_id == AccelCmdMsg::CAN_ID)
  {
    AccelCmdMsg encoder;
    encoder.encode(msg->enable,
                   msg->ignore_overrides,
                   msg->clear_override,
                   msg->command);
    return encoder.data;
  }
  else if (can_id == BrakeCmdMsg::CAN_ID)
  {
    BrakeCmdMsg encoder;
    encoder.encode(msg->enable,
                   msg->ignore_overrides,
                   msg->clear_override,
                   msg->command);
    return encoder.data;
  }
  else
  {
    std::vector<uint8_t> bad_id;
    bad_id.assign(8, 0);
    ROS_ERROR("A float system command matching the provided CAN ID could not be found.");
    return bad_id;
  }
}

std::vector<uint8_t> Pacmod3RxRosMsgHandler::unpackAndEncode(
    const uint32_t& can_id, const pacmod_msgs::SystemCmdInt::ConstPtr& msg)
{
  if (can_id == CruiseControlButtonsCmdMsg::CAN_ID)
  {
    CruiseControlButtonsCmdMsg encoder;
    encoder.encode(msg->enable,
                   msg->ignore_overrides,
                   msg->clear_override,
                   msg->command);
    return encoder.data;
  }
  else if (can_id == DashControlsLeftCmdMsg::CAN_ID)
  {
    DashControlsLeftCmdMsg encoder;
    encoder.encode(msg->enable,
                   msg->ignore_overrides,
                   msg->clear_override,
                   msg->command);
    return encoder.data;
  }
  else if (can_id == DashControlsRightCmdMsg::CAN_ID)
  {
    DashControlsRightCmdMsg encoder;
    encoder.encode(msg->enable,
                   msg->ignore_overrides,
                   msg->clear_override,
                   msg->command);
    return encoder.data;
  }
  else if (can_id == HeadlightCmdMsg::CAN_ID)
  {
    HeadlightCmdMsg encoder;
    encoder.encode(msg->enable,
                   msg->ignore_overrides,
                   msg->clear_override,
                   msg->command);
    return encoder.data;
  }
  else if (can_id == MediaControlsCmdMsg::CAN_ID)
  {
    MediaControlsCmdMsg encoder;
    encoder.encode(msg->enable,
                   msg->ignore_overrides,
                   msg->clear_override,
                   msg->command);
    return encoder.data;
  }
  else if (can_id == ShiftCmdMsg::CAN_ID)
  {
    ShiftCmdMsg encoder;
    encoder.encode(msg->enable,
                   msg->ignore_overrides,
                   msg->clear_override,
                   msg->command);
    return encoder.data;
  }
  else if (can_id == TurnSignalCmdMsg::CAN_ID)
  {
    TurnSignalCmdMsg encoder;
    encoder.encode(msg->enable,
                   msg->ignore_overrides,
                   msg->clear_override,
                   msg->command);
    return encoder.data;
  }
  else if (can_id == RearPassDoorCmdMsg::CAN_ID)
  {
    RearPassDoorCmdMsg encoder;
    encoder.encode(msg->enable,
                   msg->ignore_overrides,
                   msg->clear_override,
                   msg->command);
    return encoder.data;
  }
  else if (can_id == WiperCmdMsg::CAN_ID)
  {
    WiperCmdMsg encoder;
    encoder.encode(msg->enable,
                   msg->ignore_overrides,
                   msg->clear_override,
                   msg->command);
    return encoder.data;
  }
  else if (can_id == EngineBrakeCmdMsg::CAN_ID)
  {
    EngineBrakeCmdMsg encoder;
    encoder.encode(msg->enable,
                   msg->ignore_overrides,
                   msg->clear_override,
                   msg->command);
    return encoder.data;
  }
  else
  {
    std::vector<uint8_t> bad_id;
    bad_id.assign(8, 0);
    ROS_ERROR("An enum system command matching the provided CAN ID could not be found.");
    return bad_id;
  }
}

std::vector<uint8_t> Pacmod3RxRosMsgHandler::unpackAndEncode(
    const uint32_t& can_id, const pacmod_msgs::SteerSystemCmd::ConstPtr& msg)
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

}  // namespace PACMod3
}  // namespace Drivers
}  // namespace AS
