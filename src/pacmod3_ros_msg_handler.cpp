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
           can_id == HeadlightRptMsg::CAN_ID ||
           can_id == EngineBrakeRptMsg::CAN_ID ||
           can_id == MediaControlsRptMsg::CAN_ID ||
           can_id == RearPassDoorRptMsg::CAN_ID ||
           can_id == ShiftRptMsg::CAN_ID ||
           can_id == TurnSignalRptMsg::CAN_ID ||
           can_id == WiperRptMsg::CAN_ID)
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
  else if (can_id == ComponentRptMsg00::CAN_ID ||
           can_id == ComponentRptMsg01::CAN_ID ||
           can_id == ComponentRptMsg02::CAN_ID ||
           can_id == ComponentRptMsg03::CAN_ID ||
           can_id == ComponentRptMsg04::CAN_ID)
  {
    pacmod_msgs::ComponentRpt new_msg;
    fillComponentRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == SoftwareVerRptMsg00::CAN_ID ||
           can_id == SoftwareVerRptMsg01::CAN_ID ||
           can_id == SoftwareVerRptMsg02::CAN_ID ||
           can_id == SoftwareVerRptMsg03::CAN_ID ||
           can_id == SoftwareVerRptMsg04::CAN_ID)
  {
    pacmod_msgs::SoftwareVersionRpt new_msg;
    fillSoftwareVersionRpt(parser_class, &new_msg, frame_id);
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
  else if (can_id == CabinClimateRptMsg::CAN_ID)
  {
    pacmod_msgs::CabinClimateRpt new_msg;
    fillCabinClimateRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == SafetyBrakeRptMsg::CAN_ID)
  {
    pacmod_msgs::SafetyBrakeRpt new_msg;
    fillSafetyBrakeRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == SafetyFuncCriticalStopRptMsg::CAN_ID)
  {
    pacmod_msgs::SafetyFuncCriticalStopRpt new_msg;
    fillSafetyFuncCriticalStopRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == SafetyFuncRptMsg::CAN_ID)
  {
    pacmod_msgs::SafetyFuncRpt new_msg;
    fillSafetyFuncRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == EStopRptMsg::CAN_ID)
  {
    pacmod_msgs::EStopRpt new_msg;
    fillEStopRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == WatchdogRptMsg::CAN_ID)
  {
    pacmod_msgs::WatchdogRpt new_msg;
    fillWatchdogRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == WatchdogRpt2Msg::CAN_ID)
  {
    pacmod_msgs::WatchdogRpt2 new_msg;
    fillWatchdogRpt2(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }

// Global
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

// System Aux Reports
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
  else if (can_id == BrakeDeccelAuxRptMsg::CAN_ID)
  {
    pacmod_msgs::BrakeDeccelAuxRpt new_msg;
    fillBrakeDeccelAuxRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == HeadlightAuxRptMsg::CAN_ID)
  {
    pacmod_msgs::HeadlightAuxRpt new_msg;
    fillHeadlightAuxRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == ParkingBrakeAuxRptMsg::CAN_ID)
  {
    pacmod_msgs::ParkingBrakeAuxRpt new_msg;
    fillParkingBrakeAuxRpt(parser_class, &new_msg, frame_id);
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
  else if (can_id == WiperAuxRptMsg::CAN_ID)
  {
    pacmod_msgs::WiperAuxRpt new_msg;
    fillWiperAuxRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }

// Misc Reports
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
  else if (can_id == DetectedObjectRptMsg::CAN_ID)
  {
    pacmod_msgs::DetectedObjectRpt new_msg;
    fillDetectedObjectRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == DoorRptMsg::CAN_ID)
  {
    pacmod_msgs::DoorRpt new_msg;
    fillDoorRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == DriveTrainRptMsg::CAN_ID)
  {
    pacmod_msgs::DriveTrainRpt new_msg;
    fillDriveTrainRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == EngineRptMsg::CAN_ID)
  {
    pacmod_msgs::EngineRpt new_msg;
    fillEngineRpt(parser_class, &new_msg, frame_id);
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
  else if (can_id == LinearAccelRptMsg::CAN_ID)
  {
    pacmod_msgs::LinearAccelRpt new_msg;
    fillLinearAccelRpt(parser_class, &new_msg, frame_id);
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
  else if (can_id == TirePressureRptMsg::CAN_ID)
  {
    pacmod_msgs::TirePressureRpt new_msg;
    fillTirePressureRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == VehDynamicsRptMsg::CAN_ID)
  {
    pacmod_msgs::VehDynamicsRpt new_msg;
    fillVehDynamicsRpt(parser_class, &new_msg, frame_id);
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
  else if (can_id == YawRateRptMsg::CAN_ID)
  {
    pacmod_msgs::YawRateRpt new_msg;
    fillYawRateRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == AccelCmdLimitRptMsg::CAN_ID ||
           can_id == BrakeCmdLimitRptMsg::CAN_ID)          
  {
    pacmod_msgs::SystemCmdLimitRpt new_msg;
    fillSystemCmdLimitRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == SteerCmdLimitRptMsg::CAN_ID)
  {
    pacmod_msgs::SteerCmdLimitRpt new_msg;
    fillSteerCmdLimitRpt(parser_class, &new_msg, frame_id);
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
  new_msg->watchdog_sanity_fault = dc_parser->watchdog_sanity_fault;
  new_msg->watchdog_system_present_fault = dc_parser->watchdog_system_present_fault;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillSoftwareVersionRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::SoftwareVersionRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<SoftwareVersionRptMsg>(parser_class);

  new_msg->mjr = dc_parser->mjr;
  new_msg->mnr = dc_parser->mnr;
  new_msg->patch = dc_parser->patch;
  new_msg->build0 = dc_parser->build0;
  new_msg->build1 = dc_parser->build1;
  new_msg->build2 = dc_parser->build2;
  new_msg->build3 = dc_parser->build3;

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

void Pacmod3TxRosMsgHandler::fillSafetyFuncCriticalStopRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::SafetyFuncCriticalStopRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<SafetyFuncCriticalStopRptMsg>(parser_class);

  new_msg->automaual_opctrl_fault = dc_parser->automaual_opctrl_fault;
  new_msg->remote_stop_fault = dc_parser->remote_stop_fault;
  new_msg->safety_brake_opctrl_off = dc_parser->safety_brake_opctrl_off;
  new_msg->safety_brake_cmd_timeout = dc_parser->safety_brake_cmd_timeout;
  new_msg->safety_func_cmd_timeout = dc_parser->safety_func_cmd_timeout;
  new_msg->safety_func_critical_stop_1_cmd = dc_parser->safety_func_critical_stop_1_cmd;
  new_msg->safety_func_critical_stop_2_cmd = dc_parser->safety_func_critical_stop_2_cmd;
  new_msg->safety_func_none_cmd = dc_parser->safety_func_none_cmd;

  new_msg->pacmod_system_timeout = dc_parser->pacmod_system_timeout;
  new_msg->pacmod_system_fault = dc_parser->pacmod_system_fault;
  new_msg->pacmod_system_not_active = dc_parser->pacmod_system_not_active;
  new_msg->vehicle_report_timeout = dc_parser->vehicle_report_timeout;
  new_msg->vehicle_report_fault = dc_parser->vehicle_report_fault;
  new_msg->low_engine_rpm = dc_parser->low_engine_rpm;
  new_msg->pri_safety_brake_signal_1_fault = dc_parser->pri_safety_brake_signal_1_fault;
  new_msg->pri_safety_brake_signal_2_fault = dc_parser->pri_safety_brake_signal_2_fault;
  
  new_msg->sec_safety_brake_signal_1_fault = dc_parser->sec_safety_brake_signal_1_fault;
  new_msg->sec_safety_brake_signal_2_fault = dc_parser->sec_safety_brake_signal_2_fault;
  new_msg->primary_processor_fault = dc_parser->primary_processor_fault;
  new_msg->secondary_processor_fault = dc_parser->secondary_processor_fault;
  new_msg->remote_stop_cmd = dc_parser->remote_stop_cmd;
  new_msg->pri_safety_brake_pressure_fault = dc_parser->pri_safety_brake_pressure_fault;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillSafetyFuncRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::SafetyFuncRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<SafetyFuncRptMsg>(parser_class);

  new_msg->commanded_val = dc_parser->commanded_val;
  new_msg->state = dc_parser->state;
  new_msg->automanual_opctrl = dc_parser->automanual_opctrl;
  new_msg->cabin_safety_brake_opctrl = dc_parser->cabin_safety_brake_opctrl;
  new_msg->remote_stop_status = dc_parser->remote_stop_status;
  new_msg->engine_status = dc_parser->engine_status;
  new_msg->pacmod_system_status = dc_parser->pacmod_system_status;
  new_msg->user_pc_fault = dc_parser->user_pc_fault;
  new_msg->pacmod_system_fault = dc_parser->pacmod_system_fault;

  new_msg->manual_state_obtainable = dc_parser->manual_state_obtainable;
  new_msg->auto_ready_state_obtainable = dc_parser->auto_ready_state_obtainable;
  new_msg->auto_state_obtainable = dc_parser->auto_state_obtainable;
  new_msg->manual_ready_state_obtainable = dc_parser->manual_ready_state_obtainable;
  new_msg->critical_stop1_state_obtainable = dc_parser->critical_stop1_state_obtainable;
  new_msg->critical_stop2_state_obtainable = dc_parser->critical_stop2_state_obtainable;
  
  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillEStopRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::EStopRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<EStopRptMsg>(parser_class);

  new_msg->estop_status = dc_parser->estop_status;
  new_msg->estop_fault = dc_parser->estop_fault;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();

}

void Pacmod3TxRosMsgHandler::fillWatchdogRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::WatchdogRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<WatchdogRptMsg>(parser_class);

  new_msg->global_enabled_flag = dc_parser->global_enabled_flag;
  new_msg->global_override_active = dc_parser->global_override_active;
  new_msg->global_command_timeout_error = dc_parser->global_command_timeout_error;
  new_msg->global_pacmod_subsystem_timeout = dc_parser->global_pacmod_subsystem_timeout;
  new_msg->global_vehicle_can_timeout = dc_parser->global_vehicle_can_timeout;
  new_msg->global_pacmod_system_fault_active = dc_parser->global_pacmod_system_fault_active;
  new_msg->global_config_fault_active = dc_parser->global_config_fault_active;
  new_msg->global_timeout = dc_parser->global_timeout;

  new_msg->accel_enabled = dc_parser->accel_enabled;
  new_msg->accel_override_active = dc_parser->accel_override_active;
  new_msg->accel_command_output_fault = dc_parser->accel_command_output_fault;
  new_msg->accel_input_output_fault = dc_parser->accel_input_output_fault;
  new_msg->accel_output_reported_fault = dc_parser->accel_output_reported_fault;
  new_msg->accel_pacmod_fault = dc_parser->accel_pacmod_fault;
  new_msg->accel_vehicle_fault = dc_parser->accel_vehicle_fault;
  new_msg->accel_timeout = dc_parser->accel_timeout;

  new_msg->brake_enabled = dc_parser->brake_enabled;
  new_msg->brake_override_active = dc_parser->brake_override_active;
  new_msg->brake_command_output_fault = dc_parser->brake_command_output_fault;
  new_msg->brake_input_output_fault = dc_parser->brake_input_output_fault;
  new_msg->brake_output_reported_fault = dc_parser->brake_output_reported_fault;
  new_msg->brake_pacmod_fault = dc_parser->brake_pacmod_fault;
  new_msg->brake_vehicle_fault = dc_parser->brake_vehicle_fault;
  new_msg->brake_timeout = dc_parser->brake_timeout;
  
  new_msg->shift_enabled = dc_parser->shift_enabled;
  new_msg->shift_override_active = dc_parser->shift_override_active;
  new_msg->shift_command_output_fault = dc_parser->shift_command_output_fault;
  new_msg->shift_input_output_fault = dc_parser->shift_input_output_fault;
  new_msg->shift_output_reported_fault = dc_parser->shift_output_reported_fault;
  new_msg->shift_pacmod_fault = dc_parser->shift_pacmod_fault;
  new_msg->shift_vehicle_fault = dc_parser->shift_vehicle_fault;
  new_msg->shift_timeout = dc_parser->shift_timeout;
  
  new_msg->steer_enabled = dc_parser->steer_enabled;
  new_msg->steer_override_active = dc_parser->steer_override_active;
  new_msg->steer_command_output_fault = dc_parser->steer_command_output_fault;
  new_msg->steer_input_output_fault = dc_parser->steer_input_output_fault;
  new_msg->steer_output_reported_fault = dc_parser->steer_output_reported_fault;
  new_msg->steer_pacmod_fault = dc_parser->steer_pacmod_fault;
  new_msg->steer_vehicle_fault = dc_parser->steer_vehicle_fault;
  new_msg->steer_timeout = dc_parser->steer_timeout;
  
  new_msg->mod1_config_fault = dc_parser->mod1_config_fault;
  new_msg->mod1_can_timeout = dc_parser->mod1_can_timeout;
  new_msg->mod1_counter_fault = dc_parser->mod1_counter_fault;
  new_msg->mod2_config_fault = dc_parser->mod2_config_fault;
  new_msg->mod2_can_timeout = dc_parser->mod2_can_timeout;
  new_msg->mod2_counter_fault = dc_parser->mod2_counter_fault;
  new_msg->mod3_config_fault = dc_parser->mod3_config_fault;
  new_msg->mod3_can_timeout = dc_parser->mod3_can_timeout;
  
  new_msg->mod3_counter_fault = dc_parser->mod3_counter_fault;
  new_msg->mini1_rpt_timeout = dc_parser->mini1_rpt_timeout;
  new_msg->mini1_config_fault = dc_parser->mini1_config_fault;
  new_msg->mini1_can_timeout = dc_parser->mini1_can_timeout;
  new_msg->mini1_counter_fault = dc_parser->mini1_counter_fault;
  new_msg->mini2_rpt_timeout = dc_parser->mini2_rpt_timeout;
  new_msg->mini2_config_fault = dc_parser->mini2_config_fault;
  new_msg->mini2_can_timeout = dc_parser->mini2_can_timeout;
  
  new_msg->mini2_counter_fault = dc_parser->mini2_counter_fault;
  new_msg->mini3_rpt_timeout = dc_parser->mini3_rpt_timeout;
  new_msg->mini3_config_fault = dc_parser->mini3_config_fault;
  new_msg->mini3_can_timeout = dc_parser->mini3_can_timeout;
  new_msg->mini3_counter_fault = dc_parser->mini3_counter_fault;
  new_msg->mod_system_present_fault = dc_parser->mod_system_present_fault;
  new_msg->mini_system_present_fault = dc_parser->mini_system_present_fault;
  new_msg->global_internal_power_supply_fault = dc_parser->global_internal_power_supply_fault;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillWatchdogRpt2(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::WatchdogRpt2 * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<WatchdogRpt2Msg>(parser_class);

  new_msg->accel_rpt_timeout = dc_parser->accel_rpt_timeout;
  new_msg->brake_rpt_timeout = dc_parser->brake_rpt_timeout;
  new_msg->brake_deccel_rpt_timeout = dc_parser->brake_deccel_rpt_timeout;
  new_msg->cabin_climate_rpt_timeout = dc_parser->cabin_climate_rpt_timeout;
  new_msg->cabin_fan_speed_rpt_timeout = dc_parser->cabin_fan_speed_rpt_timeout;
  new_msg->cabin_temp_rpt_timeout = dc_parser->cabin_temp_rpt_timeout;
  new_msg->cruise_control_rpt_timeout = dc_parser->cruise_control_rpt_timeout;
  new_msg->dash_left_rpt_timeout = dc_parser->dash_left_rpt_timeout;

  new_msg->dash_right_rpt_timeout = dc_parser->dash_right_rpt_timeout;
  new_msg->engine_brake_rpt_timeout = dc_parser->engine_brake_rpt_timeout;
  new_msg->hazard_lights_rpt_timeout = dc_parser->hazard_lights_rpt_timeout;
  new_msg->headlight_rpt_timeout = dc_parser->headlight_rpt_timeout;
  new_msg->horn_rpt_timeout = dc_parser->horn_rpt_timeout;
  new_msg->marker_lamp_rpt_timeout = dc_parser->marker_lamp_rpt_timeout;
  new_msg->media_controls_rpt_timeout = dc_parser->media_controls_rpt_timeout;
  new_msg->parking_brake_rpt_timeout = dc_parser->parking_brake_rpt_timeout;

  new_msg->rear_pass_door_rpt_timeout = dc_parser->rear_pass_door_rpt_timeout;
  new_msg->shift_rpt_timeout = dc_parser->shift_rpt_timeout;
  new_msg->sprayer_rpt_timeout = dc_parser->sprayer_rpt_timeout;
  new_msg->steering_rpt_timeout = dc_parser->steering_rpt_timeout;
  new_msg->turn_rpt_timeout = dc_parser->turn_rpt_timeout;
  new_msg->wiper_rpt_timeout = dc_parser->wiper_rpt_timeout;
  new_msg->mod1_sanity_fault = dc_parser->mod1_sanity_fault;
  new_msg->mod2_sanity_fault = dc_parser->mod2_sanity_fault;
  
  new_msg->mod3_sanity_fault = dc_parser->mod3_sanity_fault;
  new_msg->mini1_sanity_fault = dc_parser->mini1_sanity_fault;
  new_msg->mini2_sanity_fault = dc_parser->mini2_sanity_fault;
  new_msg->mini3_sanity_fault = dc_parser->mini3_sanity_fault;
  new_msg->mod1_component_rpt_timeout = dc_parser->mod1_component_rpt_timeout;
  new_msg->mod2_component_rpt_timeout = dc_parser->mod2_component_rpt_timeout;
  new_msg->mod3_component_rpt_timeout = dc_parser->mod3_component_rpt_timeout;
  new_msg->mini1_component_rpt_timeout = dc_parser->mini1_component_rpt_timeout;
  
  new_msg->mini2_component_rpt_timeout = dc_parser->mini2_component_rpt_timeout;
  new_msg->mini3_component_rpt_timeout = dc_parser->mini3_component_rpt_timeout;
  new_msg->mod1_system_present_fault = dc_parser->mod1_system_present_fault;
  new_msg->mod2_system_present_fault = dc_parser->mod2_system_present_fault;
  new_msg->mod3_system_present_fault = dc_parser->mod3_system_present_fault;
  new_msg->mini1_system_present_fault = dc_parser->mini1_system_present_fault;
  new_msg->mini2_system_present_fault = dc_parser->mini2_system_present_fault;
  new_msg->mini3_system_present_fault = dc_parser->mini3_system_present_fault;
  
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
  new_msg->accel_limiting_active = dc_parser->accel_limiting_active;
  new_msg->park_brake_interlock_active = dc_parser->park_brake_interlock_active;

  new_msg->operator_interaction_avail = dc_parser->operator_interaction_avail;
  new_msg->accel_limiting_active_avail = dc_parser->accel_limiting_active_avail;
  new_msg->park_brake_interlock_active_avail = dc_parser->park_brake_interlock_active_avail;

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
  new_msg->brake_limiting_active = dc_parser->brake_limiting_active;
  new_msg->brake_reduced_assist = dc_parser->brake_reduced_assist;

  new_msg->brake_pressure_avail = dc_parser->brake_pressure_avail;
  new_msg->operator_interaction_avail = dc_parser->operator_interaction_avail;
  new_msg->brake_on_off_avail = dc_parser->brake_on_off_avail;
  new_msg->brake_limiting_active_avail = dc_parser->brake_limiting_active_avail;
  new_msg->brake_reduced_assist_avail = dc_parser->brake_reduced_assist_avail;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillBrakeDeccelAuxRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::BrakeDeccelAuxRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<BrakeDeccelAuxRptMsg>(parser_class);

  new_msg->xbr_active_control_mode = dc_parser->xbr_active_control_mode;
  new_msg->xbr_system_state = dc_parser->xbr_system_state;
  new_msg->foundation_brake_use = dc_parser->foundation_brake_use;
  new_msg->hill_holder_mode = dc_parser->hill_holder_mode;
  new_msg->xbr_active_control_mode_avail = dc_parser->xbr_active_control_mode_avail;
  new_msg->xbr_system_state_avail = dc_parser->xbr_system_state_avail;
  new_msg->foundation_brake_use_avail = dc_parser->foundation_brake_use_avail;
  new_msg->hill_holder_mode_avail = dc_parser->hill_holder_mode_avail;

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

void Pacmod3TxRosMsgHandler::fillParkingBrakeAuxRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::ParkingBrakeAuxRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<ParkingBrakeAuxRptMsg>(parser_class);

  new_msg->parking_brake_status = dc_parser->parking_brake_status;
  new_msg->parking_brake_status_avail = dc_parser->parking_brake_status_avail;

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
  new_msg->steering_limiting_active = dc_parser->steering_limiting_active;

  new_msg->steering_torque_avail = dc_parser->steering_torque_avail;
  new_msg->rotation_rate_avail = dc_parser->rotation_rate_avail;
  new_msg->operator_interaction_avail = dc_parser->operator_interaction_avail;
  new_msg->rotation_rate_sign_avail = dc_parser->rotation_rate_sign_avail;
  new_msg->vehicle_angle_calib_status_avail = dc_parser->vehicle_angle_calib_status_avail;
  new_msg->steering_limiting_active_avail = dc_parser->steering_limiting_active_avail;

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

void Pacmod3TxRosMsgHandler::fillDriveTrainRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::DriveTrainRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<DriveTrainRptMsg>(parser_class);

  new_msg->antilock_brake_active = dc_parser->antilock_brake_active;
  new_msg->traction_control_active = dc_parser->traction_control_active;
  new_msg->four_wheel_drive_active = dc_parser->four_wheel_drive_active;
  new_msg->antilock_brake_active_avail = dc_parser->antilock_brake_active_avail;
  new_msg->traction_control_active_avail = dc_parser->traction_control_active_avail;
  new_msg->four_wheel_drive_active_avail = dc_parser->four_wheel_drive_active_avail;

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

void Pacmod3TxRosMsgHandler::fillLinearAccelRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::LinearAccelRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<LinearAccelRptMsg>(parser_class);

  new_msg->lateral_new_data_rx = dc_parser->lateral_new_data_rx;
  new_msg->longitudinal_new_data_rx = dc_parser->longitudinal_new_data_rx;
  new_msg->vertical_new_data_rx = dc_parser->vertical_new_data_rx;

  new_msg->lateral_valid = dc_parser->longitudinal_valid;
  new_msg->longitudinal_valid = dc_parser->longitudinal_valid;
  new_msg->vertical_valid = dc_parser->vertical_valid;
  
  new_msg->lateral_accel = dc_parser->lateral_accel;
  new_msg->longitudinal_accel = dc_parser->longitudinal_accel;
  new_msg->vertical_accel = dc_parser->vertical_accel;

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

void Pacmod3TxRosMsgHandler::fillTirePressureRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::TirePressureRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<TirePressureRptMsg>(parser_class);

  new_msg->front_left_tire_pressure = dc_parser->front_left_tire_pressure;
  new_msg->front_right_tire_pressure = dc_parser->front_right_tire_pressure;
  new_msg->rear_left_tire_pressure = dc_parser->rear_left_tire_pressure;
  new_msg->rear_right_tire_pressure = dc_parser->rear_right_tire_pressure;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillVehDynamicsRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::VehDynamicsRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<VehDynamicsRptMsg>(parser_class);

  new_msg->veh_g_forces = dc_parser->veh_g_forces;

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

void Pacmod3TxRosMsgHandler::fillSystemCmdLimitRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::SystemCmdLimitRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<SystemCmdLimitRptMsg>(parser_class);

  new_msg->sys_cmd_limit = dc_parser->sys_cmd_limit;
  new_msg->limited_sys_cmd = dc_parser->limited_sys_cmd;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillSteerCmdLimitRpt(
    const std::shared_ptr<Pacmod3TxMsg>& parser_class,
    pacmod_msgs::SteerCmdLimitRpt * new_msg,
    const std::string& frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<SteerCmdLimitRptMsg>(parser_class);

  new_msg->pos_cmd_limit = dc_parser->pos_cmd_limit;
  new_msg->limited_pos_cmd = dc_parser->limited_pos_cmd;
  new_msg->rotation_rate_cmd_limit = dc_parser->rotation_rate_cmd_limit;
  new_msg->limited_rotation_rate_cmd = dc_parser->limited_rotation_rate_cmd;

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
  else if (can_id == EngineBrakeCmdMsg::CAN_ID)
  {
    EngineBrakeCmdMsg encoder;
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
  else if (can_id == RearPassDoorCmdMsg::CAN_ID)
  {
    RearPassDoorCmdMsg encoder;
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
  else if (can_id == WiperCmdMsg::CAN_ID)
  {
    WiperCmdMsg encoder;
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

std::vector<uint8_t> Pacmod3RxRosMsgHandler::unpackAndEncode(
    const uint32_t& can_id, const pacmod_msgs::GlobalCmd::ConstPtr& msg)
{
  if (can_id == GlobalCmdMsg::CAN_ID)
  {
    GlobalCmdMsg encoder;
    encoder.encode(msg->clear_faults);
    return encoder.data;
  }
  else
  {
    std::vector<uint8_t> bad_id;
    bad_id.assign(8, 0);
    ROS_ERROR("The global command matching the provided CAN ID could not be found.");
    return bad_id;
  }
}

std::vector<uint8_t> Pacmod3RxRosMsgHandler::unpackAndEncode(
    const uint32_t& can_id, const pacmod_msgs::SupervisoryCtrl::ConstPtr& msg)
{
  if (can_id == SupervisoryCtrlMsg::CAN_ID)
  {
    SupervisoryCtrlMsg encoder;
    encoder.encode(msg->enable,
                   msg->counter,
                   msg->complement);
    return encoder.data;
  }
  else
  {
    std::vector<uint8_t> bad_id;
    bad_id.assign(8, 0);
    ROS_ERROR("The supervisory control message matching the provided CAN ID could not be found.");
    return bad_id;
  }
}

std::vector<uint8_t> Pacmod3RxRosMsgHandler::unpackAndEncode(
    const uint32_t& can_id, const pacmod_msgs::BrakeDeccelCmd::ConstPtr& msg)
{
  if (can_id == BrakeDeccelCmdMsg::CAN_ID)
  {
    BrakeDeccelCmdMsg encoder;
    encoder.encode(msg->enable,
                   msg->ignore_overrides,
                   msg->clear_override,
                   msg->brake_deccel_command,
                   msg->xbr_ebi_mode,
                   msg->xbr_priority,
                   msg->xbr_control_mode);
    return encoder.data;
  }
  else
  {
    std::vector<uint8_t> bad_id;
    bad_id.assign(8, 0);
    ROS_ERROR("The brake deccel command matching the provided CAN ID could not be found.");
    return bad_id;
  }
}

std::vector<uint8_t> Pacmod3RxRosMsgHandler::unpackAndEncode(
    const uint32_t& can_id, const pacmod_msgs::CabinClimateCmd::ConstPtr& msg)
{
  if (can_id == CabinClimateCmdMsg::CAN_ID)
  {
    CabinClimateCmdMsg encoder;
    encoder.encode(msg->enable,
                   msg->ignore_overrides,
                   msg->clear_override,
                   msg->cmd_ac_off_on,
                   msg->cmd_max_ac_off_on,
                   msg->cmd_defrost_off_on,
                   msg->cmd_max_defrost_off_on,
                   msg->cmd_dir_up_off_on,
                   msg->cmd_dir_down_off_on);
    return encoder.data;
  }
  else
  {
    std::vector<uint8_t> bad_id;
    bad_id.assign(8, 0);
    ROS_ERROR("The cabin climate command matching the provided CAN ID could not be found.");
    return bad_id;
  }
}

std::vector<uint8_t> Pacmod3RxRosMsgHandler::unpackAndEncode(
    const uint32_t& can_id, const pacmod_msgs::SafetyBrakeCmd::ConstPtr& msg)
{
  if (can_id == SafetyBrakeCmdMsg::CAN_ID)
  {
    SafetyBrakeCmdMsg encoder;
    encoder.encode(msg->safety_brake_cmd);
    return encoder.data;
  }
  else
  {
    std::vector<uint8_t> bad_id;
    bad_id.assign(8, 0);
    ROS_ERROR("The safety brake command matching the provided CAN ID could not be found.");
    return bad_id;
  }
}

std::vector<uint8_t> Pacmod3RxRosMsgHandler::unpackAndEncode(
    const uint32_t& can_id, const pacmod_msgs::SafetyFuncCmd::ConstPtr& msg)
{
  if (can_id == SafetyFuncCmdMsg::CAN_ID)
  {
    SafetyFuncCmdMsg encoder;
    encoder.encode(msg->command);
    return encoder.data;
  }
  else
  {
    std::vector<uint8_t> bad_id;
    bad_id.assign(8, 0);
    ROS_ERROR("The safety function command matching the provided CAN ID could not be found.");
    return bad_id;
  }
}

std::vector<uint8_t> Pacmod3RxRosMsgHandler::unpackAndEncode(
    const uint32_t& can_id, const pacmod_msgs::NotificationCmd::ConstPtr& msg)
{
  if (can_id == UserNotificationCmdMsg::CAN_ID)
  {
    UserNotificationCmdMsg encoder;
    encoder.encode(msg->buzzer_mute,
                   msg->underdash_lights_white);
    return encoder.data;
  }
  else
  {
    std::vector<uint8_t> bad_id;
    bad_id.assign(8, 0);
    ROS_ERROR("The notification command matching the provided CAN ID could not be found.");
    return bad_id;
  }
}


}  // namespace PACMod3
}  // namespace Drivers
}  // namespace AS
