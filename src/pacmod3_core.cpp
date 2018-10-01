/*
* Unpublished Copyright (c) 2009-2017 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod v3 ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include <pacmod3_core.h>

using namespace AS::Drivers::PACMod3;

const int64_t AS::Drivers::PACMod3::GlobalRptMsg::CAN_ID = 0x10;
const int64_t AS::Drivers::PACMod3::ComponentRptMsg::CAN_ID = 0x20;

// System Commands
const int64_t AS::Drivers::PACMod3::AccelCmdMsg::CAN_ID = 0x100;
const int64_t AS::Drivers::PACMod3::BrakeCmdMsg::CAN_ID = 0x104;
const int64_t AS::Drivers::PACMod3::CruiseControlButtonsCmdMsg::CAN_ID = 0x108;
const int64_t AS::Drivers::PACMod3::DashControlsLeftCmdMsg::CAN_ID = 0x10C;
const int64_t AS::Drivers::PACMod3::DashControlsRightCmdMsg::CAN_ID = 0x110;
const int64_t AS::Drivers::PACMod3::HazardLightCmdMsg::CAN_ID = 0x114;
const int64_t AS::Drivers::PACMod3::HeadlightCmdMsg::CAN_ID = 0x118;
const int64_t AS::Drivers::PACMod3::HornCmdMsg::CAN_ID = 0x11C;
const int64_t AS::Drivers::PACMod3::MediaControlsCmdMsg::CAN_ID = 0x120;
const int64_t AS::Drivers::PACMod3::ParkingBrakeCmdMsg::CAN_ID = 0x124;
const int64_t AS::Drivers::PACMod3::ShiftCmdMsg::CAN_ID = 0x128;
const int64_t AS::Drivers::PACMod3::SteerCmdMsg::CAN_ID = 0x12C;
const int64_t AS::Drivers::PACMod3::TurnSignalCmdMsg::CAN_ID = 0x130;
const int64_t AS::Drivers::PACMod3::WiperCmdMsg::CAN_ID = 0x134;

// System Reports
const int64_t AS::Drivers::PACMod3::AccelRptMsg::CAN_ID = 0x200;
const int64_t AS::Drivers::PACMod3::BrakeRptMsg::CAN_ID = 0x204;
const int64_t AS::Drivers::PACMod3::CruiseControlButtonsRptMsg::CAN_ID = 0x208;
const int64_t AS::Drivers::PACMod3::DashControlsLeftRptMsg::CAN_ID = 0x20C;
const int64_t AS::Drivers::PACMod3::DashControlsRightRptMsg::CAN_ID = 0x210;
const int64_t AS::Drivers::PACMod3::HazardLightRptMsg::CAN_ID = 0x214;
const int64_t AS::Drivers::PACMod3::HeadlightRptMsg::CAN_ID = 0x218;
const int64_t AS::Drivers::PACMod3::HornRptMsg::CAN_ID = 0x21C;
const int64_t AS::Drivers::PACMod3::MediaControlsRptMsg::CAN_ID = 0x220;
const int64_t AS::Drivers::PACMod3::ParkingBrakeRptMsg::CAN_ID = 0x224;
const int64_t AS::Drivers::PACMod3::ShiftRptMsg::CAN_ID = 0x228;
const int64_t AS::Drivers::PACMod3::SteerRptMsg::CAN_ID = 0x22C;
const int64_t AS::Drivers::PACMod3::TurnSignalRptMsg::CAN_ID = 0x230;
const int64_t AS::Drivers::PACMod3::WiperRptMsg::CAN_ID = 0x234;

// System Aux Reports
const int64_t AS::Drivers::PACMod3::AccelAuxRptMsg::CAN_ID = 0x300;
const int64_t AS::Drivers::PACMod3::BrakeAuxRptMsg::CAN_ID = 0x304;
const int64_t AS::Drivers::PACMod3::HeadlightAuxRptMsg::CAN_ID = 0x318;
const int64_t AS::Drivers::PACMod3::ShiftAuxRptMsg::CAN_ID = 0x328;
const int64_t AS::Drivers::PACMod3::SteerAuxRptMsg::CAN_ID = 0x32C;
const int64_t AS::Drivers::PACMod3::TurnAuxRptMsg::CAN_ID = 0x330;
const int64_t AS::Drivers::PACMod3::WiperAuxRptMsg::CAN_ID = 0x334;

// Misc. Reports
const int64_t AS::Drivers::PACMod3::VehicleSpeedRptMsg::CAN_ID = 0x400;
const int64_t AS::Drivers::PACMod3::BrakeMotorRpt1Msg::CAN_ID = 0x401;
const int64_t AS::Drivers::PACMod3::BrakeMotorRpt2Msg::CAN_ID = 0x402;
const int64_t AS::Drivers::PACMod3::BrakeMotorRpt3Msg::CAN_ID = 0x403;
const int64_t AS::Drivers::PACMod3::SteerMotorRpt1Msg::CAN_ID = 0x404;
const int64_t AS::Drivers::PACMod3::SteerMotorRpt2Msg::CAN_ID = 0x405;
const int64_t AS::Drivers::PACMod3::SteerMotorRpt3Msg::CAN_ID = 0x406;
const int64_t AS::Drivers::PACMod3::WheelSpeedRptMsg::CAN_ID = 0x407;
const int64_t AS::Drivers::PACMod3::SteeringPIDRpt1Msg::CAN_ID = 0x408;
const int64_t AS::Drivers::PACMod3::SteeringPIDRpt2Msg::CAN_ID = 0x409;
const int64_t AS::Drivers::PACMod3::SteeringPIDRpt3Msg::CAN_ID = 0x40A;
const int64_t AS::Drivers::PACMod3::YawRateRptMsg::CAN_ID = 0x40D;
const int64_t AS::Drivers::PACMod3::LatLonHeadingRptMsg::CAN_ID = 0x40E;
const int64_t AS::Drivers::PACMod3::DateTimeRptMsg::CAN_ID = 0x40F;
const int64_t AS::Drivers::PACMod3::SteeringPIDRpt4Msg::CAN_ID = 0x410;
const int64_t AS::Drivers::PACMod3::DetectedObjectRptMsg::CAN_ID = 0x411;
const int64_t AS::Drivers::PACMod3::VehicleSpecificRpt1Msg::CAN_ID = 0x412;
const int64_t AS::Drivers::PACMod3::VehicleDynamicsRptMsg::CAN_ID = 0x413;
const int64_t AS::Drivers::PACMod3::VinRptMsg::CAN_ID = 0x414;
const int64_t AS::Drivers::PACMod3::OccupancyRptMsg::CAN_ID = 0x415;
const int64_t AS::Drivers::PACMod3::InteriorLightsRptMsg::CAN_ID = 0x416;
const int64_t AS::Drivers::PACMod3::DoorRptMsg::CAN_ID = 0x417;
const int64_t AS::Drivers::PACMod3::RearLightsRptMsg::CAN_ID = 0x418;

std::shared_ptr<Pacmod3TxMsg> Pacmod3TxMsg::make_message(const int64_t& can_id)
{
  switch (can_id)
  {
    case AccelRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new AccelRptMsg);
      break;
    case BrakeMotorRpt1Msg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new BrakeMotorRpt1Msg);
      break;
    case BrakeMotorRpt2Msg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new BrakeMotorRpt2Msg);
      break;
    case BrakeMotorRpt3Msg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new BrakeMotorRpt3Msg);
      break;
    case BrakeRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new BrakeRptMsg);
      break;
    case ComponentRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new ComponentRptMsg);
      break;
    case CruiseControlButtonsRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new CruiseControlButtonsRptMsg);
      break;
    case DashControlsLeftRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new DashControlsLeftRptMsg);
      break;
    case DashControlsRightRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new DashControlsRightRptMsg);
      break;
    case DateTimeRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new DateTimeRptMsg);
      break;
    case DetectedObjectRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new DetectedObjectRptMsg);
      break;      
    case DoorRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new DoorRptMsg);
      break;
    case GlobalRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new GlobalRptMsg);
      break;
    case HeadlightRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new HeadlightRptMsg);
      break;
    case HornRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new HornRptMsg);
      break;
    case InteriorLightsRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new InteriorLightsRptMsg);
      break;
    case LatLonHeadingRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new LatLonHeadingRptMsg);
      break;
    case MediaControlsRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new MediaControlsRptMsg);
      break;
    case OccupancyRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new OccupancyRptMsg);
      break;
    case ParkingBrakeRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new ParkingBrakeRptMsg);
      break;
    case RearLightsRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new RearLightsRptMsg);
      break;
    case ShiftRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new ShiftRptMsg);
      break;
    case SteeringPIDRpt1Msg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new SteeringPIDRpt1Msg);
      break;
    case SteeringPIDRpt2Msg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new SteeringPIDRpt2Msg);
      break;
    case SteeringPIDRpt3Msg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new SteeringPIDRpt3Msg);
      break;
    case SteeringPIDRpt4Msg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new SteeringPIDRpt4Msg);
      break;
    case SteerMotorRpt1Msg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new SteerMotorRpt1Msg);
      break;
    case SteerMotorRpt2Msg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new SteerMotorRpt2Msg);
      break;
    case SteerMotorRpt3Msg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new SteerMotorRpt3Msg);
      break;
    case SteerRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new SteerRptMsg);
      break;
    case TurnSignalRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new TurnSignalRptMsg);
      break;
    case VehicleSpecificRpt1Msg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new VehicleSpecificRpt1Msg);
      break;      
    case VehicleDynamicsRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new VehicleDynamicsRptMsg);
      break;      
    case VehicleSpeedRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new VehicleSpeedRptMsg);
      break;
    case VinRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new VinRptMsg);
      break;
    case WheelSpeedRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new WheelSpeedRptMsg);
      break;
    case WiperRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new WiperRptMsg);
      break;
    case YawRateRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new YawRateRptMsg);
      break;
    case AccelAuxRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new AccelAuxRptMsg);
      break;
    case BrakeAuxRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new BrakeAuxRptMsg);
      break;
    case HeadlightAuxRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new HeadlightAuxRptMsg);
      break;
    case ShiftAuxRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new ShiftAuxRptMsg);
      break;
    case SteerAuxRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new SteerAuxRptMsg);
      break;
    case TurnAuxRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new TurnAuxRptMsg);
      break;
    case WiperAuxRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new WiperAuxRptMsg);
      break;
    default:
      return NULL;
  }
}

bool Pacmod3TxMsg::isSystem()
{
  return false;
}

SystemRptMsg::SystemRptMsg() :
  Pacmod3TxMsg(),
  enabled(false),
  override_active(false),
  command_output_fault(false),
  input_output_fault(false),
  output_reported_fault(false),
  pacmod_fault(false),
  vehicle_fault(false)
{}

bool SystemRptMsg::isSystem()
{
  return true;
}

SystemRptBoolMsg::SystemRptBoolMsg() :
  SystemRptMsg(),
  manual_input(false),
  command(false),
  output(false)
{}

SystemRptIntMsg::SystemRptIntMsg() :
  SystemRptMsg(),
  manual_input(0),
  command(0),
  output(0)
{}

SystemRptFloatMsg::SystemRptFloatMsg() :
  SystemRptMsg(),
  manual_input(0),
  command(0),
  output(0)
{}

// TX Messages
void GlobalRptMsg::parse(uint8_t *in)
{
  enabled = in[0] & 0x01;
  override_active = ((in[0] & 0x02) > 0);
  fault_active = ((in[0] & 0x80) > 0);
  config_fault_active = ((in[1] & 0x01) > 0);
  user_can_timeout = ((in[0] & 0x04) > 0);
  steering_can_timeout = ((in[0] & 0x08) > 0);
  brake_can_timeout = ((in[0] & 0x10) > 0);
  subsystem_can_timeout = ((in[0] & 0x20) > 0);
  vehicle_can_timeout = ((in[0] & 0x40) > 0);
  user_can_read_errors = ((in[6] << 8) | in[7]);
}

void ComponentRptMsg::parse(uint8_t *in)
{
  component_type = static_cast<ComponentType>(in[0]);
  component_func = static_cast<ComponentFunction>(in[1]);
  counter = in[2] & 0x0F;
  complement = ((in[2] & 0xF0) >> 4);
  config_fault = ((in[3] & 0x01) > 0);
}

void SystemRptBoolMsg::parse(uint8_t *in)
{
  enabled = ((in[0] & 0x01) > 0);
  override_active = ((in[0] & 0x02) > 0);
  command_output_fault = ((in[0] & 0x04) > 0);
  input_output_fault = ((in[0] & 0x08) > 0);
  output_reported_fault = ((in[0] & 0x10) > 0);
  pacmod_fault = ((in[0] & 0x20) > 0);
  vehicle_fault = ((in[0] & 0x40) > 0);

  manual_input = ((in[1] & 0x01) > 0);
  command = ((in[2] & 0x01) > 0);
  output = ((in[3] & 0x01) > 0);
}

void SystemRptIntMsg::parse(uint8_t *in)
{
  enabled = ((in[0] & 0x01) > 0);
  override_active = ((in[0] & 0x02) > 0);
  command_output_fault = ((in[0] & 0x04) > 0);
  input_output_fault = ((in[0] & 0x08) > 0);
  output_reported_fault = ((in[0] & 0x10) > 0);
  pacmod_fault = ((in[0] & 0x20) > 0);
  vehicle_fault = ((in[0] & 0x40) > 0);

  manual_input = in[1];
  command = in[2];
  output = in[3];
}

void SystemRptFloatMsg::parse(uint8_t *in)
{
  enabled = ((in[0] & 0x01) > 0);
  override_active = ((in[0] & 0x02) > 0);
  command_output_fault = ((in[0] & 0x04) > 0);
  input_output_fault = ((in[0] & 0x08) > 0);
  output_reported_fault = ((in[0] & 0x10) > 0);
  pacmod_fault = ((in[0] & 0x20) > 0);
  vehicle_fault = ((in[0] & 0x40) > 0);

  int16_t temp;

  temp = ((int16_t)in[1] << 8) | in[2];
  manual_input = (double)(temp / 1000.0);

  temp = ((int16_t)in[3] << 8) | in[4];
  command = (double)(temp / 1000.0);

  temp = ((int16_t)in[5] << 8) | in[6];
  output = (double)(temp / 1000.0);
}

void AccelAuxRptMsg::parse(uint8_t *in)
{
  int16_t temp;

  temp = ((int16_t)in[0] << 8) | in[1];
  raw_pedal_pos = (float)temp / 1000.0;

  temp = ((int16_t)in[2] << 8) | in[3];
  raw_pedal_force = (float)temp / 1000.0;

  user_interaction = (in[4] & 0x01) > 0;
  raw_pedal_pos_is_valid = (in[5] & 0x01) > 0;
  raw_pedal_force_is_valid = (in[5] & 0x02) > 0;
  user_interaction_is_valid = (in[5] & 0x04) > 0;
}

void BrakeAuxRptMsg::parse(uint8_t *in)
{
  int16_t temp;

  temp = ((int16_t)in[0] << 8) | in[1];
  raw_pedal_pos = (float)temp / 1000.0;

  temp = ((int16_t)in[2] << 8) | in[3];
  raw_pedal_force = (float)temp / 1000.0;

  temp = ((int16_t)in[4] << 8) | in[5];
  raw_brake_pressure = (float)temp / 1000.0;

  user_interaction = (in[6] & 0x01) > 0;
  brake_on_off = (in[6] & 0x02) > 0;
  raw_pedal_pos_is_valid = (in[7] & 0x01) > 0;
  raw_pedal_force_is_valid = (in[7] & 0x02) > 0;
  raw_brake_pressure_is_valid = (in[7] & 0x04) > 0;
  user_interaction_is_valid = (in[7] & 0x08) > 0;
  brake_on_off_is_valid = (in[7] & 0x10) > 0;
}

void DateTimeRptMsg::parse(uint8_t *in)
{
  year = in[0];
  month = in[1];
  day = in[2];
  hour = in[3];
  minute = in[4];
  second = in[5];
}

void DetectedObjectRptMsg::parse(uint8_t *in)
{
  int16_t temp;
  
  temp = (((int16_t)in[0] << 8) | in[1]);
  front_object_distance_low_res = (double)(temp / 1000.0);
  
  temp = (((int16_t)in[2] << 8) | in[3]);
  front_object_distance_high_res = (double)(temp / 1000.0);
}

void DoorRptMsg::parse(uint8_t *in)
{
  driver_door_open = ((in[0] & 0x01) > 0);
  driver_door_open_is_valid = ((in[1] & 0x01) > 0);
  passenger_door_open = ((in[0] & 0x02) > 0);
  passenger_door_open_is_valid = ((in[1] & 0x02) > 0);
  rear_driver_door_open = ((in[0] & 0x04) > 0);
  rear_driver_door_open_is_valid = ((in[1] & 0x04) > 0);
  rear_passenger_door_open = ((in[0] & 0x08) > 0);
  rear_passenger_door_open_is_valid = ((in[1] & 0x08) > 0);
  hood_open = ((in[0] & 0x10) > 0);
  hood_open_is_valid = ((in[1] & 0x10) > 0);
  trunk_open = ((in[0] & 0x20) > 0);
  trunk_open_is_valid = ((in[1] & 0x20) > 0);
  fuel_door_open = ((in[0] & 0x40) > 0);
  fuel_door_open_is_valid = ((in[1] & 0x40) > 0);
}

void HeadlightAuxRptMsg::parse(uint8_t *in)
{
  headlights_on = (in[0] & 0x01) > 0;
  headlights_on_bright = (in[0] & 0x02) > 0;
  fog_lights_on = (in[0] & 0x04) > 0;
  headlights_mode = in[1];
  headlights_on_is_valid = (in[2] & 0x01) > 0;
  headlights_on_bright_is_valid = (in[2] & 0x02) > 0;
  fog_lights_on = (in[2] & 0x04) > 0;
  headlights_mode_is_valid = (in[2] & 0x08) > 0;
}

void InteriorLightsRptMsg::parse(uint8_t *in)
{
  front_dome_lights_on = ((in[0] & 0x01) > 0);
  front_dome_lights_on_is_valid = ((in[2] & 0x01) > 0);
  rear_dome_lights_on = ((in[0] & 0x02) > 0);
  rear_dome_lights_on_is_valid = ((in[2] & 0x02) > 0);
  mood_lights_on = ((in[0] & 0x04) > 0);
  mood_lights_on_is_valid = ((in[2] & 0x04) > 0);
  dim_level = (DimLevel)in[1];
  dim_level_is_valid = ((in[2] & 0x08) > 0);
}

void LatLonHeadingRptMsg::parse(uint8_t *in)
{
  latitude_degrees = (int8_t)in[0];
  latitude_minutes = (uint8_t)in[1];
  latitude_seconds = (uint8_t)in[2];
  longitude_degrees = (int8_t)in[3];
  longitude_minutes = (uint8_t)in[4];
  longitude_seconds = (uint8_t)in[5];
  heading = (((int16_t)in[6] << 8) | in[7]) / 100.0;
}

void MotorRpt1Msg::parse(uint8_t *in)
{
  int32_t temp;

  temp = ((int32_t)in[0] << 24) | ((int32_t)in[1] << 16) | ((int32_t)in[2] << 8) | in[3];
  current = (double)(temp / 1000.0);
  
  temp = ((int32_t)in[4] << 24) | ((int32_t)in[5] << 16) | ((int32_t)in[6] << 8) | in[7];
  position = (double)(temp / 1000.0);
}

void MotorRpt2Msg::parse(uint8_t *in)
{
  int16_t temp16;
  int32_t temp32;

  temp16 = ((int16_t)in[0] << 8) | in[1];
  encoder_temp = (double)temp16;

  temp16 = ((int16_t)in[2] << 8) | in[3];
  motor_temp = (double)temp16;

  temp32 = ((int32_t)in[7] << 24) | ((int32_t)in[6] << 16) | ((int32_t)in[5] << 8) | in[4];
  velocity = (double)(temp32 / 1000.0);
}

void MotorRpt3Msg::parse(uint8_t *in)
{
  int32_t temp;

  temp = ((int32_t)in[0] << 24) | ((int32_t)in[1] << 16) | ((int32_t)in[2] << 8) | in[3];
  torque_output = (double)(temp / 1000.0);

  temp = ((int32_t)in[4] << 24) | ((int32_t)in[5] << 16) | ((int32_t)in[6] << 8) | in[7];
  torque_input = (double)(temp / 1000.0);
}

void OccupancyRptMsg::parse(uint8_t *in)
{
  driver_seat_occupied = ((in[0] & 0x01) > 0);
  driver_seat_occupied_is_valid = ((in[1] & 0x01) > 0);
  passenger_seat_occupied = ((in[0] & 0x02) > 0);
  passenger_seat_occupied_is_valid = ((in[1] & 0x02) > 0);
  rear_seat_occupied = ((in[0] & 0x04) > 0);
  rear_seat_occupied_is_valid = ((in[1] & 0x04) > 0);
  driver_seatbelt_buckled = ((in[0] & 0x08) > 0);
  driver_seatbelt_buckled_is_valid = ((in[1] & 0x08) > 0);
  passenger_seatbelt_buckled = ((in[0] & 0x10) > 0);
  passenger_seatbelt_buckled_is_valid = ((in[1] & 0x10) > 0);
  rear_seatbelt_buckled = ((in[0] & 0x20) > 0);
  rear_seatbelt_buckled_is_valid = ((in[1] & 0x20) > 0);
}

void RearLightsRptMsg::parse(uint8_t *in)
{
  brake_lights_on = ((in[0] & 0x01) > 0);
  brake_lights_on_is_valid = ((in[1] & 0x01) > 0);
  reverse_lights_on = ((in[0] & 0x02) > 0);
  reverse_lights_on_is_valid = ((in[1] & 0x02) > 0);
}

void ShiftAuxRptMsg::parse(uint8_t *in)
{
  between_gears = (in[0] & 0x01) > 0;
  stay_in_neutral_mode = (in[0] & 0x02) > 0;
  brake_interlock_active = (in[0] & 0x04) > 0;
  speed_interlock_active = (in[0] & 0x08) > 0;
  between_gears_is_valid = (in[1] & 0x01) > 0;
  stay_in_neutral_mode_is_valid = (in[1] & 0x02) > 0;
  brake_interlock_active_is_valid = (in[1] & 0x04) > 0;
  speed_interlock_active_is_valid = (in[1] & 0x08) > 0;
}

void SteerAuxRptMsg::parse(uint8_t *in)
{
  int16_t temp;

  temp = ((int16_t)in[0] << 8) | in[1];
  raw_position = (float)temp / 10.0;

  temp = ((int16_t)in[2] << 8) | in[3];
  raw_torque = (float)temp / 10.0;

  uint16_t temp2;

  temp2 = ((uint16_t)in[4] << 8) | in[5];
  rotation_rate = (float)temp2 / 100.0;

  user_interaction = (in[6] & 0x01) > 0;
  raw_position_is_valid = (in[7] & 0x01) > 0;
  raw_torque_is_valid = (in[7] & 0x02) > 0;
  rotation_rate_is_valid = (in[7] & 0x04) > 0;
  user_interaction_is_valid = (in[7] & 0x08) > 0;
}

void SteeringPIDRpt1Msg::parse(uint8_t *in)
{
  int16_t temp;

  temp = ((int16_t)in[0] << 8) | in[1];
  dt = (double)(temp / 1000.0);

  temp = ((int16_t)in[2] << 8) | in[3];
  Kp = (double)(temp / 1000.0);

  temp = ((int16_t)in[4] << 8) | in[5];
  Ki = (double)(temp / 1000.0);

  temp = ((int16_t)in[6] << 8) | in[7];
  Kd = (double)(temp / 1000.0);
}

void SteeringPIDRpt2Msg::parse(uint8_t *in)
{
  int16_t temp;

  temp = ((int16_t)in[0] << 8) | in[1];
  P_term = (double)(temp / 1000.0);

  temp = ((int16_t)in[2] << 8) | in[3];
  I_term = (double)(temp / 1000.0);

  temp = ((int16_t)in[4] << 8) | in[5];
  D_term = (double)(temp / 1000.0);

  temp = ((int16_t)in[6] << 8) | in[7];
  all_terms = (double)(temp / 1000.0);
}

void SteeringPIDRpt3Msg::parse(uint8_t *in)
{
  int16_t temp;

  temp = ((int16_t)in[0] << 8) | in[1];
  new_torque = (double)(temp / 1000.0);

  temp = ((int16_t)in[2] << 8) | in[3];
  str_angle_desired = (double)(temp / 1000.0);

  temp = ((int16_t)in[4] << 8) | in[5];
  str_angle_actual = (double)(temp / 1000.0);

  temp = ((int16_t)in[6] << 8) | in[7];
  error = (double)(temp / 1000.0);
}

void SteeringPIDRpt4Msg::parse(uint8_t *in)
{
  int16_t temp;

  temp = ((int16_t)in[0] << 8) | in[1];
  angular_velocity = (double)(temp / 1000.0);

  temp = ((int16_t)in[2] << 8) | in[3];
  angular_acceleration = (double)(temp / 1000.0);
}

void TurnAuxRptMsg::parse(uint8_t *in)
{
  driver_blinker_bulb_on = (in[0] & 0x01) > 0;
  passenger_blinker_bulb_on = (in[0] & 0x02) > 0;
  driver_blinker_bulb_on_is_valid = (in[1] & 0x01) > 0;
  passenger_blinker_bulb_on_is_valid = (in[1] & 0x02) > 0;
}

void VehicleSpecificRpt1Msg::parse(uint8_t *in)
{
  shift_pos_1 = in[0];
  shift_pos_2 = in[1];
}

void VehicleDynamicsRptMsg::parse(uint8_t *in)
{
  int16_t temp;
  
  temp = (((int16_t)in[1] << 8) | in[2]);
  brake_torque = (double)(temp / 1000.0);
  
  g_forces = in[0];
}

void VehicleSpeedRptMsg::parse(uint8_t *in)
{
  int16_t temp;

  temp = ((int16_t)in[0] << 8) | in[1];
  vehicle_speed = (double)(temp / 100.0);

  vehicle_speed_valid = (in[2] == 1);
  vehicle_speed_raw[0] = in[3];
  vehicle_speed_raw[1] = in[4];
}

void VinRptMsg::parse(uint8_t *in)
{
  std::ostringstream oss;
  oss << in[0] << in[1] << in[2];
  mfg_code = oss.str();

  if (mfg_code == "52C")
    mfg = "POLARIS INDUSTRIES INC.";
  else if (mfg_code == "3HS")
    mfg = "NAVISTAR, INC.";
  else if (mfg_code == "2T2")
    mfg = "TOYOTA MOTOR MANUFACTURING CANADA";
  else
    mfg = "UNKNOWN";

  model_year_code = in[3];

  if (model_year_code >= '1' && model_year_code <= '9')
  {
    model_year = 2000 + model_year_code;
  }
  else if (model_year_code >= 'A' && model_year_code < 'Z')
  {
    switch (model_year_code)
    {
      case 'A':
        model_year = 2010;
        break;
      case 'B':
        model_year = 2011;
        break;
      case 'C':
        model_year = 2012;
        break;
      case 'D':
        model_year = 2013;
        break;
      case 'E':
        model_year = 2014;
        break;
      case 'F':
        model_year = 2015;
        break;
      case 'G':
        model_year = 2016;
        break;
      case 'H':
        model_year = 2017;
        break;
      case 'J':
        model_year = 2018;
        break;
      case 'K':
        model_year = 2019;
        break;
      case 'L':
        model_year = 2020;
        break;
      case 'M':
        model_year = 2021;
        break;
      case 'N':
        model_year = 2022;
        break;
      case 'P':
        model_year = 2023;
        break;
      case 'R':
        model_year = 2024;
        break;
      case 'S':
        model_year = 2025;
        break;
      case 'T':
        model_year = 2026;
        break;
      case 'V':
        model_year = 2027;
        break;
      case 'W':
        model_year = 2028;
        break;
      case 'X':
        model_year = 2029;
        break;
      case 'Y':
        model_year = 2030;
        break;
      default:
        model_year = 9999;
    }
  }
  else
  {
    model_year = 9999;
  }

  serial = (in[4] & 0x0F);
  serial = (serial << 8) | in[5];
  serial = (serial << 8) | in[6];
}

void WheelSpeedRptMsg::parse(uint8_t *in)
{
  int16_t temp;

  temp = ((int16_t)in[0] << 8) | in[1];
  front_left_wheel_speed = (double)(temp / 100.0);

  temp = ((int16_t)in[2] << 8) | in[3];
  front_right_wheel_speed = (double)(temp / 100.0);

  temp = ((int16_t)in[4] << 8) | in[5];
  rear_left_wheel_speed = (double)(temp / 100.0);

  temp = ((int16_t)in[6] << 8) | in[7];
  rear_right_wheel_speed = (double)(temp / 100.0);
}

void WiperAuxRptMsg::parse(uint8_t *in)
{
  front_wiping = (in[0] & 0x01) > 0;
  front_spraying = (in[0] & 0x02) > 0;
  rear_wiping = (in[0] & 0x04) > 0;
  rear_spraying = (in[0] & 0x08) > 0;
  spray_near_empty = (in[0] & 0x10) > 0;
  spray_empty = (in[0] & 0x20) > 0;
  front_wiping_is_valid = (in[1] & 0x01) > 0;
  front_spraying_is_valid = (in[1] & 0x02) > 0;
  rear_wiping_is_valid = (in[1] & 0x04) > 0;
  rear_spraying_is_valid = (in[1] & 0x08) > 0;
  spray_near_empty_is_valid = (in[1] & 0x10) > 0;
  spray_empty_is_valid = (in[1] & 0x20) > 0;
}

void YawRateRptMsg::parse(uint8_t *in)
{
  int16_t temp;
  
  temp = ((int16_t)in[0] << 8) | in[1];
  yaw_rate = (double)(temp / 100.0);
}

// RX Messages
void SystemCmdBool::encode(bool enable,
                           bool ignore_overrides,
                           bool clear_override,
                           bool clear_faults,
                           bool cmd)
{
  data.assign(8, 0);

  data[0] = (enable ? 0x01 : 0x00);
  data[0] |= (ignore_overrides ? 0x02 : 0x00);
  data[0] |= clear_override ? 0x04 : 0x00;
  data[0] |= clear_faults ? 0x08 : 0x00;
  data[1] = (cmd ? 0x01 : 0x00);
}

void SystemCmdFloat::encode(bool enable,
                            bool ignore_overrides,
                            bool clear_override,
                            bool clear_faults,
                            float cmd)
{
  data.assign(8, 0);

  data[0] = enable ? 0x01 : 0x00;
  data[0] |= ignore_overrides ? 0x02 : 0x00;
  data[0] |= clear_override ? 0x04 : 0x00;
  data[0] |= clear_faults ? 0x08 : 0x00;

  uint16_t cmd_float = (uint16_t)(cmd * 1000.0);
  data[1] = (cmd_float & 0xFF00) >> 8;
  data[2] = cmd_float & 0x00FF;
}

void SystemCmdInt::encode(bool enable,
                          bool ignore_overrides,
                          bool clear_override,
                          bool clear_faults,
                          uint8_t cmd)
{
  data.assign(8, 0);

  data[0] = enable ? 0x01 : 0x00;
  data[0] |= ignore_overrides ? 0x02 : 0x00;
  data[0] |= clear_override ? 0x04 : 0x00;
  data[0] |= clear_faults ? 0x08 : 0x00;
  data[1] = cmd;
}

void SteerCmdMsg::encode(bool enable,
                         bool ignore_overrides,
                         bool clear_override,
                         bool clear_faults,
                         float steer_pos,
                         float steer_spd)
{
  data.assign(8, 0);

  data[0] = enable ? 0x01 : 0x00;
  data[0] |= ignore_overrides ? 0x02 : 0x00;
  data[0] |= clear_override ? 0x04 : 0x00;
  data[0] |= clear_faults ? 0x08 : 0x00;

  int16_t raw_pos = (int16_t)(1000.0 * steer_pos);
  uint16_t raw_spd = (uint16_t)(1000.0 * steer_spd);

  data[1] = (raw_pos & 0xFF00) >> 8;
  data[2] = raw_pos & 0x00FF;
  data[3] = (raw_spd & 0xFF00) >> 8;    
  data[4] = raw_spd & 0x00FF;
}
