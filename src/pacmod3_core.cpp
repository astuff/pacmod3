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

#include <pacmod3/pacmod3_core.h>

namespace AS
{
namespace Drivers
{
namespace PACMod3
{

constexpr uint32_t GlobalRptMsg::CAN_ID;
constexpr uint32_t ComponentRptMsg::CAN_ID;

// System Commands
constexpr uint32_t AccelCmdMsg::CAN_ID;
constexpr uint32_t BrakeCmdMsg::CAN_ID;
constexpr uint32_t CruiseControlButtonsCmdMsg::CAN_ID;
constexpr uint32_t DashControlsLeftCmdMsg::CAN_ID;
constexpr uint32_t DashControlsRightCmdMsg::CAN_ID;
constexpr uint32_t EngineBrakeCmdMsg::CAN_ID;
constexpr uint32_t HazardLightCmdMsg::CAN_ID;
constexpr uint32_t HeadlightCmdMsg::CAN_ID;
constexpr uint32_t HornCmdMsg::CAN_ID;
constexpr uint32_t MarkerLampCmdMsg::CAN_ID;
constexpr uint32_t MediaControlsCmdMsg::CAN_ID;
constexpr uint32_t ParkingBrakeCmdMsg::CAN_ID;
constexpr uint32_t RearPassDoorCmdMsg::CAN_ID;
constexpr uint32_t ShiftCmdMsg::CAN_ID;
constexpr uint32_t SprayerCmdMsg::CAN_ID;
constexpr uint32_t SteerCmdMsg::CAN_ID;
constexpr uint32_t TurnSignalCmdMsg::CAN_ID;
constexpr uint32_t WiperCmdMsg::CAN_ID;

constexpr uint8_t SystemCmdBool::DATA_LENGTH;
constexpr uint8_t SystemCmdFloat::DATA_LENGTH;
constexpr uint8_t SystemCmdInt::DATA_LENGTH;
constexpr uint8_t SteerCmdMsg::DATA_LENGTH;

// System Reports
constexpr uint32_t AccelRptMsg::CAN_ID;
constexpr uint32_t BrakeRptMsg::CAN_ID;
constexpr uint32_t CruiseControlButtonsRptMsg::CAN_ID;
constexpr uint32_t DashControlsLeftRptMsg::CAN_ID;
constexpr uint32_t DashControlsRightRptMsg::CAN_ID;
constexpr uint32_t EngineBrakeRptMsg::CAN_ID;
constexpr uint32_t HazardLightRptMsg::CAN_ID;
constexpr uint32_t HeadlightRptMsg::CAN_ID;
constexpr uint32_t HornRptMsg::CAN_ID;
constexpr uint32_t MarkerLampRptMsg::CAN_ID;
constexpr uint32_t MediaControlsRptMsg::CAN_ID;
constexpr uint32_t ParkingBrakeRptMsg::CAN_ID;
constexpr uint32_t RearPassDoorRptMsg::CAN_ID;
constexpr uint32_t ShiftRptMsg::CAN_ID;
constexpr uint32_t SprayerRptMsg::CAN_ID;
constexpr uint32_t SteerRptMsg::CAN_ID;
constexpr uint32_t TurnSignalRptMsg::CAN_ID;
constexpr uint32_t WiperRptMsg::CAN_ID;

// System Aux Reports
constexpr uint32_t AccelAuxRptMsg::CAN_ID;
constexpr uint32_t BrakeAuxRptMsg::CAN_ID;
constexpr uint32_t HeadlightAuxRptMsg::CAN_ID;
constexpr uint32_t ShiftAuxRptMsg::CAN_ID;
constexpr uint32_t SteerAuxRptMsg::CAN_ID;
constexpr uint32_t TurnAuxRptMsg::CAN_ID;
constexpr uint32_t WiperAuxRptMsg::CAN_ID;

// Misc. Reports
constexpr uint32_t VehicleSpeedRptMsg::CAN_ID;
constexpr uint32_t BrakeMotorRpt1Msg::CAN_ID;
constexpr uint32_t BrakeMotorRpt2Msg::CAN_ID;
constexpr uint32_t BrakeMotorRpt3Msg::CAN_ID;
constexpr uint32_t SteerMotorRpt1Msg::CAN_ID;
constexpr uint32_t SteerMotorRpt2Msg::CAN_ID;
constexpr uint32_t SteerMotorRpt3Msg::CAN_ID;
constexpr uint32_t WheelSpeedRptMsg::CAN_ID;
constexpr uint32_t YawRateRptMsg::CAN_ID;
constexpr uint32_t LatLonHeadingRptMsg::CAN_ID;
constexpr uint32_t DateTimeRptMsg::CAN_ID;
constexpr uint32_t DetectedObjectRptMsg::CAN_ID;
constexpr uint32_t VehicleSpecificRpt1Msg::CAN_ID;
constexpr uint32_t VehicleDynamicsRptMsg::CAN_ID;
constexpr uint32_t VinRptMsg::CAN_ID;
constexpr uint32_t OccupancyRptMsg::CAN_ID;
constexpr uint32_t InteriorLightsRptMsg::CAN_ID;
constexpr uint32_t DoorRptMsg::CAN_ID;
constexpr uint32_t RearLightsRptMsg::CAN_ID;
constexpr uint32_t EngineRptMsg::CAN_ID;

std::shared_ptr<Pacmod3TxMsg> Pacmod3TxMsg::make_message(const uint32_t& can_id)
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
  case EngineBrakeRptMsg::CAN_ID:
    return std::shared_ptr<Pacmod3TxMsg>(new EngineBrakeRptMsg);
    break;
  case EngineRptMsg::CAN_ID:
    return std::shared_ptr<Pacmod3TxMsg>(new EngineRptMsg);
    break;
  case GlobalRptMsg::CAN_ID:
    return std::shared_ptr<Pacmod3TxMsg>(new GlobalRptMsg);
    break;
  case HazardLightRptMsg::CAN_ID:
    return std::shared_ptr<Pacmod3TxMsg>(new HazardLightRptMsg);
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
  case MarkerLampRptMsg::CAN_ID:
    return std::shared_ptr<Pacmod3TxMsg>(new MarkerLampRptMsg);
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
  case SprayerRptMsg::CAN_ID:
    return std::shared_ptr<Pacmod3TxMsg>(new SprayerRptMsg);
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
  case RearPassDoorRptMsg::CAN_ID:
    return std::shared_ptr<Pacmod3TxMsg>(new RearPassDoorRptMsg);
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
void GlobalRptMsg::parse(const uint8_t *in)
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

void ComponentRptMsg::parse(const uint8_t *in)
{
  component_type = static_cast<ComponentType>(in[0]);
  component_func = static_cast<ComponentFunction>(in[1]);
  counter = in[2] & 0x0F;
  complement = ((in[2] & 0xF0) >> 4);
  config_fault = ((in[3] & 0x01) > 0);
}

void SystemRptBoolMsg::parse(const uint8_t *in)
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

void SystemRptIntMsg::parse(const uint8_t *in)
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

void SystemRptFloatMsg::parse(const uint8_t *in)
{
  enabled = ((in[0] & 0x01) > 0);
  override_active = ((in[0] & 0x02) > 0);
  command_output_fault = ((in[0] & 0x04) > 0);
  input_output_fault = ((in[0] & 0x08) > 0);
  output_reported_fault = ((in[0] & 0x10) > 0);
  pacmod_fault = ((in[0] & 0x20) > 0);
  vehicle_fault = ((in[0] & 0x40) > 0);

  int16_t temp;

  temp = (static_cast<int16_t>(in[1]) << 8) | in[2];
  manual_input = static_cast<double>(temp / 1000.0);

  temp = (static_cast<int16_t>(in[3]) << 8) | in[4];
  command = static_cast<double>(temp / 1000.0);

  temp = (static_cast<int16_t>(in[5]) << 8) | in[6];
  output = static_cast<double>(temp / 1000.0);
}

void AccelAuxRptMsg::parse(const uint8_t *in)
{
  int16_t temp;

  temp = (static_cast<int16_t>(in[0]) << 8) | in[1];
  raw_pedal_pos = static_cast<float>(temp / 1000.0);

  temp = (static_cast<int16_t>(in[2]) << 8) | in[3];
  raw_pedal_force = static_cast<float>(temp / 1000.0);

  user_interaction = (in[4] & 0x01) > 0;
  raw_pedal_pos_is_valid = (in[5] & 0x01) > 0;
  raw_pedal_force_is_valid = (in[5] & 0x02) > 0;
  user_interaction_is_valid = (in[5] & 0x04) > 0;
}

void BrakeAuxRptMsg::parse(const uint8_t *in)
{
  int16_t temp;

  temp = (static_cast<int16_t>(in[0]) << 8) | in[1];
  raw_pedal_pos = static_cast<float>(temp / 1000.0);

  temp = (static_cast<int16_t>(in[2]) << 8) | in[3];
  raw_pedal_force = static_cast<float>(temp / 1000.0);

  temp = (static_cast<int16_t>(in[4]) << 8) | in[5];
  raw_brake_pressure = static_cast<float>(temp / 1000.0);

  user_interaction = (in[6] & 0x01) > 0;
  brake_on_off = (in[6] & 0x02) > 0;
  raw_pedal_pos_is_valid = (in[7] & 0x01) > 0;
  raw_pedal_force_is_valid = (in[7] & 0x02) > 0;
  raw_brake_pressure_is_valid = (in[7] & 0x04) > 0;
  user_interaction_is_valid = (in[7] & 0x08) > 0;
  brake_on_off_is_valid = (in[7] & 0x10) > 0;
}

void DateTimeRptMsg::parse(const uint8_t *in)
{
  year = in[0];
  month = in[1];
  day = in[2];
  hour = in[3];
  minute = in[4];
  second = in[5];
}

void DetectedObjectRptMsg::parse(const uint8_t *in)
{
  int16_t temp;

  temp = ((static_cast<int16_t>(in[0]) << 8) | in[1]);
  front_object_distance_low_res = static_cast<double>(temp / 1000.0);

  temp = ((static_cast<int16_t>(in[2]) << 8) | in[3]);
  front_object_distance_high_res = static_cast<double>(temp / 1000.0);
}

void DoorRptMsg::parse(const uint8_t *in)
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

void EngineRptMsg::parse(const uint8_t *in)
{
  uint16_t temp1, temp2;
  uint8_t temp3;

  temp1 = ((static_cast<uint16_t>(in[0]) << 8) | in[1]);
  engine_speed = static_cast<float>(temp1 / 4.0);

  temp2 = ((static_cast<uint16_t>(in[2]) << 8) | in[3]);
  engine_torque = static_cast<float>(temp2 / 16.0);

  temp3 = (static_cast<uint8_t>(in[4]) << 8) | in[5];
  engine_coolant_temp = static_cast<int16_t>(temp3 - 40);

  engine_speed_avail = ((in[5] & 0x01) > 0);
  engine_torque_avail = ((in[5] & 0x02) > 0);
  engine_coolant_temp_avail = ((in[5] & 0x04) > 0);
}

void HeadlightAuxRptMsg::parse(const uint8_t *in)
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

void InteriorLightsRptMsg::parse(const uint8_t *in)
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

void LatLonHeadingRptMsg::parse(const uint8_t *in)
{
  latitude_degrees = static_cast<int8_t>(in[0]);
  latitude_minutes = in[1];
  latitude_seconds = in[2];
  longitude_degrees = static_cast<int8_t>(in[3]);
  longitude_minutes = in[4];
  longitude_seconds = in[5];
  heading = ((static_cast<int16_t>(in[6]) << 8) | in[7]) / 100.0;
}

void MotorRpt1Msg::parse(const uint8_t *in)
{
  int32_t temp;

  temp =
      (static_cast<int32_t>(in[0]) << 24) |
      (static_cast<int32_t>(in[1]) << 16) |
      (static_cast<int32_t>(in[2]) << 8) | in[3];
  current = static_cast<double>(temp / 1000.0);

  temp =
    (static_cast<int32_t>(in[4]) << 24) |
    (static_cast<int32_t>(in[5]) << 16) |
    (static_cast<int32_t>(in[6]) << 8) | in[7];
  position = static_cast<double>(temp / 1000.0);
}

void MotorRpt2Msg::parse(const uint8_t *in)
{
  int16_t temp16;
  int32_t temp32;

  temp16 = (static_cast<int16_t>(in[0]) << 8) | in[1];
  encoder_temp = static_cast<double>(temp16);

  temp16 = (static_cast<int16_t>(in[2]) << 8) | in[3];
  motor_temp = static_cast<double>(temp16);

  temp32 =
    (static_cast<int32_t>(in[7]) << 24) |
    (static_cast<int32_t>(in[6]) << 16) |
    (static_cast<int32_t>(in[5]) << 8) | in[4];
  velocity = static_cast<double>(temp32 / 10.0);
}

void MotorRpt3Msg::parse(const uint8_t *in)
{
  int32_t temp;

  temp =
    (static_cast<int32_t>(in[0]) << 24) |
    (static_cast<int32_t>(in[1]) << 16) |
    (static_cast<int32_t>(in[2]) << 8) | in[3];
  torque_output = static_cast<double>(temp / 1000.0);

  temp =
    (static_cast<int32_t>(in[4]) << 24) |
    (static_cast<int32_t>(in[5]) << 16) |
    (static_cast<int32_t>(in[6]) << 8) | in[7];
  torque_input = static_cast<double>(temp / 1000.0);
}

void OccupancyRptMsg::parse(const uint8_t *in)
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

void RearLightsRptMsg::parse(const uint8_t *in)
{
  brake_lights_on = ((in[0] & 0x01) > 0);
  brake_lights_on_is_valid = ((in[1] & 0x01) > 0);
  reverse_lights_on = ((in[0] & 0x02) > 0);
  reverse_lights_on_is_valid = ((in[1] & 0x02) > 0);
}

void ShiftAuxRptMsg::parse(const uint8_t *in)
{
  between_gears = (in[0] & 0x01) > 0;
  stay_in_neutral_mode = (in[0] & 0x02) > 0;
  brake_interlock_active = (in[0] & 0x04) > 0;
  speed_interlock_active = (in[0] & 0x08) > 0;
  between_gears_is_valid = (in[1] & 0x01) > 0;
  stay_in_neutral_mode_is_valid = (in[1] & 0x02) > 0;
  brake_interlock_active_is_valid = (in[1] & 0x04) > 0;
  speed_interlock_active_is_valid = (in[1] & 0x08) > 0;
  gear_number_avail = (in[1] & 0x20) > 0;

  gear_number = static_cast<int8_t>(in[2] & 0x3F);
}

void SteerAuxRptMsg::parse(const uint8_t *in)
{
  int16_t temp;

  temp = (static_cast<int16_t>(in[0]) << 8) | in[1];
  raw_position = temp / 10.0;

  temp = (static_cast<int16_t>(in[2]) << 8) | in[3];
  raw_torque = temp / 10.0;

  uint16_t temp2;

  temp2 = (static_cast<uint16_t>(in[4]) << 8) | in[5];
  rotation_rate = temp2 / 100.0;

  user_interaction = (in[6] & 0x01) > 0;
  raw_position_is_valid = (in[7] & 0x01) > 0;
  raw_torque_is_valid = (in[7] & 0x02) > 0;
  rotation_rate_is_valid = (in[7] & 0x04) > 0;
  user_interaction_is_valid = (in[7] & 0x08) > 0;
}

void TurnAuxRptMsg::parse(const uint8_t *in)
{
  driver_blinker_bulb_on = (in[0] & 0x01) > 0;
  passenger_blinker_bulb_on = (in[0] & 0x02) > 0;
  driver_blinker_bulb_on_is_valid = (in[1] & 0x01) > 0;
  passenger_blinker_bulb_on_is_valid = (in[1] & 0x02) > 0;
}

void VehicleSpecificRpt1Msg::parse(const uint8_t *in)
{
  shift_pos_1 = in[0];
  shift_pos_2 = in[1];
}

void VehicleDynamicsRptMsg::parse(const uint8_t *in)
{
  int16_t temp;

  temp = ((static_cast<int16_t>(in[1]) << 8) | in[2]);
  brake_torque = static_cast<double>(temp / 1000.0);

  g_forces = in[0];
}

void VehicleSpeedRptMsg::parse(const uint8_t *in)
{
  int16_t temp;

  temp = (static_cast<int16_t>(in[0]) << 8) | in[1];
  vehicle_speed = static_cast<double>(temp / 100.0);

  vehicle_speed_valid = (in[2] == 1);
  vehicle_speed_raw[0] = in[3];
  vehicle_speed_raw[1] = in[4];
}

void VinRptMsg::parse(const uint8_t *in)
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

void WheelSpeedRptMsg::parse(const uint8_t *in)
{
  int16_t temp;

  temp = (static_cast<int16_t>(in[0]) << 8) | in[1];
  front_left_wheel_speed = static_cast<double>(temp / 100.0);

  temp = (static_cast<int16_t>(in[2]) << 8) | in[3];
  front_right_wheel_speed = static_cast<double>(temp / 100.0);

  temp = (static_cast<int16_t>(in[4]) << 8) | in[5];
  rear_left_wheel_speed = static_cast<double>(temp / 100.0);

  temp = (static_cast<int16_t>(in[6]) << 8) | in[7];
  rear_right_wheel_speed = static_cast<double>(temp / 100.0);
}

void WiperAuxRptMsg::parse(const uint8_t *in)
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

void YawRateRptMsg::parse(const uint8_t *in)
{
  int16_t temp;

  temp = (static_cast<int16_t>(in[0]) << 8) | in[1];
  yaw_rate = static_cast<double>(temp / 100.0);
}

// RX Messages
void SystemCmdBool::encode(bool enable,
                           bool ignore_overrides,
                           bool clear_override,
                           bool clear_faults,
                           bool cmd)
{
  data.assign(DATA_LENGTH, 0);

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
  data.assign(DATA_LENGTH, 0);

  data[0] = enable ? 0x01 : 0x00;
  data[0] |= ignore_overrides ? 0x02 : 0x00;
  data[0] |= clear_override ? 0x04 : 0x00;
  data[0] |= clear_faults ? 0x08 : 0x00;

  uint16_t cmd_float = static_cast<uint16_t>(cmd * 1000.0);
  data[1] = (cmd_float & 0xFF00) >> 8;
  data[2] = cmd_float & 0x00FF;
}

void SystemCmdInt::encode(bool enable,
                          bool ignore_overrides,
                          bool clear_override,
                          bool clear_faults,
                          uint8_t cmd)
{
  data.assign(DATA_LENGTH, 0);

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
  data.assign(DATA_LENGTH, 0);

  data[0] = enable ? 0x01 : 0x00;
  data[0] |= ignore_overrides ? 0x02 : 0x00;
  data[0] |= clear_override ? 0x04 : 0x00;
  data[0] |= clear_faults ? 0x08 : 0x00;

  int16_t raw_pos = static_cast<int16_t>(1000.0 * steer_pos);
  uint16_t raw_spd = static_cast<uint16_t>(1000.0 * steer_spd);

  data[1] = (raw_pos & 0xFF00) >> 8;
  data[2] = raw_pos & 0x00FF;
  data[3] = (raw_spd & 0xFF00) >> 8;
  data[4] = raw_spd & 0x00FF;
}

}  // namespace PACMod3
}  // namespace Drivers
}  // namespace AS
