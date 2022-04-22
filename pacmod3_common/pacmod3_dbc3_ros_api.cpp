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

#include <pacmod3_dbc3_ros_api.h>
#include <autogen/pacmod3.h>

#include <vector>
#include <string>
#include <memory>


namespace pacmod3
{

Dbc3Api::Dbc3Api() : DbcApi(3)
{}

std::shared_ptr<void> Dbc3Api::ParseAccelAuxRpt(const can_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::AccelAuxRpt> new_msg( new pm_msgs::AccelAuxRpt() );

  ACCEL_AUX_RPT_t parsed_rpt;
  Unpack_ACCEL_AUX_RPT_pacmod3(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->operator_interaction = parsed_rpt.USER_INTERACTION;
  // Following fields not present in dbc3
  new_msg->accel_limiting_active = 0;
  new_msg->park_brake_interlock_active = 0;
  new_msg->brake_interlock_active = 0;

  new_msg->operator_interaction_avail = parsed_rpt.USER_INTERACTION_IS_VALID;
  // Following fields not present in dbc3
  new_msg->accel_limiting_active_avail = 0;
  new_msg->park_brake_interlock_active_avail = 0;
  new_msg->brake_interlock_active_avail = 0;

  return new_msg;
}

std::shared_ptr<void> Dbc3Api::ParseAngVelRpt(const can_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::AngVelRpt> new_msg( new pm_msgs::AngVelRpt() );

  PrintParseError("AngVelRpt");

  return new_msg;
}

std::shared_ptr<void> Dbc3Api::ParseBrakeAuxRpt(const can_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::BrakeAuxRpt> new_msg( new pm_msgs::BrakeAuxRpt() );

  BRAKE_AUX_RPT_t parsed_rpt;
  Unpack_BRAKE_AUX_RPT_pacmod3(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->brake_pressure = parsed_rpt.RAW_BRAKE_PRESSURE;
  new_msg->operator_interaction = parsed_rpt.USER_INTERACTION;
  new_msg->brake_on_off = parsed_rpt.BRAKE_ON_OFF;
  // Following fields not present in dbc3
  new_msg->brake_limiting_active = 0;
  new_msg->brake_reduced_assist = 0;

  new_msg->brake_pressure_avail = parsed_rpt.RAW_BRAKE_PRESSURE_IS_VALID;
  new_msg->operator_interaction_avail = parsed_rpt.USER_INTERACTION_IS_VALID;
  new_msg->brake_on_off_avail = parsed_rpt.BRAKE_ON_OFF_IS_VALID;
  // Following fields not present in dbc3
  new_msg->brake_limiting_active_avail = 0;
  new_msg->brake_reduced_assist_avail = 0;

  return new_msg;
}

std::shared_ptr<void> Dbc3Api::ParseComponentRpt(const can_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::ComponentRpt> new_msg( new pm_msgs::ComponentRpt() );

  COMPONENT_RPT_t parsed_rpt;
  Unpack_COMPONENT_RPT_pacmod3(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->component_type = parsed_rpt.COMPONENT_TYPE;

  // Following fields not present in dbc3
  new_msg->accel = 0;
  new_msg->brake = 0;
  new_msg->cruise_control_buttons = 0;
  new_msg->dash_controls_left = 0;
  new_msg->dash_controls_right = 0;
  new_msg->hazard_lights = 0;
  new_msg->headlight = 0;
  new_msg->horn = 0;
  new_msg->media_controls = 0;
  new_msg->parking_brake = 0;
  new_msg->shift = 0;
  new_msg->steering = 0;
  new_msg->turn = 0;
  new_msg->wiper = 0;
  new_msg->brake_deccel = 0;
  new_msg->cabin_climate = 0;
  new_msg->cabin_fan_speed = 0;
  new_msg->cabin_temp = 0;
  new_msg->engine_brake = 0;
  new_msg->marker_lamp = 0;
  new_msg->rear_pass_door = 0;
  new_msg->sprayer = 0;
  new_msg->watchdog = 0;

  new_msg->counter = parsed_rpt.COUNTER;
  new_msg->complement = parsed_rpt.COMPLEMENT;
  new_msg->config_fault = parsed_rpt.CONFIG_FAULT;

  // Following fields not present in dbc3
  new_msg->can_timeout_fault = 0;
  new_msg->internal_supply_voltage_fault = 0;
  new_msg->supervisory_timeout = 0;
  new_msg->supervisory_sanity_fault = 0;
  new_msg->watchdog_sanity_fault = 0;
  new_msg->watchdog_system_present_fault = 0;

  return new_msg;
}

std::shared_ptr<void> Dbc3Api::ParseDateTimeRpt(const can_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::DateTimeRpt> new_msg( new pm_msgs::DateTimeRpt() );

  DATE_TIME_RPT_t parsed_rpt;
  Unpack_DATE_TIME_RPT_pacmod3(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->year = parsed_rpt.DATE_YEAR_phys;
  new_msg->month = parsed_rpt.DATE_MONTH_phys;
  new_msg->day = parsed_rpt.DATE_DAY_phys;

  new_msg->hour = parsed_rpt.TIME_HOUR;
  new_msg->minute = parsed_rpt.TIME_MINUTE;
  new_msg->second = parsed_rpt.TIME_SECOND;

  return new_msg;
}

std::shared_ptr<void> Dbc3Api::ParseDetectedObjectRpt(const can_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::DetectedObjectRpt> new_msg( new pm_msgs::DetectedObjectRpt() );

  DETECTED_OBJECT_RPT_t parsed_rpt;
  Unpack_DETECTED_OBJECT_RPT_pacmod3(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->front_object_distance_low_res = parsed_rpt.FRONT_OBJECT_DISTANCE_LOW_RES_phys;
  new_msg->front_object_distance_high_res = parsed_rpt.FRONT_OBJECT_DISTANCE_HIGH_RES_phys;

  return new_msg;
}

std::shared_ptr<void> Dbc3Api::ParseDoorRpt(const can_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::DoorRpt> new_msg( new pm_msgs::DoorRpt() );

  DOOR_RPT_t parsed_rpt;
  Unpack_DOOR_RPT_pacmod3(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->driver_door_open = parsed_rpt.DRIVER_DOOR_OPEN;
  new_msg->passenger_door_open = parsed_rpt.PASS_DOOR_OPEN;
  new_msg->rear_driver_door_open = parsed_rpt.REAR_DRIVER_DOOR_OPEN;
  new_msg->rear_passenger_door_open = parsed_rpt.REAR_PASS_DOOR_OPEN;
  new_msg->hood_open = parsed_rpt.HOOD_OPEN;
  new_msg->trunk_open = parsed_rpt.TRUNK_OPEN;
  new_msg->fuel_door_open = parsed_rpt.FUEL_DOOR_OPEN;
  new_msg->driver_door_open_avail = parsed_rpt.DRIVER_DOOR_OPEN_IS_VALID;
  new_msg->passenger_door_open_avail = parsed_rpt.PASS_DOOR_OPEN_IS_VALID;
  new_msg->rear_driver_door_open_avail = parsed_rpt.REAR_DRIVER_DOOR_OPEN_IS_VALID;
  new_msg->rear_passenger_door_open_avail = parsed_rpt.REAR_PASS_DOOR_OPEN_IS_VALID;
  new_msg->hood_open_avail = parsed_rpt.HOOD_OPEN_IS_VALID;
  new_msg->trunk_open_avail = parsed_rpt.TRUNK_OPEN_IS_VALID;
  new_msg->fuel_door_open_avail = parsed_rpt.FUEL_DOOR_OPEN_IS_VALID;

  return new_msg;
}

std::shared_ptr<void> Dbc3Api::ParseEngineRpt(const can_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::EngineRpt> new_msg( new pm_msgs::EngineRpt() );

  PrintParseError("EngineRpt");

  return new_msg;
}

std::shared_ptr<void> Dbc3Api::ParseGlobalRpt(const can_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::GlobalRpt> new_msg( new pm_msgs::GlobalRpt() );

  GLOBAL_RPT_t parsed_rpt;
  Unpack_GLOBAL_RPT_pacmod3(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->enabled = parsed_rpt.PACMOD_SYSTEM_ENABLED;
  new_msg->override_active = parsed_rpt.PACMOD_SYSTEM_OVERRIDE_ACTIVE;
  new_msg->pacmod_sys_fault_active = parsed_rpt.PACMOD_SYSTEM_FAULT_ACTIVE;
  new_msg->config_fault_active = parsed_rpt.CONFIG_FAULT_ACTIVE;
  new_msg->user_can_timeout = parsed_rpt.USR_CAN_TIMEOUT;
  new_msg->steering_can_timeout = parsed_rpt.STR_CAN_TIMEOUT;
  new_msg->brake_can_timeout = parsed_rpt.BRK_CAN_TIMEOUT;
  new_msg->subsystem_can_timeout = parsed_rpt.PACMOD_SUBSYSTEM_TIMEOUT;
  new_msg->vehicle_can_timeout = parsed_rpt.VEH_CAN_TIMEOUT;
  new_msg->user_can_read_errors = parsed_rpt.USR_CAN_READ_ERRORS;
  new_msg->supervisory_enable_required = 0;  // dbc3 doesn't have this field

  return new_msg;
}

std::shared_ptr<void> Dbc3Api::ParseHeadlightAuxRpt(const can_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::HeadlightAuxRpt> new_msg( new pm_msgs::HeadlightAuxRpt() );

  HEADLIGHT_AUX_RPT_t parsed_rpt;
  Unpack_HEADLIGHT_AUX_RPT_pacmod3(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->headlights_on = parsed_rpt.HEADLIGHTS_ON;
  new_msg->headlights_on_bright = parsed_rpt.HEADLIGHTS_ON_BRIGHT;
  new_msg->fog_lights_on = parsed_rpt.FOG_LIGHTS_ON;
  new_msg->headlights_mode = parsed_rpt.HEADLIGHTS_MODE;
  new_msg->headlights_on_avail = parsed_rpt.HEADLIGHTS_ON_IS_VALID;
  new_msg->headlights_on_bright_avail = parsed_rpt.HEADLIGHTS_ON_BRIGHT_IS_VALID;
  new_msg->fog_lights_on_avail = parsed_rpt.FOG_LIGHTS_ON_IS_VALID;
  new_msg->headlights_mode_avail = parsed_rpt.HEADLIGHTS_MODE_IS_VALID;

  return new_msg;
}

std::shared_ptr<void> Dbc3Api::ParseInteriorLightsRpt(const can_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::InteriorLightsRpt> new_msg( new pm_msgs::InteriorLightsRpt() );

  INTERIOR_LIGHTS_RPT_t parsed_rpt;
  Unpack_INTERIOR_LIGHTS_RPT_pacmod3(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->front_dome_lights_on = parsed_rpt.FRONT_DOME_LIGHTS_ON;
  new_msg->front_dome_lights_on_avail = parsed_rpt.FRONT_DOME_LIGHTS_ON_IS_VALID;
  new_msg->rear_dome_lights_on = parsed_rpt.REAR_DOME_LIGHTS_ON;
  new_msg->rear_dome_lights_on_avail = parsed_rpt.REAR_DOME_LIGHTS_ON_IS_VALID;
  new_msg->mood_lights_on = parsed_rpt.MOOD_LIGHTS_ON;
  new_msg->mood_lights_on_avail = parsed_rpt.MOOD_LIGHTS_ON_IS_VALID;
  new_msg->dim_level = parsed_rpt.DIM_LEVEL;
  new_msg->dim_level_avail = parsed_rpt.DIM_LEVEL_IS_VALID;

  // Not available in dbc3
  new_msg->ambient_light_sensor = false;
  new_msg->ambient_light_sensor_avail = false;

  return new_msg;
}

std::shared_ptr<void> Dbc3Api::ParseLatLonHeadingRpt(const can_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::LatLonHeadingRpt> new_msg( new pm_msgs::LatLonHeadingRpt() );

  LAT_LON_HEADING_RPT_t parsed_rpt;
  Unpack_LAT_LON_HEADING_RPT_pacmod3(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->latitude_degrees = parsed_rpt.LATITUDE_DEGREES;
  new_msg->latitude_minutes = parsed_rpt.LATITUDE_MINUTES;
  new_msg->latitude_seconds = parsed_rpt.LATITUDE_SECONDS;
  new_msg->longitude_degrees = parsed_rpt.LONGITUDE_DEGREES;
  new_msg->longitude_minutes = parsed_rpt.LONGITUDE_MINUTES;
  new_msg->longitude_seconds = parsed_rpt.LONGITUDE_SECONDS;
  new_msg->heading = parsed_rpt.HEADING_phys;

  return new_msg;
}

std::shared_ptr<void> Dbc3Api::ParseLinearAccelRpt(const can_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::LinearAccelRpt> new_msg( new pm_msgs::LinearAccelRpt() );

  PrintParseError("LinearAccelRpt");

  return new_msg;
}

std::shared_ptr<void> Dbc3Api::ParseMotorRpt1(const can_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::MotorRpt1> new_msg( new pm_msgs::MotorRpt1() );

  BRAKE_MOTOR_RPT_1_t parsed_rpt;
  Unpack_BRAKE_MOTOR_RPT_1_pacmod3(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->current = parsed_rpt.MOTOR_CURRENT_phys;
  new_msg->position = parsed_rpt.SHAFT_POSITION_phys;

  return new_msg;
}

std::shared_ptr<void> Dbc3Api::ParseMotorRpt2(const can_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::MotorRpt2> new_msg( new pm_msgs::MotorRpt2() );

  BRAKE_MOTOR_RPT_2_t parsed_rpt;
  Unpack_BRAKE_MOTOR_RPT_2_pacmod3(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->encoder_temp = parsed_rpt.ENCODER_TEMPERATURE;
  new_msg->motor_temp = parsed_rpt.MOTOR_TEMPERATURE;
  new_msg->angular_speed = parsed_rpt.ANGULAR_SPEED_phys;

  return new_msg;
}

std::shared_ptr<void> Dbc3Api::ParseMotorRpt3(const can_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::MotorRpt3> new_msg( new pm_msgs::MotorRpt3() );

  BRAKE_MOTOR_RPT_3_t parsed_rpt;
  Unpack_BRAKE_MOTOR_RPT_3_pacmod3(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->torque_output = parsed_rpt.TORQUE_OUTPUT_phys;
  new_msg->torque_input = parsed_rpt.TORQUE_INPUT_phys;

  return new_msg;
}

std::shared_ptr<void> Dbc3Api::ParseOccupancyRpt(const can_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::OccupancyRpt> new_msg( new pm_msgs::OccupancyRpt() );

  OCCUPANCY_RPT_t parsed_rpt;
  Unpack_OCCUPANCY_RPT_pacmod3(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->driver_seat_occupied = parsed_rpt.DRIVER_SEAT_OCCUPIED;
  new_msg->driver_seat_occupied_avail = parsed_rpt.DRIVER_SEAT_OCCUPIED_IS_VALID;
  new_msg->passenger_seat_occupied = parsed_rpt.PASS_SEAT_OCCUPIED;
  new_msg->passenger_seat_occupied_avail = parsed_rpt.PASS_SEAT_OCCUPIED_IS_VALID;
  new_msg->rear_seat_occupied = parsed_rpt.REAR_SEAT_OCCUPIED;
  new_msg->rear_seat_occupied_avail = parsed_rpt.REAR_SEAT_OCCUPIED_IS_VALID;
  new_msg->driver_seatbelt_buckled = parsed_rpt.DRIVER_SEATBELT_BUCKLED;
  new_msg->driver_seatbelt_buckled_avail = parsed_rpt.DRIVER_SEATBELT_BUCKLED_IS_VALID;
  new_msg->passenger_seatbelt_buckled = parsed_rpt.PASS_SEATBELT_BUCKLED;
  new_msg->passenger_seatbelt_buckled_avail = parsed_rpt.PASS_SEATBELT_BUCKLED_IS_VALID;

  // dbc3 uses the field "rear seatbelt", it has been mapped to center_rear_seatbelt
  new_msg->center_rear_seatbelt_buckled = parsed_rpt.REAR_SEATBELT_BUCKLED;
  new_msg->center_rear_seatbelt_buckled_avail = parsed_rpt.REAR_SEATBELT_BUCKLED_IS_VALID;

  // Following fields not present in dbc3
  new_msg->driver_rear_seatbelt_buckled = false;
  new_msg->driver_rear_seatbelt_buckled_avail = false;
  new_msg->pass_rear_seatbelt_buckled = false;
  new_msg->pass_rear_seatbelt_buckled_avail = false;

  return new_msg;
}

std::shared_ptr<void> Dbc3Api::ParseRearLightsRpt(const can_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::RearLightsRpt> new_msg( new pm_msgs::RearLightsRpt() );

  REAR_LIGHTS_RPT_t parsed_rpt;
  Unpack_REAR_LIGHTS_RPT_pacmod3(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->brake_lights_on = parsed_rpt.BRAKE_LIGHTS_ON;
  new_msg->brake_lights_on_avail = parsed_rpt.BRAKE_LIGHTS_ON_IS_VALID;
  new_msg->reverse_lights_on = parsed_rpt.REVERSE_LIGHTS_ON;
  new_msg->reverse_lights_on_avail = parsed_rpt.REVERSE_LIGHTS_ON_IS_VALID;

  return new_msg;
}

std::shared_ptr<void> Dbc3Api::ParseShiftAuxRpt(const can_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::ShiftAuxRpt> new_msg( new pm_msgs::ShiftAuxRpt() );

  SHIFT_AUX_RPT_t parsed_rpt;
  Unpack_SHIFT_AUX_RPT_pacmod3(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->between_gears = parsed_rpt.BETWEEN_GEARS;
  new_msg->stay_in_neutral_mode = parsed_rpt.STAY_IN_NEUTRAL_MODE;
  new_msg->brake_interlock_active = parsed_rpt.BRAKE_INTERLOCK_ACTIVE;
  new_msg->speed_interlock_active = parsed_rpt.SPEED_INTERLOCK_ACTIVE;
  new_msg->between_gears_avail = parsed_rpt.BETWEEN_GEARS_IS_VALID;
  new_msg->stay_in_neutral_mode_avail = parsed_rpt.STAY_IN_NEUTRAL_MODE_IS_VALID;
  new_msg->brake_interlock_active_avail = parsed_rpt.BRAKE_INTERLOCK_ACTIVE_IS_VALID;
  new_msg->speed_interlock_active_avail = parsed_rpt.SPEED_INTERLOCK_ACTIVE_IS_VALID;

  // Following fields not present in dbc3
  new_msg->gear_number = 0;
  new_msg->gear_number_avail = false;
  new_msg->write_to_config = false;
  new_msg->write_to_config_is_valid = false;

  return new_msg;
}

std::shared_ptr<void> Dbc3Api::ParseSteeringAuxRpt(const can_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::SteeringAuxRpt> new_msg( new pm_msgs::SteeringAuxRpt() );

  STEERING_AUX_RPT_t parsed_rpt;
  Unpack_STEERING_AUX_RPT_pacmod3(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->steering_torque = parsed_rpt.RAW_TORQUE_phys;
  new_msg->rotation_rate = parsed_rpt.ROTATION_RATE_phys;
  new_msg->operator_interaction = parsed_rpt.USER_INTERACTION;
  new_msg->steering_torque_avail = parsed_rpt.RAW_TORQUE_IS_VALID;
  new_msg->rotation_rate_avail = parsed_rpt.ROTATION_RATE_IS_VALID;
  new_msg->operator_interaction_avail = parsed_rpt.USER_INTERACTION_IS_VALID;

  // Following fields not present in dbc3
  new_msg->rotation_rate_sign = false;
  new_msg->vehicle_angle_calib_status = false;
  new_msg->steering_limiting_active = false;
  new_msg->rotation_rate_sign_avail = false;
  new_msg->vehicle_angle_calib_status_avail = false;
  new_msg->steering_limiting_active_avail = false;

  return new_msg;
}

std::shared_ptr<void> Dbc3Api::ParseSystemRptBool(const can_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::SystemRptBool> new_msg( new pm_msgs::SystemRptBool() );

  HORN_RPT_t parsed_rpt;
  Unpack_HORN_RPT_pacmod3(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->enabled = parsed_rpt.ENABLED;
  new_msg->override_active = parsed_rpt.OVERRIDE_ACTIVE;
  new_msg->command_output_fault = parsed_rpt.COMMAND_OUTPUT_FAULT;
  new_msg->input_output_fault = parsed_rpt.INPUT_OUTPUT_FAULT;
  new_msg->output_reported_fault = parsed_rpt.OUTPUT_REPORTED_FAULT;
  new_msg->pacmod_fault = parsed_rpt.PACMOD_FAULT;
  new_msg->vehicle_fault = parsed_rpt.VEHICLE_FAULT;
  new_msg->command_timeout = 0;  // dbc3 doesn't have this field
  new_msg->manual_input = parsed_rpt.MANUAL_INPUT;
  new_msg->command = parsed_rpt.COMMANDED_VALUE;
  new_msg->output = parsed_rpt.OUTPUT_VALUE;

  return new_msg;
}

std::shared_ptr<void> Dbc3Api::ParseSystemRptFloat(const can_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::SystemRptFloat> new_msg( new pm_msgs::SystemRptFloat() );

  ACCEL_RPT_t parsed_rpt;
  Unpack_ACCEL_RPT_pacmod3(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->enabled = parsed_rpt.ENABLED;
  new_msg->override_active = parsed_rpt.OVERRIDE_ACTIVE;
  new_msg->command_output_fault = parsed_rpt.COMMAND_OUTPUT_FAULT;
  new_msg->input_output_fault = parsed_rpt.INPUT_OUTPUT_FAULT;
  new_msg->output_reported_fault = parsed_rpt.OUTPUT_REPORTED_FAULT;
  new_msg->pacmod_fault = parsed_rpt.PACMOD_FAULT;
  new_msg->vehicle_fault = parsed_rpt.VEHICLE_FAULT;
  new_msg->command_timeout = 0;  // dbc3 doesn't have this field
  new_msg->manual_input = parsed_rpt.MANUAL_INPUT_phys;
  new_msg->command = parsed_rpt.COMMANDED_VALUE_phys;
  new_msg->output = parsed_rpt.OUTPUT_VALUE_phys;

  return new_msg;
}

std::shared_ptr<void> Dbc3Api::ParseSystemRptInt(const can_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::SystemRptInt> new_msg( new pm_msgs::SystemRptInt() );

  SHIFT_RPT_t parsed_rpt;
  Unpack_SHIFT_RPT_pacmod3(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->enabled = parsed_rpt.ENABLED;
  new_msg->override_active = parsed_rpt.OVERRIDE_ACTIVE;
  new_msg->command_output_fault = parsed_rpt.COMMAND_OUTPUT_FAULT;
  new_msg->input_output_fault = parsed_rpt.INPUT_OUTPUT_FAULT;
  new_msg->output_reported_fault = parsed_rpt.OUTPUT_REPORTED_FAULT;
  new_msg->pacmod_fault = parsed_rpt.PACMOD_FAULT;
  new_msg->vehicle_fault = parsed_rpt.VEHICLE_FAULT;
  new_msg->command_timeout = 0;  // dbc3 doesn't have this field
  new_msg->manual_input = parsed_rpt.MANUAL_INPUT;
  new_msg->command = parsed_rpt.COMMANDED_VALUE;
  new_msg->output = parsed_rpt.OUTPUT_VALUE;

  return new_msg;
}

std::shared_ptr<void> Dbc3Api::ParseTurnAuxRpt(const can_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::TurnAuxRpt> new_msg( new pm_msgs::TurnAuxRpt() );

  TURN_AUX_RPT_t parsed_rpt;
  Unpack_TURN_AUX_RPT_pacmod3(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->driver_blinker_bulb_on = parsed_rpt.DRIVER_BLINKER_BULB_ON;
  new_msg->passenger_blinker_bulb_on = parsed_rpt.PASS_BLINKER_BULB_ON;
  new_msg->driver_blinker_bulb_on_avail = parsed_rpt.DRIVER_BLINKER_BULB_ON_IS_VALID;
  new_msg->passenger_blinker_bulb_on_avail = parsed_rpt.PASS_BLINKER_BULB_ON_IS_VALID;

  return new_msg;
}

std::shared_ptr<void> Dbc3Api::ParseVehicleDynamicsRpt(const can_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::VehicleDynamicsRpt> new_msg( new pm_msgs::VehicleDynamicsRpt() );

  VEH_DYNAMICS_RPT_t parsed_rpt;
  Unpack_VEH_DYNAMICS_RPT_pacmod3(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->veh_g_forces = parsed_rpt.VEH_G_FORCES_phys;

  return new_msg;
}

std::shared_ptr<void> Dbc3Api::ParseVehicleSpeedRpt(const can_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::VehicleSpeedRpt> new_msg( new pm_msgs::VehicleSpeedRpt() );

  VEHICLE_SPEED_RPT_t parsed_rpt;
  Unpack_VEHICLE_SPEED_RPT_pacmod3(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->vehicle_speed = parsed_rpt.VEHICLE_SPEED_phys;
  new_msg->vehicle_speed_valid = parsed_rpt.VEHICLE_SPEED_VALID;

  return new_msg;
}

std::shared_ptr<void> Dbc3Api::ParseVinRpt(const can_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::VinRpt> new_msg( new pm_msgs::VinRpt() );

  VIN_RPT_t parsed_rpt;
  Unpack_VIN_RPT_pacmod3(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->mfg_code = parsed_rpt.VEH_MFG_CODE;
  new_msg->model_year_code = parsed_rpt.VEH_MY_CODE;
  new_msg->serial = parsed_rpt.VEH_SERIAL;

  // Following fields not present in dbc3
  new_msg->model_year = 0;
  new_msg->mfg = "";

  return new_msg;
}

std::shared_ptr<void> Dbc3Api::ParseWheelSpeedRpt(const can_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::WheelSpeedRpt> new_msg( new pm_msgs::WheelSpeedRpt() );

  WHEEL_SPEED_RPT_t parsed_rpt;
  Unpack_WHEEL_SPEED_RPT_pacmod3(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->front_left_wheel_speed = parsed_rpt.WHEEL_SPD_FRONT_LEFT_phys;
  new_msg->front_right_wheel_speed = parsed_rpt.WHEEL_SPD_FRONT_RIGHT_phys;
  new_msg->rear_left_wheel_speed = parsed_rpt.WHEEL_SPD_REAR_LEFT_phys;
  new_msg->rear_right_wheel_speed = parsed_rpt.WHEEL_SPD_REAR_RIGHT_phys;

  return new_msg;
}

std::shared_ptr<void> Dbc3Api::ParseWiperAuxRpt(const can_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::WiperAuxRpt> new_msg( new pm_msgs::WiperAuxRpt() );

  WIPER_AUX_RPT_t parsed_rpt;
  Unpack_WIPER_AUX_RPT_pacmod3(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->front_wiping = parsed_rpt.FRONT_WIPING;
  new_msg->front_spraying = parsed_rpt.FRONT_SPRAYING;
  new_msg->rear_wiping = parsed_rpt.REAR_WIPING;
  new_msg->rear_spraying = parsed_rpt.REAR_SPRAYING;
  new_msg->spray_near_empty = parsed_rpt.SPRAY_NEAR_EMPTY;
  new_msg->spray_empty = parsed_rpt.SPRAY_EMPTY;

  new_msg->front_wiping_avail = parsed_rpt.FRONT_WIPING_IS_VALID;
  new_msg->front_spraying_avail = parsed_rpt.FRONT_SPRAYING_IS_VALID;
  new_msg->rear_wiping_avail = parsed_rpt.REAR_WIPING_IS_VALID;
  new_msg->rear_spraying_avail = parsed_rpt.REAR_SPRAYING_IS_VALID;
  new_msg->spray_near_empty_avail = parsed_rpt.SPRAY_NEAR_EMPTY_IS_VALID;
  new_msg->spray_empty_avail = parsed_rpt.SPRAY_EMPTY_IS_VALID;

  return new_msg;
}

std::shared_ptr<void> Dbc3Api::ParseYawRateRpt(const can_msgs::Frame& can_msg)
{
  std::shared_ptr<pm_msgs::YawRateRpt> new_msg( new pm_msgs::YawRateRpt() );

  YAW_RATE_RPT_t parsed_rpt;
  Unpack_YAW_RATE_RPT_pacmod3(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->yaw_rate = parsed_rpt.YAW_RATE_phys;

  return new_msg;
}

// Message Encoding

can_msgs::Frame Dbc3Api::EncodeGlobalCmd(const pm_msgs::GlobalCmd& msg)
{
  can_msgs::Frame packed_frame;

  PrintEncodeError("GlobalCmd");

  return packed_frame;
}

can_msgs::Frame Dbc3Api::EncodeNotificationCmd(const pm_msgs::NotificationCmd& msg)
{
  can_msgs::Frame packed_frame;

  PrintEncodeError("Notification");

  return packed_frame;
}

can_msgs::Frame Dbc3Api::EncodeSteeringCmd(const pm_msgs::SteeringCmd& msg)
{
  can_msgs::Frame packed_frame;

  STEERING_CMD_t unpacked_cmd;
  unpacked_cmd.ENABLE = msg.enable;
  unpacked_cmd.IGNORE_OVERRIDES = msg.ignore_overrides;
  unpacked_cmd.CLEAR_OVERRIDE = msg.clear_override;
  unpacked_cmd.POSITION_phys = msg.command;
  unpacked_cmd.ROTATION_RATE_phys = msg.command;

  uint8_t unused_ide;
  Pack_STEERING_CMD_pacmod3(&unpacked_cmd, static_cast<uint8_t*>(&packed_frame.data[0]), static_cast<uint8_t*>(&packed_frame.dlc), &unused_ide);

  return packed_frame;
}

can_msgs::Frame Dbc3Api::EncodeSystemCmdBool(const pm_msgs::SystemCmdBool& msg)
{
  can_msgs::Frame packed_frame;

  HORN_CMD_t unpacked_cmd;
  unpacked_cmd.ENABLE = msg.enable;
  unpacked_cmd.IGNORE_OVERRIDES = msg.ignore_overrides;
  unpacked_cmd.CLEAR_OVERRIDE = msg.clear_override;
  unpacked_cmd.HORN_CMD = msg.command;

  uint8_t unused_ide;
  Pack_HORN_CMD_pacmod3(&unpacked_cmd, static_cast<uint8_t*>(&packed_frame.data[0]), static_cast<uint8_t*>(&packed_frame.dlc), &unused_ide);

  return packed_frame;
}

can_msgs::Frame Dbc3Api::EncodeSystemCmdFloat(const pm_msgs::SystemCmdFloat& msg)
{
  can_msgs::Frame packed_frame;

  ACCEL_CMD_t unpacked_cmd;
  unpacked_cmd.ENABLE = msg.enable;
  unpacked_cmd.IGNORE_OVERRIDES = msg.ignore_overrides;
  unpacked_cmd.CLEAR_OVERRIDE = msg.clear_override;
  unpacked_cmd.ACCEL_CMD_phys = msg.command;

  uint8_t unused_ide;
  Pack_ACCEL_CMD_pacmod3(&unpacked_cmd, static_cast<uint8_t*>(&packed_frame.data[0]), static_cast<uint8_t*>(&packed_frame.dlc), &unused_ide);

  return packed_frame;
}

can_msgs::Frame Dbc3Api::EncodeSystemCmdInt(const pm_msgs::SystemCmdInt& msg)
{
  can_msgs::Frame packed_frame;

  SHIFT_CMD_t unpacked_cmd;
  unpacked_cmd.ENABLE = msg.enable;
  unpacked_cmd.IGNORE_OVERRIDES = msg.ignore_overrides;
  unpacked_cmd.CLEAR_OVERRIDE = msg.clear_override;
  unpacked_cmd.SHIFT_CMD = msg.command;

  uint8_t unused_ide;
  Pack_SHIFT_CMD_pacmod3(&unpacked_cmd, static_cast<uint8_t*>(&packed_frame.data[0]), static_cast<uint8_t*>(&packed_frame.dlc), &unused_ide);

  return packed_frame;
}



}  // namespace pacmod3

