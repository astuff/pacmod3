/*
* Unpublished Copyright (c) 2009-2017 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include <pacmod3_ros_msg_handler.h>

using namespace AS::Drivers::PACMod3;

LockedData::LockedData() :
  _data(),
  _data_mut()
{
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

void Pacmod3TxRosMsgHandler::fillAndPublish(const int64_t& can_id,
                                            std::string frame_id,
                                            ros::Publisher& pub,
                                            std::shared_ptr<Pacmod3TxMsg>& parser_class)
{
  if (can_id == HornRptMsg::CAN_ID ||
      can_id == ParkingBrakeRptMsg::CAN_ID)
  {
    pacmod_msgs::SystemRptBool new_msg;
    fillSystemRptBool(parser_class, new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == CruiseControlButtonsRptMsg::CAN_ID ||
      can_id == DashControlsLeftRptMsg::CAN_ID ||
      can_id == DashControlsRightRptMsg::CAN_ID ||
      can_id == TurnSignalRptMsg::CAN_ID ||
      can_id == ShiftRptMsg::CAN_ID ||
      can_id == HeadlightRptMsg::CAN_ID ||
      can_id == MediaControlsRptMsg::CAN_ID ||
      can_id == WiperRptMsg::CAN_ID)
  {
    pacmod_msgs::SystemRptInt new_msg;
    fillSystemRptInt(parser_class, new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == AccelRptMsg::CAN_ID ||
           can_id == BrakeRptMsg::CAN_ID ||
           can_id == SteerRptMsg::CAN_ID)
  {
    pacmod_msgs::SystemRptFloat new_msg;
    fillSystemRptFloat(parser_class, new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == GlobalRptMsg::CAN_ID)
  {
    pacmod_msgs::GlobalRpt new_msg;
    fillGlobalRpt(parser_class, new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == ComponentRptMsg::CAN_ID)
  {
    pacmod_msgs::ComponentRpt new_msg;
    fillComponentRpt(parser_class, new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == BrakeMotorRpt1Msg::CAN_ID ||
           can_id == SteerMotorRpt1Msg::CAN_ID)
  {
    pacmod_msgs::MotorRpt1 new_msg;
    fillMotorRpt1(parser_class, new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == BrakeMotorRpt2Msg::CAN_ID ||
           can_id == SteerMotorRpt2Msg::CAN_ID)
  {
    pacmod_msgs::MotorRpt2 new_msg;
    fillMotorRpt2(parser_class, new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == BrakeMotorRpt3Msg::CAN_ID ||
           can_id == SteerMotorRpt3Msg::CAN_ID)
  {
    pacmod_msgs::MotorRpt3 new_msg;
    fillMotorRpt3(parser_class, new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == AccelAuxRptMsg::CAN_ID)
  {
    pacmod_msgs::AccelAuxRpt new_msg;
    fillAccelAuxRpt(parser_class, new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == BrakeAuxRptMsg::CAN_ID)
  {
    pacmod_msgs::BrakeAuxRpt new_msg;
    fillBrakeAuxRpt(parser_class, new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == DateTimeRptMsg::CAN_ID)
  {
    pacmod_msgs::DateTimeRpt new_msg;
    fillDateTimeRpt(parser_class, new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == DoorRptMsg::CAN_ID)
  {
    pacmod_msgs::DoorRpt new_msg;
    fillDoorRpt(parser_class, new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == HeadlightAuxRptMsg::CAN_ID)
  {
    pacmod_msgs::HeadlightAuxRpt new_msg;
    fillHeadlightAuxRpt(parser_class, new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == InteriorLightsRptMsg::CAN_ID)
  {
    pacmod_msgs::InteriorLightsRpt new_msg;
    fillInteriorLightsRpt(parser_class, new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == LatLonHeadingRptMsg::CAN_ID)
  {
    pacmod_msgs::LatLonHeadingRpt new_msg;
    fillLatLonHeadingRpt(parser_class, new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == OccupancyRptMsg::CAN_ID)
  {
    pacmod_msgs::OccupancyRpt new_msg;
    fillOccupancyRpt(parser_class, new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == RearLightsRptMsg::CAN_ID)
  {
    pacmod_msgs::RearLightsRpt new_msg;
    fillRearLightsRpt(parser_class, new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == ShiftAuxRptMsg::CAN_ID)
  {
    pacmod_msgs::ShiftAuxRpt new_msg;
    fillShiftAuxRpt(parser_class, new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == SteerAuxRptMsg::CAN_ID)
  {
    pacmod_msgs::SteerAuxRpt new_msg;
    fillSteerAuxRpt(parser_class, new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == SteeringPIDRpt1Msg::CAN_ID)
  {
    pacmod_msgs::SteeringPIDRpt1 new_msg;
    fillSteeringPIDRpt1(parser_class, new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == SteeringPIDRpt2Msg::CAN_ID)
  {
    pacmod_msgs::SteeringPIDRpt2 new_msg;
    fillSteeringPIDRpt2(parser_class, new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == SteeringPIDRpt3Msg::CAN_ID)
  {
    pacmod_msgs::SteeringPIDRpt3 new_msg;
    fillSteeringPIDRpt3(parser_class, new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == SteeringPIDRpt4Msg::CAN_ID)
  {
    pacmod_msgs::SteeringPIDRpt4 new_msg;
    fillSteeringPIDRpt4(parser_class, new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == TurnAuxRptMsg::CAN_ID)
  {
    pacmod_msgs::TurnAuxRpt new_msg;
    fillTurnAuxRpt(parser_class, new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == YawRateRptMsg::CAN_ID)
  {
    pacmod_msgs::YawRateRpt new_msg;
    fillYawRateRpt(parser_class, new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == VehicleSpeedRptMsg::CAN_ID)
  {
    pacmod_msgs::VehicleSpeedRpt new_msg;
    fillVehicleSpeedRpt(parser_class, new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == VinRptMsg::CAN_ID)
  {
    pacmod_msgs::VinRpt new_msg;
    fillVinRpt(parser_class, new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == WheelSpeedRptMsg::CAN_ID)
  {
    pacmod_msgs::WheelSpeedRpt new_msg;
    fillWheelSpeedRpt(parser_class, new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == WiperAuxRptMsg::CAN_ID)
  {
    pacmod_msgs::WiperAuxRpt new_msg;
    fillWiperAuxRpt(parser_class, new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == DetectedObjectRptMsg::CAN_ID)
  {
    pacmod_msgs::DetectedObjectRpt new_msg;
    fillDetectedObjectRpt(parser_class, new_msg, frame_id);
    pub.publish(new_msg);
  }      
  else if (can_id == VehicleSpecificRpt1Msg::CAN_ID)
  {
    pacmod_msgs::VehicleSpecificRpt1 new_msg;
    fillVehicleSpecificRpt1(parser_class, new_msg, frame_id);
    pub.publish(new_msg);
  }  
  else if (can_id == VehicleDynamicsRptMsg::CAN_ID)
  {
    pacmod_msgs::VehicleDynamicsRpt new_msg;
    fillVehicleDynamicsRpt(parser_class, new_msg, frame_id);
    pub.publish(new_msg);
  }   
}

// Report messages
void Pacmod3TxRosMsgHandler::fillSystemRptBool(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::SystemRptBool& new_msg, std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<SystemRptBoolMsg>(parser_class);

  new_msg.enabled = dc_parser->enabled;
  new_msg.override_active = dc_parser->override_active;
  new_msg.command_output_fault = dc_parser->command_output_fault;
  new_msg.input_output_fault = dc_parser->input_output_fault;
  new_msg.output_reported_fault = dc_parser->output_reported_fault;
  new_msg.pacmod_fault = dc_parser->pacmod_fault;
  new_msg.vehicle_fault = dc_parser->vehicle_fault;

  new_msg.manual_input = dc_parser->manual_input;
  new_msg.command = dc_parser->command;
  new_msg.output = dc_parser->output;

  new_msg.header.frame_id = frame_id;
  new_msg.header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillSystemRptInt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::SystemRptInt& new_msg, std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<SystemRptIntMsg>(parser_class);

  new_msg.enabled = dc_parser->enabled;
  new_msg.override_active = dc_parser->override_active;
  new_msg.command_output_fault = dc_parser->command_output_fault;
  new_msg.input_output_fault = dc_parser->input_output_fault;
  new_msg.output_reported_fault = dc_parser->output_reported_fault;
  new_msg.pacmod_fault = dc_parser->pacmod_fault;
  new_msg.vehicle_fault = dc_parser->vehicle_fault;

	new_msg.manual_input = dc_parser->manual_input;
	new_msg.command = dc_parser->command;
	new_msg.output = dc_parser->output;

  new_msg.header.frame_id = frame_id;
  new_msg.header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillSystemRptFloat(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::SystemRptFloat& new_msg, std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<SystemRptFloatMsg>(parser_class);

  new_msg.enabled = dc_parser->enabled;
  new_msg.override_active = dc_parser->override_active;
  new_msg.command_output_fault = dc_parser->command_output_fault;
  new_msg.input_output_fault = dc_parser->input_output_fault;
  new_msg.output_reported_fault = dc_parser->output_reported_fault;
  new_msg.pacmod_fault = dc_parser->pacmod_fault;
  new_msg.vehicle_fault = dc_parser->vehicle_fault;

  new_msg.manual_input = dc_parser->manual_input;
  new_msg.command = dc_parser->command;
  new_msg.output = dc_parser->output;

  new_msg.header.frame_id = frame_id;
  new_msg.header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillGlobalRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::GlobalRpt& new_msg, std::string frame_id)
{
	auto dc_parser = std::dynamic_pointer_cast<GlobalRptMsg>(parser_class);

	new_msg.enabled = dc_parser->enabled;
	new_msg.override_active = dc_parser->override_active;
  new_msg.fault_active = dc_parser->fault_active;
  new_msg.config_fault_active = dc_parser->config_fault_active;
	new_msg.user_can_timeout = dc_parser->user_can_timeout;
	new_msg.steering_can_timeout = dc_parser->steering_can_timeout;
	new_msg.brake_can_timeout = dc_parser->brake_can_timeout;
  new_msg.subsystem_can_timeout = dc_parser->subsystem_can_timeout;
	new_msg.vehicle_can_timeout = dc_parser->vehicle_can_timeout;
	new_msg.user_can_read_errors = dc_parser->user_can_read_errors;

  new_msg.header.frame_id = frame_id;
	new_msg.header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillComponentRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::ComponentRpt& new_msg, std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<ComponentRptMsg>(parser_class);

  new_msg.component_type = dc_parser->component_type;
  new_msg.component_func = dc_parser->component_func;
  new_msg.counter = dc_parser->counter;
  new_msg.complement = dc_parser->complement;
  new_msg.config_fault = dc_parser->config_fault;

  new_msg.header.frame_id = frame_id;
  new_msg.header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillAccelAuxRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::AccelAuxRpt& new_msg, std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<AccelAuxRptMsg>(parser_class);

  new_msg.raw_pedal_pos = dc_parser->raw_pedal_pos;
  new_msg.raw_pedal_force = dc_parser->raw_pedal_force;
  new_msg.user_interaction = dc_parser->user_interaction;
  new_msg.raw_pedal_pos_is_valid = dc_parser->raw_pedal_pos_is_valid;
  new_msg.raw_pedal_force_is_valid = dc_parser->raw_pedal_force_is_valid;
  new_msg.user_interaction_is_valid = dc_parser->user_interaction_is_valid;

  new_msg.header.frame_id = frame_id;
  new_msg.header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillBrakeAuxRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::BrakeAuxRpt& new_msg, std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<BrakeAuxRptMsg>(parser_class);

  new_msg.raw_pedal_pos = dc_parser->raw_pedal_pos;
  new_msg.raw_pedal_force = dc_parser->raw_pedal_force;
  new_msg.raw_brake_pressure = dc_parser->raw_brake_pressure;
  new_msg.user_interaction = dc_parser->user_interaction;
  new_msg.brake_on_off = dc_parser->brake_on_off;
  new_msg.raw_pedal_pos_is_valid = dc_parser->raw_pedal_pos_is_valid;
  new_msg.raw_pedal_force_is_valid = dc_parser->raw_pedal_force_is_valid;
  new_msg.user_interaction_is_valid = dc_parser->user_interaction_is_valid;
  new_msg.brake_on_off_is_valid = dc_parser->brake_on_off_is_valid;

  new_msg.header.frame_id = frame_id;
  new_msg.header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillDateTimeRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::DateTimeRpt& new_msg, std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<DateTimeRptMsg>(parser_class);

	new_msg.year = dc_parser->year;
	new_msg.month = dc_parser->month;
	new_msg.day = dc_parser->day;
	new_msg.hour = dc_parser->hour;
	new_msg.minute = dc_parser->minute;
	new_msg.second = dc_parser->second;

  new_msg.header.frame_id = frame_id;
  new_msg.header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillDetectedObjectRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::DetectedObjectRpt& new_msg, std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<DetectedObjectRptMsg>(parser_class);

  new_msg.front_object_distance_low_res = dc_parser->front_object_distance_low_res;
  new_msg.front_object_distance_high_res = dc_parser->front_object_distance_high_res;

  new_msg.header.frame_id = frame_id;
  new_msg.header.stamp = ros::Time::now();
}
void Pacmod3TxRosMsgHandler::fillDoorRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::DoorRpt& new_msg, std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<DoorRptMsg>(parser_class);

  new_msg.driver_door_open = dc_parser->driver_door_open;
  new_msg.driver_door_open_is_valid = dc_parser->driver_door_open_is_valid;
  new_msg.passenger_door_open = dc_parser->passenger_door_open;
  new_msg.passenger_door_open_is_valid = dc_parser->passenger_door_open_is_valid;
  new_msg.rear_driver_door_open = dc_parser->rear_driver_door_open;
  new_msg.rear_driver_door_open_is_valid = dc_parser->rear_driver_door_open_is_valid;
  new_msg.rear_passenger_door_open = dc_parser->rear_passenger_door_open;
  new_msg.rear_passenger_door_open_is_valid = dc_parser->rear_passenger_door_open_is_valid;
  new_msg.hood_open = dc_parser->hood_open;
  new_msg.hood_open_is_valid = dc_parser->hood_open_is_valid;
  new_msg.trunk_open = dc_parser->trunk_open;
  new_msg.trunk_open_is_valid = dc_parser->trunk_open_is_valid;
  new_msg.fuel_door_open = dc_parser->fuel_door_open;
  new_msg.fuel_door_open_is_valid = dc_parser->fuel_door_open_is_valid;

  new_msg.header.frame_id = frame_id;
  new_msg.header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillHeadlightAuxRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::HeadlightAuxRpt& new_msg, std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<HeadlightAuxRptMsg>(parser_class);

  new_msg.headlights_on = dc_parser->headlights_on;
  new_msg.headlights_on_bright = dc_parser->headlights_on_bright;
  new_msg.fog_lights_on = dc_parser->fog_lights_on;
  new_msg.headlights_mode = dc_parser->headlights_mode;
  new_msg.headlights_on_is_valid = dc_parser->headlights_on_is_valid;
  new_msg.headlights_on_bright_is_valid = dc_parser->headlights_on_bright_is_valid;
  new_msg.fog_lights_on_is_valid = dc_parser->fog_lights_on_is_valid;
  new_msg.headlights_mode_is_valid = dc_parser->headlights_mode_is_valid;

  new_msg.header.frame_id = frame_id;
  new_msg.header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillInteriorLightsRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::InteriorLightsRpt& new_msg, std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<InteriorLightsRptMsg>(parser_class);

  new_msg.front_dome_lights_on = dc_parser->front_dome_lights_on;
  new_msg.front_dome_lights_on_is_valid = dc_parser->front_dome_lights_on_is_valid;
  new_msg.rear_dome_lights_on = dc_parser->rear_dome_lights_on;
  new_msg.rear_dome_lights_on_is_valid = dc_parser->rear_dome_lights_on_is_valid;
  new_msg.mood_lights_on = dc_parser->mood_lights_on;
  new_msg.mood_lights_on_is_valid = dc_parser->mood_lights_on_is_valid;
  new_msg.dim_level = dc_parser->dim_level;
  new_msg.dim_level_is_valid = dc_parser->dim_level_is_valid;

  new_msg.header.frame_id = frame_id;
  new_msg.header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillLatLonHeadingRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::LatLonHeadingRpt& new_msg, std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<LatLonHeadingRptMsg>(parser_class);

	new_msg.latitude_degrees = dc_parser->latitude_degrees;
	new_msg.latitude_minutes = dc_parser->latitude_minutes;
	new_msg.latitude_seconds = dc_parser->latitude_seconds;
	new_msg.longitude_degrees = dc_parser->longitude_degrees;
	new_msg.longitude_minutes = dc_parser->longitude_minutes;
	new_msg.longitude_seconds = dc_parser->longitude_seconds;
	new_msg.heading = dc_parser->heading;

  new_msg.header.frame_id = frame_id;
  new_msg.header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillMotorRpt1(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::MotorRpt1& new_msg, std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<MotorRpt1Msg>(parser_class);

	new_msg.current = dc_parser->current;
	new_msg.position = dc_parser->position;

  new_msg.header.frame_id = frame_id;
  new_msg.header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillMotorRpt2(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::MotorRpt2& new_msg, std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<MotorRpt2Msg>(parser_class);

	new_msg.encoder_temp = dc_parser->encoder_temp;
	new_msg.motor_temp = dc_parser->motor_temp;
	new_msg.angular_velocity = dc_parser->velocity;

  new_msg.header.frame_id = frame_id;
  new_msg.header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillMotorRpt3(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::MotorRpt3& new_msg, std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<MotorRpt3Msg>(parser_class);

	new_msg.torque_output = dc_parser->torque_output;
	new_msg.torque_input = dc_parser->torque_input;

  new_msg.header.frame_id = frame_id;
  new_msg.header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillOccupancyRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::OccupancyRpt& new_msg, std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<OccupancyRptMsg>(parser_class);

  new_msg.driver_seat_occupied = dc_parser->driver_seat_occupied;
  new_msg.driver_seat_occupied_is_valid = dc_parser->driver_seat_occupied_is_valid;
  new_msg.passenger_seat_occupied = dc_parser->passenger_seat_occupied;
  new_msg.passenger_seat_occupied_is_valid = dc_parser->passenger_seat_occupied_is_valid;
  new_msg.rear_seat_occupied = dc_parser->rear_seat_occupied;
  new_msg.rear_seat_occupied_is_valid = dc_parser->rear_seat_occupied_is_valid;
  new_msg.driver_seatbelt_buckled = dc_parser->driver_seatbelt_buckled;
  new_msg.driver_seatbelt_buckled_is_valid = dc_parser->driver_seatbelt_buckled_is_valid;
  new_msg.passenger_seatbelt_buckled = dc_parser->passenger_seatbelt_buckled;
  new_msg.passenger_seatbelt_buckled_is_valid = dc_parser->passenger_seatbelt_buckled_is_valid;
  new_msg.rear_seatbelt_buckled = dc_parser->rear_seatbelt_buckled;
  new_msg.rear_seatbelt_buckled_is_valid = dc_parser->rear_seatbelt_buckled_is_valid;

  new_msg.header.frame_id = frame_id;
  new_msg.header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillRearLightsRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::RearLightsRpt& new_msg, std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<RearLightsRptMsg>(parser_class);

  new_msg.brake_lights_on = dc_parser->brake_lights_on;
  new_msg.brake_lights_on_is_valid = dc_parser->brake_lights_on_is_valid;
  new_msg.reverse_lights_on = dc_parser->reverse_lights_on;
  new_msg.reverse_lights_on_is_valid = dc_parser->reverse_lights_on_is_valid;

  new_msg.header.frame_id = frame_id;
  new_msg.header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillShiftAuxRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::ShiftAuxRpt& new_msg, std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<ShiftAuxRptMsg>(parser_class);

  new_msg.between_gears = dc_parser->between_gears;
  new_msg.stay_in_neutral_mode = dc_parser->stay_in_neutral_mode;
  new_msg.brake_interlock_active = dc_parser->brake_interlock_active;
  new_msg.speed_interlock_active = dc_parser->speed_interlock_active;
  new_msg.between_gears_is_valid = dc_parser->between_gears_is_valid;
  new_msg.stay_in_neutral_mode_is_valid = dc_parser->stay_in_neutral_mode_is_valid;
  new_msg.brake_interlock_active_is_valid = dc_parser->brake_interlock_active_is_valid;
  new_msg.speed_interlock_active_is_valid = dc_parser->speed_interlock_active_is_valid;

  new_msg.header.frame_id = frame_id;
  new_msg.header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillSteerAuxRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::SteerAuxRpt& new_msg, std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<SteerAuxRptMsg>(parser_class);

  new_msg.raw_position = dc_parser->raw_position;
  new_msg.raw_torque = dc_parser->raw_torque;
  new_msg.rotation_rate = dc_parser->rotation_rate;
  new_msg.user_interaction = dc_parser->user_interaction;
  new_msg.raw_position_is_valid = dc_parser->raw_position_is_valid;
  new_msg.raw_torque_is_valid = dc_parser->raw_torque_is_valid;
  new_msg.rotation_rate_is_valid = dc_parser->rotation_rate_is_valid;
  new_msg.user_interaction_is_valid = dc_parser->user_interaction_is_valid;

  new_msg.header.frame_id = frame_id;
  new_msg.header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillSteeringPIDRpt1(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::SteeringPIDRpt1& new_msg, std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<SteeringPIDRpt1Msg>(parser_class);

	new_msg.dt = dc_parser->dt;
	new_msg.Kp = dc_parser->Kp;
	new_msg.Ki = dc_parser->Ki;
	new_msg.Kd = dc_parser->Kd;

  new_msg.header.frame_id = frame_id;
  new_msg.header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillSteeringPIDRpt2(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::SteeringPIDRpt2& new_msg, std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<SteeringPIDRpt2Msg>(parser_class);

	new_msg.P_term = dc_parser->P_term;
	new_msg.I_term = dc_parser->I_term;
	new_msg.D_term = dc_parser->D_term;
	new_msg.all_terms = dc_parser->all_terms;

  new_msg.header.frame_id = frame_id;
  new_msg.header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillSteeringPIDRpt3(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::SteeringPIDRpt3& new_msg, std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<SteeringPIDRpt3Msg>(parser_class);

	new_msg.new_torque = dc_parser->new_torque;
	new_msg.str_angle_desired = dc_parser->str_angle_desired;
	new_msg.str_angle_actual = dc_parser->str_angle_actual;
	new_msg.error = dc_parser->error;

  new_msg.header.frame_id = frame_id;
  new_msg.header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillSteeringPIDRpt4(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::SteeringPIDRpt4& new_msg, std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<SteeringPIDRpt4Msg>(parser_class);

	new_msg.angular_velocity = dc_parser->angular_velocity;
	new_msg.angular_acceleration = dc_parser->angular_acceleration;

  new_msg.header.frame_id = frame_id;
  new_msg.header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillTurnAuxRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::TurnAuxRpt& new_msg, std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<TurnAuxRptMsg>(parser_class);

  new_msg.driver_blinker_bulb_on = dc_parser->driver_blinker_bulb_on;
  new_msg.passenger_blinker_bulb_on = dc_parser->passenger_blinker_bulb_on;
  new_msg.driver_blinker_bulb_on_is_valid = dc_parser->driver_blinker_bulb_on_is_valid;
  new_msg.passenger_blinker_bulb_on_is_valid = dc_parser->passenger_blinker_bulb_on_is_valid;

  new_msg.header.frame_id = frame_id;
  new_msg.header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillVehicleDynamicsRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::VehicleDynamicsRpt& new_msg, std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<VehicleDynamicsRptMsg>(parser_class);

  new_msg.g_forces = dc_parser->g_forces;
  new_msg.brake_torque = dc_parser->brake_torque;

  new_msg.header.frame_id = frame_id;
  new_msg.header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillVehicleSpecificRpt1(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::VehicleSpecificRpt1& new_msg, std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<VehicleSpecificRpt1Msg>(parser_class);

  new_msg.shift_pos_1 = dc_parser->shift_pos_1;
  new_msg.shift_pos_2 = dc_parser->shift_pos_2;

  new_msg.header.frame_id = frame_id;
  new_msg.header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillVehicleSpeedRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::VehicleSpeedRpt& new_msg, std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<VehicleSpeedRptMsg>(parser_class);

	new_msg.vehicle_speed = dc_parser->vehicle_speed;
	new_msg.vehicle_speed_valid = dc_parser->vehicle_speed_valid;
	new_msg.vehicle_speed_raw[0] = dc_parser->vehicle_speed_raw[0];
	new_msg.vehicle_speed_raw[1] = dc_parser->vehicle_speed_raw[1];

  new_msg.header.frame_id = frame_id;
  new_msg.header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillVinRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::VinRpt& new_msg, std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<VinRptMsg>(parser_class);

	new_msg.mfg_code = dc_parser->mfg_code;
	new_msg.mfg = dc_parser->mfg;
	new_msg.model_year_code = dc_parser->model_year_code;
	new_msg.model_year = dc_parser->model_year;
	new_msg.serial = dc_parser->serial;

  new_msg.header.frame_id = frame_id;
  new_msg.header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillWheelSpeedRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::WheelSpeedRpt& new_msg, std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<WheelSpeedRptMsg>(parser_class);

	new_msg.front_left_wheel_speed = dc_parser->front_left_wheel_speed;
	new_msg.front_right_wheel_speed = dc_parser->front_right_wheel_speed;
	new_msg.rear_left_wheel_speed = dc_parser->rear_left_wheel_speed;
	new_msg.rear_right_wheel_speed = dc_parser->rear_right_wheel_speed;

  new_msg.header.frame_id = frame_id;
  new_msg.header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillWiperAuxRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::WiperAuxRpt& new_msg, std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<WiperAuxRptMsg>(parser_class);

  new_msg.front_wiping = dc_parser->front_wiping;
  new_msg.front_spraying = dc_parser->front_spraying;
  new_msg.rear_wiping = dc_parser->rear_wiping;
  new_msg.rear_spraying = dc_parser->rear_spraying;
  new_msg.spray_near_empty = dc_parser->spray_near_empty;
  new_msg.spray_empty = dc_parser->spray_empty;
  new_msg.front_wiping_is_valid = dc_parser->front_wiping_is_valid;
  new_msg.front_spraying_is_valid = dc_parser->front_spraying_is_valid;
  new_msg.rear_wiping_is_valid = dc_parser->rear_wiping_is_valid;
  new_msg.rear_spraying_is_valid = dc_parser->rear_spraying_is_valid;
  new_msg.spray_near_empty_is_valid = dc_parser->spray_near_empty_is_valid;
  new_msg.spray_empty_is_valid = dc_parser->spray_empty_is_valid;

  new_msg.header.frame_id = frame_id;
  new_msg.header.stamp = ros::Time::now();
}

void Pacmod3TxRosMsgHandler::fillYawRateRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::YawRateRpt& new_msg, std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<YawRateRptMsg>(parser_class);

	new_msg.yaw_rate = dc_parser->yaw_rate;

  new_msg.header.frame_id = frame_id;
  new_msg.header.stamp = ros::Time::now();
}


// Command messages
std::vector<uint8_t> Pacmod3RxRosMsgHandler::unpackAndEncode(const int64_t& can_id, const pacmod_msgs::SystemCmdBool::ConstPtr& msg)
{
  if (can_id == HornCmdMsg::CAN_ID)
  {
    HornCmdMsg encoder;
    encoder.encode(msg->enable,
                   msg->ignore_overrides,
                   msg->clear_override,
                   msg->clear_faults,
                   msg->command);
    return encoder.data;
  }
  else if (can_id == ParkingBrakeCmdMsg::CAN_ID)
  {
    ParkingBrakeCmdMsg encoder;
    encoder.encode(msg->enable,
                   msg->ignore_overrides,
                   msg->clear_override,
                   msg->clear_faults,
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

std::vector<uint8_t> Pacmod3RxRosMsgHandler::unpackAndEncode(const int64_t& can_id, const pacmod_msgs::SystemCmdFloat::ConstPtr& msg)
{
	if (can_id == AccelCmdMsg::CAN_ID)
	{
    AccelCmdMsg encoder;
    encoder.encode(msg->enable,
                   msg->ignore_overrides,
                   msg->clear_override,
                   msg->clear_faults,
                   msg->command);
    return encoder.data;
	}
	else if (can_id == BrakeCmdMsg::CAN_ID)
	{
    BrakeCmdMsg encoder;
    encoder.encode(msg->enable,
                   msg->ignore_overrides,
                   msg->clear_override,
                   msg->clear_faults,
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

std::vector<uint8_t> Pacmod3RxRosMsgHandler::unpackAndEncode(const int64_t& can_id, const pacmod_msgs::SystemCmdInt::ConstPtr& msg)
{
  if (can_id == CruiseControlButtonsCmdMsg::CAN_ID)
  {
    CruiseControlButtonsCmdMsg encoder;
    encoder.encode(msg->enable,
                   msg->ignore_overrides,
                   msg->clear_override,
                   msg->clear_faults,
                   msg->command);
    return encoder.data;
  }
  else if (can_id == DashControlsLeftCmdMsg::CAN_ID)
  {
    DashControlsLeftCmdMsg encoder;
    encoder.encode(msg->enable,
                   msg->ignore_overrides,
                   msg->clear_override,
                   msg->clear_faults,
                   msg->command);
    return encoder.data;
  }
  else if (can_id == DashControlsRightCmdMsg::CAN_ID)
  {
    DashControlsRightCmdMsg encoder;
    encoder.encode(msg->enable,
                   msg->ignore_overrides,
                   msg->clear_override,
                   msg->clear_faults,
                   msg->command);
    return encoder.data;
  }
  else if (can_id == HeadlightCmdMsg::CAN_ID)
	{
    HeadlightCmdMsg encoder;
    encoder.encode(msg->enable,
                   msg->ignore_overrides,
                   msg->clear_override,
                   msg->clear_faults,
                   msg->command);
    return encoder.data;
	}
  else if (can_id == MediaControlsCmdMsg::CAN_ID)
  {
    MediaControlsCmdMsg encoder;
    encoder.encode(msg->enable,
                   msg->ignore_overrides,
                   msg->clear_override,
                   msg->clear_faults,
                   msg->command);
    return encoder.data;
  }
	else if (can_id == ShiftCmdMsg::CAN_ID)
	{
    ShiftCmdMsg encoder;
    encoder.encode(msg->enable,
                   msg->ignore_overrides,
                   msg->clear_override,
                   msg->clear_faults,
                   msg->command);
    return encoder.data;
	}
  else if (can_id == TurnSignalCmdMsg::CAN_ID)
	{
    TurnSignalCmdMsg encoder;
    encoder.encode(msg->enable,
                   msg->ignore_overrides,
                   msg->clear_override,
                   msg->clear_faults,
                   msg->command);
    return encoder.data;
	}
	else if (can_id == WiperCmdMsg::CAN_ID)
	{
    WiperCmdMsg encoder;
    encoder.encode(msg->enable,
                   msg->ignore_overrides,
                   msg->clear_override,
                   msg->clear_faults,
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

std::vector<uint8_t> Pacmod3RxRosMsgHandler::unpackAndEncode(const int64_t& can_id, const pacmod_msgs::SteerSystemCmd::ConstPtr& msg)
{
  if (can_id == SteerCmdMsg::CAN_ID)
  {
    SteerCmdMsg encoder;
    encoder.encode(msg->enable,
                   msg->ignore_overrides,
                   msg->clear_override,
                   msg->clear_faults,
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
