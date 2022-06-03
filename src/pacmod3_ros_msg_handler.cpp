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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <vector>
#include <string>
#include <memory>

#include "pacmod3/pacmod3_ros_msg_handler.hpp"

namespace lc = rclcpp_lifecycle;

namespace pacmod3
{

LockedData::LockedData(unsigned char data_length)
: _data(),
  _data_mut()
{
  _data.assign(data_length, 0);
}

std::vector<unsigned char> LockedData::getData() const
{
  std::lock_guard<std::mutex> lck(_data_mut);
  return _data;
}

void LockedData::setData(std::vector<unsigned char> && new_data)
{
  std::lock_guard<std::mutex> lck(_data_mut);
  _data = new_data;
}

void Pacmod3TxRosMsgHandler::fillAndPublish(
  const uint32_t & can_id,
  const std::string & frame_id,
  const std::shared_ptr<lc::ManagedEntityInterface> & pub,
  const std::shared_ptr<Pacmod3TxMsg> & parser_class)
{
  if (can_id == HornRptMsg::CAN_ID ||
    can_id == ParkingBrakeRptMsg::CAN_ID ||
    can_id == HazardLightRptMsg::CAN_ID)
  {
    pacmod3_msgs::msg::SystemRptBool new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod3_msgs::msg::SystemRptBool>>(pub);

    fillSystemRptBool(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  } else if (can_id == CruiseControlButtonsRptMsg::CAN_ID ||  // NOLINT
    can_id == DashControlsLeftRptMsg::CAN_ID || can_id == DashControlsRightRptMsg::CAN_ID ||
    can_id == TurnSignalRptMsg::CAN_ID || can_id == ShiftRptMsg::CAN_ID ||
    can_id == HeadlightRptMsg::CAN_ID || can_id == MediaControlsRptMsg::CAN_ID ||
    can_id == WiperRptMsg::CAN_ID || can_id == RearPassDoorRptMsg::CAN_ID)
  {
    pacmod3_msgs::msg::SystemRptInt new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod3_msgs::msg::SystemRptInt>>(pub);

    fillSystemRptInt(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  } else if (can_id == AccelRptMsg::CAN_ID || can_id == BrakeRptMsg::CAN_ID ||  // NOLINT
    can_id == SteeringRptMsg::CAN_ID)
  {
    pacmod3_msgs::msg::SystemRptFloat new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod3_msgs::msg::SystemRptFloat>>(pub);

    fillSystemRptFloat(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  } else if (can_id == GlobalRptMsg::CAN_ID) {
    pacmod3_msgs::msg::GlobalRpt new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod3_msgs::msg::GlobalRpt>>(pub);

    fillGlobalRpt(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  } else if (can_id == ComponentRptMsg::CAN_ID) {
    pacmod3_msgs::msg::ComponentRpt new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod3_msgs::msg::ComponentRpt>>(pub);

    fillComponentRpt(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  } else if (can_id == BrakeMotorRpt1Msg::CAN_ID || can_id == SteeringMotorRpt1Msg::CAN_ID) {
    pacmod3_msgs::msg::MotorRpt1 new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod3_msgs::msg::MotorRpt1>>(pub);

    fillMotorRpt1(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  } else if (can_id == BrakeMotorRpt2Msg::CAN_ID || can_id == SteeringMotorRpt2Msg::CAN_ID) {
    pacmod3_msgs::msg::MotorRpt2 new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod3_msgs::msg::MotorRpt2>>(pub);

    fillMotorRpt2(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  } else if (can_id == BrakeMotorRpt3Msg::CAN_ID || can_id == SteeringMotorRpt3Msg::CAN_ID) {
    pacmod3_msgs::msg::MotorRpt3 new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod3_msgs::msg::MotorRpt3>>(pub);

    fillMotorRpt3(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  } else if (can_id == AccelAuxRptMsg::CAN_ID) {
    pacmod3_msgs::msg::AccelAuxRpt new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod3_msgs::msg::AccelAuxRpt>>(pub);

    fillAccelAuxRpt(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  } else if (can_id == BrakeAuxRptMsg::CAN_ID) {
    pacmod3_msgs::msg::BrakeAuxRpt new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod3_msgs::msg::BrakeAuxRpt>>(pub);

    fillBrakeAuxRpt(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  } else if (can_id == DateTimeRptMsg::CAN_ID) {
    pacmod3_msgs::msg::DateTimeRpt new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod3_msgs::msg::DateTimeRpt>>(pub);

    fillDateTimeRpt(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  } else if (can_id == DoorRptMsg::CAN_ID) {
    pacmod3_msgs::msg::DoorRpt new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod3_msgs::msg::DoorRpt>>(pub);

    fillDoorRpt(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  } else if (can_id == HeadlightAuxRptMsg::CAN_ID) {
    pacmod3_msgs::msg::HeadlightAuxRpt new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod3_msgs::msg::HeadlightAuxRpt>>(pub);

    fillHeadlightAuxRpt(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  } else if (can_id == InteriorLightsRptMsg::CAN_ID) {
    pacmod3_msgs::msg::InteriorLightsRpt new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod3_msgs::msg::InteriorLightsRpt>>(pub);

    fillInteriorLightsRpt(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  } else if (can_id == LatLonHeadingRptMsg::CAN_ID) {
    pacmod3_msgs::msg::LatLonHeadingRpt new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod3_msgs::msg::LatLonHeadingRpt>>(pub);

    fillLatLonHeadingRpt(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  } else if (can_id == OccupancyRptMsg::CAN_ID) {
    pacmod3_msgs::msg::OccupancyRpt new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod3_msgs::msg::OccupancyRpt>>(pub);

    fillOccupancyRpt(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  } else if (can_id == RearLightsRptMsg::CAN_ID) {
    pacmod3_msgs::msg::RearLightsRpt new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod3_msgs::msg::RearLightsRpt>>(pub);

    fillRearLightsRpt(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  } else if (can_id == ShiftAuxRptMsg::CAN_ID) {
    pacmod3_msgs::msg::ShiftAuxRpt new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod3_msgs::msg::ShiftAuxRpt>>(pub);

    fillShiftAuxRpt(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  } else if (can_id == SteeringAuxRptMsg::CAN_ID) {
    pacmod3_msgs::msg::SteeringAuxRpt new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod3_msgs::msg::SteeringAuxRpt>>(pub);

    fillSteeringAuxRpt(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  } else if (can_id == TurnAuxRptMsg::CAN_ID) {
    pacmod3_msgs::msg::TurnAuxRpt new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod3_msgs::msg::TurnAuxRpt>>(pub);

    fillTurnAuxRpt(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  } else if (can_id == YawRateRptMsg::CAN_ID) {
    pacmod3_msgs::msg::YawRateRpt new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod3_msgs::msg::YawRateRpt>>(pub);

    fillYawRateRpt(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  } else if (can_id == VehicleSpeedRptMsg::CAN_ID) {
    pacmod3_msgs::msg::VehicleSpeedRpt new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod3_msgs::msg::VehicleSpeedRpt>>(pub);

    fillVehicleSpeedRpt(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  } else if (can_id == VinRptMsg::CAN_ID) {
    pacmod3_msgs::msg::VinRpt new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod3_msgs::msg::VinRpt>>(pub);

    fillVinRpt(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  } else if (can_id == WheelSpeedRptMsg::CAN_ID) {
    pacmod3_msgs::msg::WheelSpeedRpt new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod3_msgs::msg::WheelSpeedRpt>>(pub);

    fillWheelSpeedRpt(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  } else if (can_id == WiperAuxRptMsg::CAN_ID) {
    pacmod3_msgs::msg::WiperAuxRpt new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod3_msgs::msg::WiperAuxRpt>>(pub);

    fillWiperAuxRpt(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  } else if (can_id == DetectedObjectRptMsg::CAN_ID) {
    pacmod3_msgs::msg::DetectedObjectRpt new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod3_msgs::msg::DetectedObjectRpt>>(pub);

    fillDetectedObjectRpt(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  } else if (can_id == VehicleDynamicsRptMsg::CAN_ID) {
    pacmod3_msgs::msg::VehicleDynamicsRpt new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod3_msgs::msg::VehicleDynamicsRpt>>(pub);

    fillVehicleDynamicsRpt(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  }
}

// Report messages
void Pacmod3TxRosMsgHandler::fillSystemRptBool(
  const std::shared_ptr<Pacmod3TxMsg> & parser_class,
  pacmod3_msgs::msg::SystemRptBool * const new_msg,
  const std::string & frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<SystemRptBoolMsg>(parser_class);

  new_msg->enabled = dc_parser->enabled;
  new_msg->override_active = dc_parser->override_active;
  new_msg->command_output_fault = dc_parser->command_output_fault;
  new_msg->input_output_fault = dc_parser->input_output_fault;
  new_msg->output_reported_fault = dc_parser->output_reported_fault;
  new_msg->pacmod_fault = dc_parser->pacmod_fault;
  new_msg->vehicle_fault = dc_parser->vehicle_fault;

  new_msg->manual_input = dc_parser->manual_input;
  new_msg->command = dc_parser->command;
  new_msg->output = dc_parser->output;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = rclcpp::Clock().now();
}

void Pacmod3TxRosMsgHandler::fillSystemRptInt(
  const std::shared_ptr<Pacmod3TxMsg> & parser_class,
  pacmod3_msgs::msg::SystemRptInt * const new_msg,
  const std::string & frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<SystemRptIntMsg>(parser_class);

  new_msg->enabled = dc_parser->enabled;
  new_msg->override_active = dc_parser->override_active;
  new_msg->command_output_fault = dc_parser->command_output_fault;
  new_msg->input_output_fault = dc_parser->input_output_fault;
  new_msg->output_reported_fault = dc_parser->output_reported_fault;
  new_msg->pacmod_fault = dc_parser->pacmod_fault;
  new_msg->vehicle_fault = dc_parser->vehicle_fault;

  new_msg->manual_input = dc_parser->manual_input;
  new_msg->command = dc_parser->command;
  new_msg->output = dc_parser->output;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = rclcpp::Clock().now();
}

void Pacmod3TxRosMsgHandler::fillSystemRptFloat(
  const std::shared_ptr<Pacmod3TxMsg> & parser_class,
  pacmod3_msgs::msg::SystemRptFloat * const new_msg,
  const std::string & frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<SystemRptFloatMsg>(parser_class);

  new_msg->enabled = dc_parser->enabled;
  new_msg->override_active = dc_parser->override_active;
  new_msg->command_output_fault = dc_parser->command_output_fault;
  new_msg->input_output_fault = dc_parser->input_output_fault;
  new_msg->output_reported_fault = dc_parser->output_reported_fault;
  new_msg->pacmod_fault = dc_parser->pacmod_fault;
  new_msg->vehicle_fault = dc_parser->vehicle_fault;

  new_msg->manual_input = dc_parser->manual_input;
  new_msg->command = dc_parser->command;
  new_msg->output = dc_parser->output;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = rclcpp::Clock().now();
}

void Pacmod3TxRosMsgHandler::fillGlobalRpt(
  const std::shared_ptr<Pacmod3TxMsg> & parser_class,
  pacmod3_msgs::msg::GlobalRpt * const new_msg,
  const std::string & frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<GlobalRptMsg>(parser_class);

  new_msg->enabled = dc_parser->enabled;
  new_msg->override_active = dc_parser->override_active;
  new_msg->pacmod_sys_fault_active = dc_parser->system_fault_active;
  new_msg->config_fault_active = dc_parser->config_fault_active;
  new_msg->user_can_timeout = dc_parser->user_can_timeout;
  new_msg->steering_can_timeout = dc_parser->steering_can_timeout;
  new_msg->brake_can_timeout = dc_parser->brake_can_timeout;
  new_msg->subsystem_can_timeout = dc_parser->subsystem_can_timeout;
  new_msg->vehicle_can_timeout = dc_parser->vehicle_can_timeout;
  new_msg->user_can_read_errors = dc_parser->user_can_read_errors;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = rclcpp::Clock().now();
}

void Pacmod3TxRosMsgHandler::fillComponentRpt(
  const std::shared_ptr<Pacmod3TxMsg> & parser_class,
  pacmod3_msgs::msg::ComponentRpt * const new_msg,
  const std::string & frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<ComponentRptMsg>(parser_class);

  new_msg->component_type = static_cast<unsigned char>(dc_parser->component_type);
  // TODO(icolwell-as): build converter from old CAN message to new ROS message format
  // (which matches new CAN format)
  // new_msg->component_func = static_cast<unsigned char>(dc_parser->component_func);
  new_msg->counter = dc_parser->counter;
  new_msg->complement = dc_parser->complement;
  new_msg->config_fault = dc_parser->config_fault;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = rclcpp::Clock().now();
}

void Pacmod3TxRosMsgHandler::fillAccelAuxRpt(
  const std::shared_ptr<Pacmod3TxMsg> & parser_class,
  pacmod3_msgs::msg::AccelAuxRpt * const new_msg,
  const std::string & frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<AccelAuxRptMsg>(parser_class);

  new_msg->operator_interaction = dc_parser->user_interaction;
  new_msg->operator_interaction_avail = dc_parser->user_interaction_is_valid;

  // Fields omitted since they are no longer in use by latest dbc:
  // raw_pedal_pos
  // raw_pedal_force
  // raw_pedal_pos_is_valid
  // raw_pedal_force_is_valid

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = rclcpp::Clock().now();
}

void Pacmod3TxRosMsgHandler::fillBrakeAuxRpt(
  const std::shared_ptr<Pacmod3TxMsg> & parser_class,
  pacmod3_msgs::msg::BrakeAuxRpt * const new_msg,
  const std::string & frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<BrakeAuxRptMsg>(parser_class);

  new_msg->brake_pressure = dc_parser->raw_brake_pressure;
  new_msg->operator_interaction = dc_parser->user_interaction;
  new_msg->brake_on_off = dc_parser->brake_on_off;
  new_msg->operator_interaction_avail = dc_parser->user_interaction_is_valid;
  new_msg->brake_on_off_avail = dc_parser->brake_on_off_is_valid;

  // Fields omitted since they are no longer in use by latest dbc:
  // raw_pedal_pos
  // raw_pedal_force
  // raw_pedal_pos_is_valid
  // raw_pedal_force_is_valid

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = rclcpp::Clock().now();
}

void Pacmod3TxRosMsgHandler::fillDateTimeRpt(
  const std::shared_ptr<Pacmod3TxMsg> & parser_class,
  pacmod3_msgs::msg::DateTimeRpt * const new_msg,
  const std::string & frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<DateTimeRptMsg>(parser_class);

  new_msg->year = dc_parser->year;
  new_msg->month = dc_parser->month;
  new_msg->day = dc_parser->day;
  new_msg->hour = dc_parser->hour;
  new_msg->minute = dc_parser->minute;
  new_msg->second = dc_parser->second;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = rclcpp::Clock().now();
}

void Pacmod3TxRosMsgHandler::fillDetectedObjectRpt(
  const std::shared_ptr<Pacmod3TxMsg> & parser_class,
  pacmod3_msgs::msg::DetectedObjectRpt * const new_msg,
  const std::string & frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<DetectedObjectRptMsg>(parser_class);

  new_msg->front_object_distance_low_res = dc_parser->front_object_distance_low_res;
  new_msg->front_object_distance_high_res = dc_parser->front_object_distance_high_res;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = rclcpp::Clock().now();
}
void Pacmod3TxRosMsgHandler::fillDoorRpt(
  const std::shared_ptr<Pacmod3TxMsg> & parser_class,
  pacmod3_msgs::msg::DoorRpt * const new_msg,
  const std::string & frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<DoorRptMsg>(parser_class);

  new_msg->driver_door_open = dc_parser->driver_door_open;
  new_msg->driver_door_open_avail = dc_parser->driver_door_open_is_valid;
  new_msg->passenger_door_open = dc_parser->passenger_door_open;
  new_msg->passenger_door_open_avail = dc_parser->passenger_door_open_is_valid;
  new_msg->rear_driver_door_open = dc_parser->rear_driver_door_open;
  new_msg->rear_driver_door_open_avail = dc_parser->rear_driver_door_open_is_valid;
  new_msg->rear_passenger_door_open = dc_parser->rear_passenger_door_open;
  new_msg->rear_passenger_door_open_avail = dc_parser->rear_passenger_door_open_is_valid;
  new_msg->hood_open = dc_parser->hood_open;
  new_msg->hood_open_avail = dc_parser->hood_open_is_valid;
  new_msg->trunk_open = dc_parser->trunk_open;
  new_msg->trunk_open_avail = dc_parser->trunk_open_is_valid;
  new_msg->fuel_door_open = dc_parser->fuel_door_open;
  new_msg->fuel_door_open_avail = dc_parser->fuel_door_open_is_valid;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = rclcpp::Clock().now();
}

void Pacmod3TxRosMsgHandler::fillHeadlightAuxRpt(
  const std::shared_ptr<Pacmod3TxMsg> & parser_class,
  pacmod3_msgs::msg::HeadlightAuxRpt * const new_msg,
  const std::string & frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<HeadlightAuxRptMsg>(parser_class);

  new_msg->headlights_on = dc_parser->headlights_on;
  new_msg->headlights_on_bright = dc_parser->headlights_on_bright;
  new_msg->fog_lights_on = dc_parser->fog_lights_on;
  new_msg->headlights_mode = dc_parser->headlights_mode;
  new_msg->headlights_on_avail = dc_parser->headlights_on_is_valid;
  new_msg->headlights_on_bright_avail = dc_parser->headlights_on_bright_is_valid;
  new_msg->fog_lights_on_avail = dc_parser->fog_lights_on_is_valid;
  new_msg->headlights_mode_avail = dc_parser->headlights_mode_is_valid;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = rclcpp::Clock().now();
}

void Pacmod3TxRosMsgHandler::fillInteriorLightsRpt(
  const std::shared_ptr<Pacmod3TxMsg> & parser_class,
  pacmod3_msgs::msg::InteriorLightsRpt * const new_msg,
  const std::string & frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<InteriorLightsRptMsg>(parser_class);

  new_msg->front_dome_lights_on = dc_parser->front_dome_lights_on;
  new_msg->front_dome_lights_on_avail = dc_parser->front_dome_lights_on_is_valid;
  new_msg->rear_dome_lights_on = dc_parser->rear_dome_lights_on;
  new_msg->rear_dome_lights_on_avail = dc_parser->rear_dome_lights_on_is_valid;
  new_msg->mood_lights_on = dc_parser->mood_lights_on;
  new_msg->mood_lights_on_avail = dc_parser->mood_lights_on_is_valid;
  new_msg->dim_level = static_cast<unsigned char>(dc_parser->dim_level);
  new_msg->dim_level_avail = dc_parser->dim_level_is_valid;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = rclcpp::Clock().now();
}

void Pacmod3TxRosMsgHandler::fillLatLonHeadingRpt(
  const std::shared_ptr<Pacmod3TxMsg> & parser_class,
  pacmod3_msgs::msg::LatLonHeadingRpt * const new_msg,
  const std::string & frame_id)
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
  new_msg->header.stamp = rclcpp::Clock().now();
}

void Pacmod3TxRosMsgHandler::fillMotorRpt1(
  const std::shared_ptr<Pacmod3TxMsg> & parser_class,
  pacmod3_msgs::msg::MotorRpt1 * const new_msg,
  const std::string & frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<MotorRpt1Msg>(parser_class);

  new_msg->current = dc_parser->current;
  new_msg->position = dc_parser->position;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = rclcpp::Clock().now();
}

void Pacmod3TxRosMsgHandler::fillMotorRpt2(
  const std::shared_ptr<Pacmod3TxMsg> & parser_class,
  pacmod3_msgs::msg::MotorRpt2 * const new_msg,
  const std::string & frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<MotorRpt2Msg>(parser_class);

  new_msg->encoder_temp = dc_parser->encoder_temp;
  new_msg->motor_temp = dc_parser->motor_temp;
  new_msg->angular_speed = dc_parser->velocity;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = rclcpp::Clock().now();
}

void Pacmod3TxRosMsgHandler::fillMotorRpt3(
  const std::shared_ptr<Pacmod3TxMsg> & parser_class,
  pacmod3_msgs::msg::MotorRpt3 * const new_msg,
  const std::string & frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<MotorRpt3Msg>(parser_class);

  new_msg->torque_output = dc_parser->torque_output;
  new_msg->torque_input = dc_parser->torque_input;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = rclcpp::Clock().now();
}

void Pacmod3TxRosMsgHandler::fillOccupancyRpt(
  const std::shared_ptr<Pacmod3TxMsg> & parser_class,
  pacmod3_msgs::msg::OccupancyRpt * const new_msg,
  const std::string & frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<OccupancyRptMsg>(parser_class);

  new_msg->driver_seat_occupied = dc_parser->driver_seat_occupied;
  new_msg->driver_seat_occupied_avail = dc_parser->driver_seat_occupied_is_valid;
  new_msg->passenger_seat_occupied = dc_parser->passenger_seat_occupied;
  new_msg->passenger_seat_occupied_avail = dc_parser->passenger_seat_occupied_is_valid;
  new_msg->rear_seat_occupied = dc_parser->rear_seat_occupied;
  new_msg->rear_seat_occupied_avail = dc_parser->rear_seat_occupied_is_valid;
  new_msg->driver_seatbelt_buckled = dc_parser->driver_seatbelt_buckled;
  new_msg->driver_seatbelt_buckled_avail = dc_parser->driver_seatbelt_buckled_is_valid;
  new_msg->passenger_seatbelt_buckled = dc_parser->passenger_seatbelt_buckled;
  new_msg->passenger_seatbelt_buckled_avail = dc_parser->passenger_seatbelt_buckled_is_valid;
  new_msg->driver_rear_seatbelt_buckled = dc_parser->rear_seatbelt_buckled;
  new_msg->driver_rear_seatbelt_buckled_avail = dc_parser->rear_seatbelt_buckled_is_valid;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = rclcpp::Clock().now();
}

void Pacmod3TxRosMsgHandler::fillRearLightsRpt(
  const std::shared_ptr<Pacmod3TxMsg> & parser_class,
  pacmod3_msgs::msg::RearLightsRpt * const new_msg,
  const std::string & frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<RearLightsRptMsg>(parser_class);

  new_msg->brake_lights_on = dc_parser->brake_lights_on;
  new_msg->brake_lights_on_avail = dc_parser->brake_lights_on_is_valid;
  new_msg->reverse_lights_on = dc_parser->reverse_lights_on;
  new_msg->reverse_lights_on_avail = dc_parser->reverse_lights_on_is_valid;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = rclcpp::Clock().now();
}

void Pacmod3TxRosMsgHandler::fillShiftAuxRpt(
  const std::shared_ptr<Pacmod3TxMsg> & parser_class,
  pacmod3_msgs::msg::ShiftAuxRpt * const new_msg,
  const std::string & frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<ShiftAuxRptMsg>(parser_class);

  new_msg->between_gears = dc_parser->between_gears;
  new_msg->stay_in_neutral_mode = dc_parser->stay_in_neutral_mode;
  new_msg->brake_interlock_active = dc_parser->brake_interlock_active;
  new_msg->speed_interlock_active = dc_parser->speed_interlock_active;
  new_msg->between_gears_avail = dc_parser->between_gears_is_valid;
  new_msg->stay_in_neutral_mode_avail = dc_parser->stay_in_neutral_mode_is_valid;
  new_msg->brake_interlock_active_avail = dc_parser->brake_interlock_active_is_valid;
  new_msg->speed_interlock_active_avail = dc_parser->speed_interlock_active_is_valid;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = rclcpp::Clock().now();
}

void Pacmod3TxRosMsgHandler::fillSteeringAuxRpt(
  const std::shared_ptr<Pacmod3TxMsg> & parser_class,
  pacmod3_msgs::msg::SteeringAuxRpt * const new_msg,
  const std::string & frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<SteeringAuxRptMsg>(parser_class);

  new_msg->steering_torque = dc_parser->raw_torque;
  new_msg->rotation_rate = dc_parser->rotation_rate;
  new_msg->operator_interaction = dc_parser->user_interaction;
  new_msg->steering_torque_avail = dc_parser->raw_torque_is_valid;
  new_msg->rotation_rate_avail = dc_parser->rotation_rate_is_valid;
  new_msg->operator_interaction_avail = dc_parser->user_interaction_is_valid;

  // Fields omitted since they are no longer in use by latest dbc:
  // raw_position
  // raw_position_is_valid

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = rclcpp::Clock().now();
}

void Pacmod3TxRosMsgHandler::fillTurnAuxRpt(
  const std::shared_ptr<Pacmod3TxMsg> & parser_class,
  pacmod3_msgs::msg::TurnAuxRpt * const new_msg,
  const std::string & frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<TurnAuxRptMsg>(parser_class);

  new_msg->driver_blinker_bulb_on = dc_parser->driver_blinker_bulb_on;
  new_msg->passenger_blinker_bulb_on = dc_parser->passenger_blinker_bulb_on;
  new_msg->driver_blinker_bulb_on_avail = dc_parser->driver_blinker_bulb_on_is_valid;
  new_msg->passenger_blinker_bulb_on_avail = dc_parser->passenger_blinker_bulb_on_is_valid;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = rclcpp::Clock().now();
}

void Pacmod3TxRosMsgHandler::fillVehicleDynamicsRpt(
  const std::shared_ptr<Pacmod3TxMsg> & parser_class,
  pacmod3_msgs::msg::VehicleDynamicsRpt * const new_msg,
  const std::string & frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<VehicleDynamicsRptMsg>(parser_class);

  new_msg->veh_g_forces = dc_parser->g_forces;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = rclcpp::Clock().now();
}

void Pacmod3TxRosMsgHandler::fillVehicleSpeedRpt(
  const std::shared_ptr<Pacmod3TxMsg> & parser_class,
  pacmod3_msgs::msg::VehicleSpeedRpt * const new_msg,
  const std::string & frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<VehicleSpeedRptMsg>(parser_class);

  new_msg->vehicle_speed = dc_parser->vehicle_speed;
  new_msg->vehicle_speed_valid = dc_parser->vehicle_speed_valid;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = rclcpp::Clock().now();
}

void Pacmod3TxRosMsgHandler::fillVinRpt(
  const std::shared_ptr<Pacmod3TxMsg> & parser_class,
  pacmod3_msgs::msg::VinRpt * const new_msg,
  const std::string & frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<VinRptMsg>(parser_class);

  new_msg->mfg_code = dc_parser->mfg_code;
  new_msg->mfg = dc_parser->mfg;
  new_msg->model_year_code = dc_parser->model_year_code;
  new_msg->model_year = dc_parser->model_year;
  new_msg->serial = dc_parser->serial;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = rclcpp::Clock().now();
}

void Pacmod3TxRosMsgHandler::fillWheelSpeedRpt(
  const std::shared_ptr<Pacmod3TxMsg> & parser_class,
  pacmod3_msgs::msg::WheelSpeedRpt * const new_msg,
  const std::string & frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<WheelSpeedRptMsg>(parser_class);

  new_msg->front_left_wheel_speed = dc_parser->front_left_wheel_speed;
  new_msg->front_right_wheel_speed = dc_parser->front_right_wheel_speed;
  new_msg->rear_left_wheel_speed = dc_parser->rear_left_wheel_speed;
  new_msg->rear_right_wheel_speed = dc_parser->rear_right_wheel_speed;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = rclcpp::Clock().now();
}

void Pacmod3TxRosMsgHandler::fillWiperAuxRpt(
  const std::shared_ptr<Pacmod3TxMsg> & parser_class,
  pacmod3_msgs::msg::WiperAuxRpt * const new_msg,
  const std::string & frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<WiperAuxRptMsg>(parser_class);

  new_msg->front_wiping = dc_parser->front_wiping;
  new_msg->front_spraying = dc_parser->front_spraying;
  new_msg->rear_wiping = dc_parser->rear_wiping;
  new_msg->rear_spraying = dc_parser->rear_spraying;
  new_msg->spray_near_empty = dc_parser->spray_near_empty;
  new_msg->spray_empty = dc_parser->spray_empty;
  new_msg->front_wiping_avail = dc_parser->front_wiping_is_valid;
  new_msg->front_spraying_avail = dc_parser->front_spraying_is_valid;
  new_msg->rear_wiping_avail = dc_parser->rear_wiping_is_valid;
  new_msg->rear_spraying_avail = dc_parser->rear_spraying_is_valid;
  new_msg->spray_near_empty_avail = dc_parser->spray_near_empty_is_valid;
  new_msg->spray_empty_avail = dc_parser->spray_empty_is_valid;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = rclcpp::Clock().now();
}

void Pacmod3TxRosMsgHandler::fillYawRateRpt(
  const std::shared_ptr<Pacmod3TxMsg> & parser_class,
  pacmod3_msgs::msg::YawRateRpt * const new_msg,
  const std::string & frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<YawRateRptMsg>(parser_class);

  new_msg->yaw_rate = dc_parser->yaw_rate;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = rclcpp::Clock().now();
}


// Command messages
std::vector<uint8_t> Pacmod3RxRosMsgHandler::unpackAndEncode(
  const uint32_t & can_id,
  const pacmod3_msgs::msg::SystemCmdBool::SharedPtr & msg)
{
  // Note: The clear_faults field has been removed in later DBC versions.
  // It is omitted here.

  // TODO(icolwell-as): should clear_faults be added back in here from global_cmd?

  if (can_id == HornCmdMsg::CAN_ID) {
    HornCmdMsg encoder;
    encoder.encode(
      msg->enable,
      msg->ignore_overrides,
      msg->clear_override,
      false,
      msg->command);
    return encoder.data;
  } else if (can_id == ParkingBrakeCmdMsg::CAN_ID) {
    ParkingBrakeCmdMsg encoder;
    encoder.encode(
      msg->enable,
      msg->ignore_overrides,
      msg->clear_override,
      false,
      msg->command);
    return encoder.data;
  } else if (can_id == HazardLightCmdMsg::CAN_ID) {
    HazardLightCmdMsg encoder;
    encoder.encode(
      msg->enable,
      msg->ignore_overrides,
      msg->clear_override,
      false,
      msg->command);
    return encoder.data;
  } else {
    std::vector<uint8_t> bad_id;
    bad_id.assign(8, 0);
    return bad_id;
  }
}

std::vector<uint8_t> Pacmod3RxRosMsgHandler::unpackAndEncode(
  const uint32_t & can_id,
  const pacmod3_msgs::msg::SystemCmdFloat::SharedPtr & msg)
{
  // Note: The clear_faults field has been removed in later DBC versions.
  // It is omitted here.

  // TODO(icolwell-as): should clear_faults be added back in here from global_cmd?

  if (can_id == AccelCmdMsg::CAN_ID) {
    AccelCmdMsg encoder;
    encoder.encode(
      msg->enable,
      msg->ignore_overrides,
      msg->clear_override,
      false,
      msg->command);
    return encoder.data;
  } else if (can_id == BrakeCmdMsg::CAN_ID) {
    BrakeCmdMsg encoder;
    encoder.encode(
      msg->enable,
      msg->ignore_overrides,
      msg->clear_override,
      false,
      msg->command);
    return encoder.data;
  } else {
    std::vector<uint8_t> bad_id;
    bad_id.assign(8, 0);
    return bad_id;
  }
}

std::vector<uint8_t> Pacmod3RxRosMsgHandler::unpackAndEncode(
  const uint32_t & can_id,
  const pacmod3_msgs::msg::SystemCmdInt::SharedPtr & msg)
{
  // Note: The clear_faults field has been removed in later DBC versions.
  // It is omitted here.

  // TODO(icolwell-as): should clear_faults be added back in here from global_cmd?

  if (can_id == CruiseControlButtonsCmdMsg::CAN_ID) {
    CruiseControlButtonsCmdMsg encoder;
    encoder.encode(
      msg->enable,
      msg->ignore_overrides,
      msg->clear_override,
      false,
      msg->command);
    return encoder.data;
  } else if (can_id == DashControlsLeftCmdMsg::CAN_ID) {
    DashControlsLeftCmdMsg encoder;
    encoder.encode(
      msg->enable,
      msg->ignore_overrides,
      msg->clear_override,
      false,
      msg->command);
    return encoder.data;
  } else if (can_id == DashControlsRightCmdMsg::CAN_ID) {
    DashControlsRightCmdMsg encoder;
    encoder.encode(
      msg->enable,
      msg->ignore_overrides,
      msg->clear_override,
      false,
      msg->command);
    return encoder.data;
  } else if (can_id == HeadlightCmdMsg::CAN_ID) {
    HeadlightCmdMsg encoder;
    encoder.encode(
      msg->enable,
      msg->ignore_overrides,
      msg->clear_override,
      false,
      msg->command);
    return encoder.data;
  } else if (can_id == MediaControlsCmdMsg::CAN_ID) {
    MediaControlsCmdMsg encoder;
    encoder.encode(
      msg->enable,
      msg->ignore_overrides,
      msg->clear_override,
      false,
      msg->command);
    return encoder.data;
  } else if (can_id == RearPassDoorCmdMsg::CAN_ID) {
    RearPassDoorCmdMsg encoder;
    encoder.encode(
      msg->enable,
      msg->ignore_overrides,
      msg->clear_override,
      false,
      msg->command);
    return encoder.data;
  } else if (can_id == ShiftCmdMsg::CAN_ID) {
    ShiftCmdMsg encoder;
    encoder.encode(
      msg->enable,
      msg->ignore_overrides,
      msg->clear_override,
      false,
      msg->command);
    return encoder.data;
  } else if (can_id == TurnSignalCmdMsg::CAN_ID) {
    TurnSignalCmdMsg encoder;
    encoder.encode(
      msg->enable,
      msg->ignore_overrides,
      msg->clear_override,
      false,
      msg->command);
    return encoder.data;
  } else if (can_id == WiperCmdMsg::CAN_ID) {
    WiperCmdMsg encoder;
    encoder.encode(
      msg->enable,
      msg->ignore_overrides,
      msg->clear_override,
      false,
      msg->command);
    return encoder.data;
  } else {
    std::vector<uint8_t> bad_id;
    bad_id.assign(8, 0);
    return bad_id;
  }
}

std::vector<uint8_t> Pacmod3RxRosMsgHandler::unpackAndEncode(
  const uint32_t & can_id,
  const pacmod3_msgs::msg::SteeringCmd::SharedPtr & msg)
{
  // Note: The clear_faults field has been removed in later DBC versions.
  // It is omitted here.

  // TODO(icolwell-as): should clear_faults be added back in here from global_cmd?

  if (can_id == SteeringCmdMsg::CAN_ID) {
    SteeringCmdMsg encoder;
    encoder.encode(
      msg->enable,
      msg->ignore_overrides,
      msg->clear_override,
      false,
      msg->command,
      msg->rotation_rate);
    return encoder.data;
  } else {
    std::vector<uint8_t> bad_id;
    bad_id.assign(8, 0);
    return bad_id;
  }
}

}  // namespace pacmod3
