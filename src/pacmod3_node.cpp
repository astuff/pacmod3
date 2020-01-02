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

#include <lifecycle_msgs/msg/state.hpp>

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "pacmod3/pacmod3_node.hpp"

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using namespace std::chrono_literals;

namespace pacmod3
{

constexpr std::chrono::milliseconds PACMod3Node::SEND_CMD_INTERVAL;
constexpr std::chrono::milliseconds PACMod3Node::INTER_MSG_PAUSE;

PACMod3Node::PACMod3Node(rclcpp::NodeOptions options)
: lc::LifecycleNode("pacmod3_driver", options)
{
  std::string vehicle_type_string = this->declare_parameter("vehicle_type", "POLARIS_GEM");
  frame_id_ = this->declare_parameter("frame_id", "pacmod");

  if (vehicle_type_string == "INTERNATIONAL_PROSTAR_122") {
    vehicle_type_ = VehicleType::INTERNATIONAL_PROSTAR_122;
  } else if (vehicle_type_string == "JUPITER_SPIRIT") {
    vehicle_type_ = VehicleType::JUPITER_SPIRIT;
  } else if (vehicle_type_string == "LEXUS_RX_450H") {
    vehicle_type_ = VehicleType::LEXUS_RX_450H;
  } else if (vehicle_type_string == "POLARIS_GEM") {
    vehicle_type_ = VehicleType::POLARIS_GEM;
  } else if (vehicle_type_string == "POLARIS_RANGER") {
    vehicle_type_ = VehicleType::POLARIS_RANGER;
  } else if (vehicle_type_string == "VEHICLE_4") {
    vehicle_type_ = VehicleType::VEHICLE_4;
  } else if (vehicle_type_string == "VEHICLE_5") {
    vehicle_type_ = VehicleType::VEHICLE_5;
  } else if (vehicle_type_string == "VEHICLE_6") {
    vehicle_type_ = VehicleType::VEHICLE_6;
  } else {
    vehicle_type_string = "POLARIS_GEM";
    vehicle_type_ = VehicleType::POLARIS_GEM;
    RCLCPP_WARN(
      this->get_logger(),
      "An invalid vehicle type was entered. Defaulting to POLARIS_GEM.");
  }

  RCLCPP_INFO(this->get_logger(), "Got vehicle type: %s", vehicle_type_string.c_str());
  RCLCPP_INFO(this->get_logger(), "Got frame id: %s", frame_id_.c_str());
}

PACMod3Node::~PACMod3Node()
{
  if (pub_thread_ && pub_thread_->joinable()) {
    pub_thread_->join();
  }
}

LNI::CallbackReturn PACMod3Node::on_configure(const lc::State & state)
{
  (void)state;

  pub_can_rx_ = this->create_publisher<can_msgs::msg::Frame>("can_rx", 100);

  can_pubs_[GlobalRptMsg::CAN_ID] =
    this->create_publisher<pacmod_msgs::msg::GlobalRpt>("parsed_tx/global_rpt", 20);
  can_pubs_[ComponentRptMsg::CAN_ID] =
    this->create_publisher<pacmod_msgs::msg::ComponentRpt>("parsed_tx/component_rpt", 20);
  can_pubs_[AccelRptMsg::CAN_ID] =
    this->create_publisher<pacmod_msgs::msg::SystemRptFloat>("parsed_tx/accel_rpt", 20);
  can_pubs_[BrakeRptMsg::CAN_ID] =
    this->create_publisher<pacmod_msgs::msg::SystemRptFloat>("parsed_tx/brake_rpt", 20);
  can_pubs_[ShiftRptMsg::CAN_ID] =
    this->create_publisher<pacmod_msgs::msg::SystemRptInt>("parsed_tx/shift_rpt", 20);
  can_pubs_[SteerRptMsg::CAN_ID] =
    this->create_publisher<pacmod_msgs::msg::SystemRptFloat>("parsed_tx/steer_rpt", 20);
  can_pubs_[TurnSignalRptMsg::CAN_ID] =
    this->create_publisher<pacmod_msgs::msg::SystemRptInt>("parsed_tx/turn_rpt", 20);
  can_pubs_[AccelAuxRptMsg::CAN_ID] =
    this->create_publisher<pacmod_msgs::msg::AccelAuxRpt>("parsed_tx/accel_aux_rpt", 20);
  can_pubs_[BrakeAuxRptMsg::CAN_ID] =
    this->create_publisher<pacmod_msgs::msg::BrakeAuxRpt>("parsed_tx/brake_aux_rpt", 20);
  can_pubs_[ShiftAuxRptMsg::CAN_ID] =
    this->create_publisher<pacmod_msgs::msg::ShiftAuxRpt>("parsed_tx/shift_aux_rpt", 20);
  can_pubs_[SteerAuxRptMsg::CAN_ID] =
    this->create_publisher<pacmod_msgs::msg::SteerAuxRpt>("parsed_tx/steer_aux_rpt", 20);
  can_pubs_[TurnAuxRptMsg::CAN_ID] =
    this->create_publisher<pacmod_msgs::msg::TurnAuxRpt>("parsed_tx/turn_aux_rpt", 20);
  can_pubs_[VehicleSpeedRptMsg::CAN_ID] =
    this->create_publisher<pacmod_msgs::msg::VehicleSpeedRpt>("parsed_tx/vehicle_speed_rpt", 20);
  can_pubs_[VinRptMsg::CAN_ID] =
    this->create_publisher<pacmod_msgs::msg::VinRpt>("parsed_tx/vin_rpt", 5);

  pub_enabled_ = this->create_publisher<std_msgs::msg::Bool>(
    "as_tx/enabled", rclcpp::QoS(1).transient_local());
  pub_vehicle_speed_ms_ = this->create_publisher<std_msgs::msg::Float64>(
    "as_tx/vehicle_speed", 20);
  pub_all_system_statuses_ = this->create_publisher<pacmod_msgs::msg::AllSystemStatuses>(
    "as_tx/all_system_statuses", 20);

  sub_can_tx_ = this->create_subscription<can_msgs::msg::Frame>(
    "can_tx", 100, std::bind(&PACMod3Node::callback_can_tx, this, std::placeholders::_1));

  can_subs_[AccelCmdMsg::CAN_ID] = std::make_pair(
    this->create_subscription<pacmod_msgs::msg::SystemCmdFloat>(
      "as_rx/accel_cmd", 20,
      std::bind(&PACMod3Node::callback_accel_cmd, this, std::placeholders::_1)),
    std::shared_ptr<LockedData>(new LockedData(AccelCmdMsg::DATA_LENGTH)));

  can_subs_[BrakeCmdMsg::CAN_ID] = std::make_pair(
    this->create_subscription<pacmod_msgs::msg::SystemCmdFloat>(
      "as_rx/brake_cmd", 20,
      std::bind(&PACMod3Node::callback_brake_cmd, this, std::placeholders::_1)),
    std::shared_ptr<LockedData>(new LockedData(BrakeCmdMsg::DATA_LENGTH)));

  can_subs_[ShiftCmdMsg::CAN_ID] = std::make_pair(
    this->create_subscription<pacmod_msgs::msg::SystemCmdInt>(
      "as_rx/shift_cmd", 20,
      std::bind(&PACMod3Node::callback_shift_cmd, this, std::placeholders::_1)),
    std::shared_ptr<LockedData>(new LockedData(ShiftCmdMsg::DATA_LENGTH)));

  can_subs_[SteerCmdMsg::CAN_ID] = std::make_pair(
    this->create_subscription<pacmod_msgs::msg::SteerSystemCmd>(
      "as_rx/steer_cmd", 20,
      std::bind(&PACMod3Node::callback_steer_cmd, this, std::placeholders::_1)),
    std::shared_ptr<LockedData>(new LockedData(SteerCmdMsg::DATA_LENGTH)));

  can_subs_[TurnSignalCmdMsg::CAN_ID] = std::make_pair(
    this->create_subscription<pacmod_msgs::msg::SystemCmdInt>(
      "as_rx/turn_cmd", 20,
      std::bind(&PACMod3Node::callback_turn_cmd, this, std::placeholders::_1)),
    std::shared_ptr<LockedData>(new LockedData(TurnSignalCmdMsg::DATA_LENGTH)));

  // Need to initialize TurnSignalCmdMsg with non-0 starting value
  TurnSignalCmdMsg turn_encoder;
  turn_encoder.encode(false, false, false, false, pacmod_msgs::msg::SystemCmdInt::TURN_NONE);
  can_subs_[TurnSignalCmdMsg::CAN_ID].second->setData(std::move(turn_encoder.data));

  if (
    vehicle_type_ == VehicleType::POLARIS_GEM ||
    vehicle_type_ == VehicleType::POLARIS_RANGER ||
    vehicle_type_ == VehicleType::INTERNATIONAL_PROSTAR_122 ||
    vehicle_type_ == VehicleType::FREIGHTLINER_CASCADIA)
  {
    can_pubs_[BrakeMotorRpt1Msg::CAN_ID] =
      this->create_publisher<pacmod_msgs::msg::MotorRpt1>("parsed_tx/brake_rpt_detail_1", 20);
    can_pubs_[BrakeMotorRpt2Msg::CAN_ID] =
      this->create_publisher<pacmod_msgs::msg::MotorRpt2>("parsed_tx/brake_rpt_detail_2", 20);
    can_pubs_[BrakeMotorRpt3Msg::CAN_ID] =
      this->create_publisher<pacmod_msgs::msg::MotorRpt3>("parsed_tx/brake_rpt_detail_3", 20);
    can_pubs_[SteerMotorRpt1Msg::CAN_ID] =
      this->create_publisher<pacmod_msgs::msg::MotorRpt1>("parsed_tx/steering_rpt_detail_1", 20);
    can_pubs_[SteerMotorRpt2Msg::CAN_ID] =
      this->create_publisher<pacmod_msgs::msg::MotorRpt2>("parsed_tx/steering_rpt_detail_2", 20);
    can_pubs_[SteerMotorRpt3Msg::CAN_ID] =
      this->create_publisher<pacmod_msgs::msg::MotorRpt3>("parsed_tx/steering_rpt_detail_3", 20);
  }

  if (
    vehicle_type_ == VehicleType::INTERNATIONAL_PROSTAR_122 ||
    vehicle_type_ == VehicleType::FREIGHTLINER_CASCADIA)
  {
    can_pubs_[WiperRptMsg::CAN_ID] =
      this->create_publisher<pacmod_msgs::msg::SystemRptInt>("parsed_tx/wiper_rpt", 20);
    can_pubs_[WiperAuxRptMsg::CAN_ID] =
      this->create_publisher<pacmod_msgs::msg::WiperAuxRpt>("parsed_tx/wiper_aux_rpt", 20);

    can_subs_[WiperCmdMsg::CAN_ID] = std::make_pair(
      this->create_subscription<pacmod_msgs::msg::SystemCmdInt>(
        "as_rx/wiper_cmd", 20,
        std::bind(&PACMod3Node::callback_wiper_cmd, this, std::placeholders::_1)),
      std::shared_ptr<LockedData>(new LockedData(WiperCmdMsg::DATA_LENGTH)));
  }

  if (
    vehicle_type_ == VehicleType::LEXUS_RX_450H ||
    vehicle_type_ == VehicleType::FREIGHTLINER_CASCADIA ||
    vehicle_type_ == VehicleType::JUPITER_SPIRIT ||
    vehicle_type_ == VehicleType::VEHICLE_5 ||
    vehicle_type_ == VehicleType::VEHICLE_6)
  {
    can_pubs_[DateTimeRptMsg::CAN_ID] =
      this->create_publisher<pacmod_msgs::msg::DateTimeRpt>(
      "parsed_tx/date_time_rpt", 20);
    can_pubs_[HeadlightRptMsg::CAN_ID] =
      this->create_publisher<pacmod_msgs::msg::SystemRptInt>(
      "parsed_tx/headlight_rpt", 20);
    can_pubs_[HeadlightAuxRptMsg::CAN_ID] =
      this->create_publisher<pacmod_msgs::msg::HeadlightAuxRpt>(
      "parsed_tx/headlight_aux_rpt", 20);
    can_pubs_[HornRptMsg::CAN_ID] =
      this->create_publisher<pacmod_msgs::msg::SystemRptBool>(
      "parsed_tx/horn_rpt", 20);
    can_pubs_[LatLonHeadingRptMsg::CAN_ID] =
      this->create_publisher<pacmod_msgs::msg::LatLonHeadingRpt>(
      "parsed_tx/lat_lon_heading_rpt", 20);
    can_pubs_[ParkingBrakeRptMsg::CAN_ID] =
      this->create_publisher<pacmod_msgs::msg::SystemRptBool>(
      "parsed_tx/parking_brake_status_rpt", 20);
    can_pubs_[WheelSpeedRptMsg::CAN_ID] =
      this->create_publisher<pacmod_msgs::msg::WheelSpeedRpt>(
      "parsed_tx/wheel_speed_rpt", 20);
    can_pubs_[YawRateRptMsg::CAN_ID] =
      this->create_publisher<pacmod_msgs::msg::YawRateRpt>(
      "parsed_tx/yaw_rate_rpt", 20);

    can_subs_[HeadlightCmdMsg::CAN_ID] = std::make_pair(
      this->create_subscription<pacmod_msgs::msg::SystemCmdInt>(
        "as_rx/headlight_cmd", 20,
        std::bind(&PACMod3Node::callback_headlight_cmd, this, std::placeholders::_1)),
      std::shared_ptr<LockedData>(new LockedData(HeadlightCmdMsg::DATA_LENGTH)));

    can_subs_[HornCmdMsg::CAN_ID] = std::make_pair(
      this->create_subscription<pacmod_msgs::msg::SystemCmdBool>(
        "as_rx/horn_cmd", 20,
        std::bind(&PACMod3Node::callback_horn_cmd, this, std::placeholders::_1)),
      std::shared_ptr<LockedData>(new LockedData(HornCmdMsg::DATA_LENGTH)));
  }

  if (vehicle_type_ == VehicleType::FREIGHTLINER_CASCADIA) {
    can_pubs_[CruiseControlButtonsRptMsg::CAN_ID] =
      this->create_publisher<pacmod_msgs::msg::SystemRptInt>(
      "parsed_tx/cruise_control_buttons_rpt", 20);
    can_pubs_[EngineBrakeRptMsg::CAN_ID] =
      this->create_publisher<pacmod_msgs::msg::SystemRptInt>(
      "parsed_tx/engine_brake_rpt", 20);
    can_pubs_[MarkerLampRptMsg::CAN_ID] =
      this->create_publisher<pacmod_msgs::msg::SystemRptBool>(
      "parsed_tx/marker_lamp_rpt", 20);
    can_pubs_[SprayerRptMsg::CAN_ID] =
      this->create_publisher<pacmod_msgs::msg::SystemRptBool>(
      "parsed_tx/sprayer_rpt", 20);
    can_pubs_[HazardLightRptMsg::CAN_ID] =
      this->create_publisher<pacmod_msgs::msg::SystemRptBool>(
      "parsed_tx/hazard_lights_rpt", 20);

    can_subs_[CruiseControlButtonsCmdMsg::CAN_ID] = std::make_pair(
      this->create_subscription<pacmod_msgs::msg::SystemCmdInt>(
        "as_rx/cruise_control_buttons_cmd", 20,
        std::bind(&PACMod3Node::callback_cruise_control_buttons_cmd, this, std::placeholders::_1)),
      std::shared_ptr<LockedData>(new LockedData(CruiseControlButtonsCmdMsg::DATA_LENGTH)));

    can_subs_[EngineBrakeCmdMsg::CAN_ID] = std::make_pair(
      this->create_subscription<pacmod_msgs::msg::SystemCmdInt>(
        "as_rx/engine_brake_cmd", 20,
        std::bind(&PACMod3Node::callback_engine_brake_cmd, this, std::placeholders::_1)),
      std::shared_ptr<LockedData>(new LockedData(EngineBrakeCmdMsg::DATA_LENGTH)));

    can_subs_[MarkerLampCmdMsg::CAN_ID] = std::make_pair(
      this->create_subscription<pacmod_msgs::msg::SystemCmdBool>(
        "as_rx/marker_lamp_cmd", 20,
        std::bind(&PACMod3Node::callback_marker_lamp_cmd, this, std::placeholders::_1)),
      std::shared_ptr<LockedData>(new LockedData(MarkerLampCmdMsg::DATA_LENGTH)));

    can_subs_[SprayerCmdMsg::CAN_ID] = std::make_pair(
      this->create_subscription<pacmod_msgs::msg::SystemCmdBool>(
        "as_rx/sprayer_cmd", 20,
        std::bind(&PACMod3Node::callback_sprayer_cmd, this, std::placeholders::_1)),
      std::shared_ptr<LockedData>(new LockedData(SprayerCmdMsg::DATA_LENGTH)));

    can_subs_[HazardLightCmdMsg::CAN_ID] = std::make_pair(
      this->create_subscription<pacmod_msgs::msg::SystemCmdBool>(
        "as_rx/hazard_lights_cmd", 20,
        std::bind(&PACMod3Node::callback_hazard_lights_cmd, this, std::placeholders::_1)),
      std::shared_ptr<LockedData>(new LockedData(HazardLightCmdMsg::DATA_LENGTH)));
  }

  if (vehicle_type_ == VehicleType::JUPITER_SPIRIT) {
    can_subs_[RearPassDoorCmdMsg::CAN_ID] = std::make_pair(
      this->create_subscription<pacmod_msgs::msg::SystemCmdInt>(
        "as_rx/rear_pass_door_cmd", 20,
        std::bind(&PACMod3Node::callback_rear_pass_door_cmd, this, std::placeholders::_1)),
      std::shared_ptr<LockedData>(new LockedData(RearPassDoorCmdMsg::DATA_LENGTH)));
  }

  if (vehicle_type_ == VehicleType::VEHICLE_4) {
    can_pubs_[DetectedObjectRptMsg::CAN_ID] =
      this->create_publisher<pacmod_msgs::msg::DetectedObjectRpt>(
      "parsed_tx/detected_object_rpt", 20);
    can_pubs_[VehicleDynamicsRptMsg::CAN_ID] =
      this->create_publisher<pacmod_msgs::msg::VehicleDynamicsRpt>(
      "parsed_tx/vehicle_dynamics_rpt", 20);
  }

  if (vehicle_type_ == VehicleType::VEHICLE_5) {
    can_pubs_[DoorRptMsg::CAN_ID] =
      this->create_publisher<pacmod_msgs::msg::DoorRpt>(
      "parsed_tx/door_rpt", 20);
    can_pubs_[InteriorLightsRptMsg::CAN_ID] =
      this->create_publisher<pacmod_msgs::msg::InteriorLightsRpt>(
      "parsed_tx/interior_lights_rpt", 20);
    can_pubs_[OccupancyRptMsg::CAN_ID] =
      this->create_publisher<pacmod_msgs::msg::OccupancyRpt>(
      "parsed_tx/occupancy_rpt", 20);
    can_pubs_[RearLightsRptMsg::CAN_ID] =
      this->create_publisher<pacmod_msgs::msg::RearLightsRpt>(
      "parsed_tx/rear_lights_rpt", 20);
  }

  pub_thread_ = std::make_shared<std::thread>();

  system_statuses_timer_ = this->create_wall_timer(
    33ms, std::bind(&PACMod3Node::publish_all_system_statuses, this));

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn PACMod3Node::on_activate(const lc::State & state)
{
  (void)state;

  pub_can_rx_->on_activate();

  for (auto & pub : can_pubs_) {
    pub.second->on_activate();
  }

  pub_enabled_->on_activate();
  pub_vehicle_speed_ms_->on_activate();
  pub_all_system_statuses_->on_activate();

  pub_thread_ = std::make_shared<std::thread>(std::bind(&PACMod3Node::publish_cmds, this));

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn PACMod3Node::on_deactivate(const lc::State & state)
{
  (void)state;

  pub_thread_->join();

  pub_can_rx_->on_deactivate();

  for (auto & pub : can_pubs_) {
    pub.second->on_deactivate();
  }

  pub_enabled_->on_deactivate();
  pub_vehicle_speed_ms_->on_deactivate();
  pub_all_system_statuses_->on_deactivate();

  // Reset all data in commands to 0
  for (auto & cmd : can_subs_) {
    auto data = cmd.second.second->getData();
    std::fill(data.begin(), data.end(), 0);
    cmd.second.second->setData(std::move(data));
  }

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn PACMod3Node::on_cleanup(const lc::State & state)
{
  (void)state;

  if (pub_thread_ && pub_thread_->joinable()) {
    pub_thread_->join();
  }

  pub_thread_.reset();
  system_statuses_timer_.reset();

  sub_can_tx_.reset();
  can_subs_.clear();

  pub_can_rx_.reset();
  can_pubs_.clear();
  pub_enabled_.reset();
  pub_vehicle_speed_ms_.reset();
  pub_all_system_statuses_.reset();

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn PACMod3Node::on_shutdown(const lc::State & state)
{
  (void)state;

  if (pub_thread_ && pub_thread_->joinable()) {
    pub_thread_->join();
  }

  pub_thread_.reset();

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn PACMod3Node::on_error(const lc::State & state)
{
  (void)state;

  if (pub_thread_ && pub_thread_->joinable()) {
    pub_thread_->join();
  }

  pub_thread_.reset();

  return LNI::CallbackReturn::FAILURE;
}

void PACMod3Node::callback_can_tx(const can_msgs::msg::Frame::SharedPtr msg)
{
  auto parser_class = Pacmod3TxMsg::make_message(msg->id);
  auto pub = can_pubs_.find(msg->id);

  if (parser_class != nullptr && pub != can_pubs_.end()) {
    const std::vector<uint8_t> data_copy(msg->data.begin(), msg->data.end());
    parser_class->parse(data_copy);
    tx_handler_.fillAndPublish(msg->id, frame_id_, pub->second, parser_class);

    if (parser_class->isSystem()) {
      auto dc_parser = std::dynamic_pointer_cast<SystemRptMsg>(parser_class);

      system_statuses[msg->id] = std::make_tuple(
        dc_parser->enabled,
        dc_parser->override_active,
        (dc_parser->command_output_fault |
        dc_parser->input_output_fault |
        dc_parser->output_reported_fault |
        dc_parser->pacmod_fault |
        dc_parser->vehicle_fault));
    }

    if (msg->id == GlobalRptMsg::CAN_ID) {
      auto dc_parser = std::dynamic_pointer_cast<GlobalRptMsg>(parser_class);

      auto enabled_msg = std::make_unique<std_msgs::msg::Bool>();
      enabled_msg->data = dc_parser->enabled;
      pub_enabled_->publish(std::move(enabled_msg));

      if (dc_parser->override_active || dc_parser->fault_active) {
        set_enable(false);
      }
    } else if (msg->id == VehicleSpeedRptMsg::CAN_ID) {
      auto dc_parser = std::dynamic_pointer_cast<VehicleSpeedRptMsg>(parser_class);

      auto msg = std::make_unique<std_msgs::msg::Float64>();
      msg->data = dc_parser->vehicle_speed;
      pub_vehicle_speed_ms_->publish(std::move(msg));
    }
  }
}

void PACMod3Node::callback_accel_cmd(const pacmod_msgs::msg::SystemCmdFloat::SharedPtr msg)
{
  lookup_and_encode(AccelCmdMsg::CAN_ID, msg);
}

void PACMod3Node::callback_brake_cmd(const pacmod_msgs::msg::SystemCmdFloat::SharedPtr msg)
{
  lookup_and_encode(BrakeCmdMsg::CAN_ID, msg);
}

void PACMod3Node::callback_cruise_control_buttons_cmd(
  const pacmod_msgs::msg::SystemCmdInt::SharedPtr msg)
{
  lookup_and_encode(CruiseControlButtonsCmdMsg::CAN_ID, msg);
}

void PACMod3Node::callback_engine_brake_cmd(const pacmod_msgs::msg::SystemCmdInt::SharedPtr msg)
{
  lookup_and_encode(EngineBrakeCmdMsg::CAN_ID, msg);
}

void PACMod3Node::callback_headlight_cmd(const pacmod_msgs::msg::SystemCmdInt::SharedPtr msg)
{
  lookup_and_encode(HeadlightCmdMsg::CAN_ID, msg);
}

void PACMod3Node::callback_hazard_lights_cmd(const pacmod_msgs::msg::SystemCmdBool::SharedPtr msg)
{
  lookup_and_encode(HazardLightCmdMsg::CAN_ID, msg);
}

void PACMod3Node::callback_horn_cmd(const pacmod_msgs::msg::SystemCmdBool::SharedPtr msg)
{
  lookup_and_encode(HornCmdMsg::CAN_ID, msg);
}

void PACMod3Node::callback_marker_lamp_cmd(const pacmod_msgs::msg::SystemCmdBool::SharedPtr msg)
{
  lookup_and_encode(MarkerLampCmdMsg::CAN_ID, msg);
}

void PACMod3Node::callback_rear_pass_door_cmd(const pacmod_msgs::msg::SystemCmdInt::SharedPtr msg)
{
  lookup_and_encode(RearPassDoorCmdMsg::CAN_ID, msg);
}

void PACMod3Node::callback_shift_cmd(const pacmod_msgs::msg::SystemCmdInt::SharedPtr msg)
{
  lookup_and_encode(ShiftCmdMsg::CAN_ID, msg);
}

void PACMod3Node::callback_sprayer_cmd(const pacmod_msgs::msg::SystemCmdBool::SharedPtr msg)
{
  lookup_and_encode(SprayerCmdMsg::CAN_ID, msg);
}

void PACMod3Node::callback_steer_cmd(const pacmod_msgs::msg::SteerSystemCmd::SharedPtr msg)
{
  lookup_and_encode(SteerCmdMsg::CAN_ID, msg);
}

void PACMod3Node::callback_turn_cmd(const pacmod_msgs::msg::SystemCmdInt::SharedPtr msg)
{
  lookup_and_encode(TurnSignalCmdMsg::CAN_ID, msg);
}

void PACMod3Node::callback_wiper_cmd(const pacmod_msgs::msg::SystemCmdInt::SharedPtr msg)
{
  lookup_and_encode(WiperCmdMsg::CAN_ID, msg);
}

void PACMod3Node::publish_cmds()
{
  while (rclcpp::ok() &&
    this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    auto next_time = std::chrono::steady_clock::now() + SEND_CMD_INTERVAL;

    for (auto & cmd : can_subs_) {
      auto msg = std::make_unique<can_msgs::msg::Frame>();
      auto data = cmd.second.second->getData();

      msg->id = cmd.first;
      msg->is_rtr = false;
      msg->is_extended = false;
      msg->is_error = false;
      msg->dlc = data.size();
      std::move(data.begin(), data.end(), msg->data.begin());

      pub_can_rx_->publish(std::move(msg));

      std::this_thread::sleep_for(INTER_MSG_PAUSE);
    }

    std::this_thread::sleep_until(next_time);
  }
}

void PACMod3Node::publish_all_system_statuses()
{
  auto ss_msg = std::make_unique<pacmod_msgs::msg::AllSystemStatuses>();

  for (const auto & system : system_statuses) {
    pacmod_msgs::msg::KeyValuePair kvp;

    if (system.first == AccelRptMsg::CAN_ID) {
      kvp.key = "Accelerator";
    } else if (system.first == BrakeRptMsg::CAN_ID) {
      kvp.key = "Brakes";
    } else if (system.first == CruiseControlButtonsRptMsg::CAN_ID) {
      kvp.key = "Cruise Control Buttons";
    } else if (system.first == DashControlsLeftRptMsg::CAN_ID) {
      kvp.key = "Dash Controls Left";
    } else if (system.first == DashControlsRightRptMsg::CAN_ID) {
      kvp.key = "Dash Controls Right";
    } else if (system.first == HazardLightRptMsg::CAN_ID) {
      kvp.key = "Hazard Lights";
    } else if (system.first == HeadlightRptMsg::CAN_ID) {
      kvp.key = "Headlights";
    } else if (system.first == HornRptMsg::CAN_ID) {
      kvp.key = "Horn";
    } else if (system.first == MediaControlsRptMsg::CAN_ID) {
      kvp.key = "Media Controls";
    } else if (system.first == ParkingBrakeRptMsg::CAN_ID) {
      kvp.key = "Parking Brake";
    } else if (system.first == ShiftRptMsg::CAN_ID) {
      kvp.key = "Shifter";
    } else if (system.first == SteerRptMsg::CAN_ID) {
      kvp.key = "Steering";
    } else if (system.first == TurnSignalRptMsg::CAN_ID) {
      kvp.key = "Turn Signals";
    } else if (system.first == RearPassDoorRptMsg::CAN_ID) {
      kvp.key = "Rear Passenger Door";
    } else if (system.first == WiperRptMsg::CAN_ID) {
      kvp.key = "Wipers";
    }

    kvp.value = std::get<0>(system.second) ? "True" : "False";
    ss_msg->enabled_status.push_back(kvp);

    kvp.value = std::get<1>(system.second) ? "True" : "False";
    ss_msg->overridden_status.push_back(kvp);

    kvp.value = std::get<2>(system.second) ? "True" : "False";
    ss_msg->fault_status.push_back(kvp);
  }

  pub_all_system_statuses_->publish(std::move(ss_msg));
}

void PACMod3Node::set_enable(bool enable)
{
  for (auto & cmd : can_subs_) {
    std::vector<uint8_t> current_data = cmd.second.second->getData();

    if (enable) {
      current_data[0] |= 0x01;  // Set Enable True
    } else {
      current_data[0] &= 0xFE;  // Set Enable False
    }

    cmd.second.second->setData(std::move(current_data));
  }
}

}  // namespace pacmod3

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(pacmod3::PACMod3Node)
