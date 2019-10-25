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

#include "pacmod3/pacmod3_node.hpp"

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

namespace AS
{
namespace Drivers
{
namespace PACMod3
{

PACMod3Node::PACMod3Node(rclcpp::NodeOptions options)
  : lc::LifecycleNode("pacmod3_driver", options)
{
  std::string vehicle_type_string = this->declare_parameter("vehicle_type", "POLARIS_GEM");
  frame_id_ = this->declare_parameter("frame_id", "pacmod");

  if (vehicle_type_string == "POLARIS_GEM") {
    vehicle_type_ = VehicleType::POLARIS_GEM;
  } else if (vehicle_type_string == "POLARIS_RANGER") {
    vehicle_type_ = VehicleType::POLARIS_RANGER;
  } else if (vehicle_type_string == "INTERNATIONAL_PROSTAR_122") {
    vehicle_type_ = VehicleType::INTERNATIONAL_PROSTAR_122;
  } else if (vehicle_type_string == "LEXUS_RX_450H") {
    vehicle_type_ = VehicleType::LEXUS_RX_450H;
  } else if (vehicle_type_string == "VEHICLE_4") {
    vehicle_type_ = VehicleType::VEHICLE_4;
  } else if (vehicle_type_string == "VEHICLE_5") {
    vehicle_type_ = VehicleType::VEHICLE_5;
  } else if (vehicle_type_string == "VEHICLE_6") {
    vehicle_type_ = VehicleType::VEHICLE_6;
  } else if (vehicle_type_string == "JUPITER_SPIRIT") {
    vehicle_type_ = VehicleType::JUPITER_SPIRIT;
  } else {
    vehicle_type_string = "POLARIS_GEM";
    vehicle_type_ = VehicleType::POLARIS_GEM;
    RCLCPP_WARN(this->get_logger(), "An invalid vehicle type was entered. Defaulting to POLARIS_GEM.");
  }

  RCLCPP_INFO(this->get_logger(), "Got vehicle type: %s", vehicle_type_string.c_str());
  RCLCPP_INFO(this->get_logger(), "Got frame id: %s", frame_id_.c_str());
}

LNI::CallbackReturn PACMod3Node::on_configure(const lc::State & state)
{
  (void)state;

  pub_can_rx_ = this->create_publisher<can_msgs::msg::Frame>("can_rx", 100);

  can_pubs_[GlobalRptMsg::CAN_ID] = this->create_publisher<pacmod_msgs::msg::GlobalRpt>("parsed_tx/global_rpt", 20);
  can_pubs_[ComponentRptMsg::CAN_ID] = this->create_publisher<pacmod_msgs::msg::ComponentRpt>("parsed_tx/component_rpt", 20);
  can_pubs_[AccelRptMsg::CAN_ID] = this->create_publisher<pacmod_msgs::msg::SystemRptFloat>("parsed_tx/accel_rpt", 20);
  can_pubs_[BrakeRptMsg::CAN_ID] = this->create_publisher<pacmod_msgs::msg::SystemRptFloat>("parsed_tx/brake_rpt", 20);
  can_pubs_[ShiftRptMsg::CAN_ID] = this->create_publisher<pacmod_msgs::msg::SystemRptInt>("parsed_tx/shift_rpt", 20);
  can_pubs_[SteerRptMsg::CAN_ID] = this->create_publisher<pacmod_msgs::msg::SystemRptFloat>("parsed_tx/steer_rpt", 20);
  can_pubs_[TurnSignalRptMsg::CAN_ID] = this->create_publisher<pacmod_msgs::msg::SystemRptInt>("parsed_tx/turn_rpt", 20);
  can_pubs_[AccelAuxRptMsg::CAN_ID] = this->create_publisher<pacmod_msgs::msg::AccelAuxRpt>("parsed_tx/accel_aux_rpt", 20);
  can_pubs_[BrakeAuxRptMsg::CAN_ID] = this->create_publisher<pacmod_msgs::msg::BrakeAuxRpt>("parsed_tx/brake_aux_rpt", 20);
  can_pubs_[ShiftAuxRptMsg::CAN_ID] = this->create_publisher<pacmod_msgs::msg::ShiftAuxRpt>("parsed_tx/shift_aux_rpt", 20);
  can_pubs_[SteerAuxRptMsg::CAN_ID] = this->create_publisher<pacmod_msgs::msg::SteerAuxRpt>("parsed_tx/steer_aux_rpt", 20);
  can_pubs_[TurnAuxRptMsg::CAN_ID] = this->create_publisher<pacmod_msgs::msg::TurnAuxRpt>("parsed_tx/turn_aux_rpt", 20);
  can_pubs_[VehicleSpeedRptMsg::CAN_ID] = this->create_publisher<pacmod_msgs::msg::VehicleSpeedRpt>("parsed_tx/vehicle_speed_rpt", 20);
  can_pubs_[VinRptMsg::CAN_ID] = this->create_publisher<pacmod_msgs::msg::VinRpt>("parsed_tx/vin_rpt", 5);

  pub_enabled_ = this->create_publisher<std_msgs::msg::Bool>("as_tx/enabled", rclcpp::QoS(1).transient_local());
  pub_vehicle_speed_ms_ = this->create_publisher<std_msgs::msg::Float64>("as_tx/vehicle_speed", 20);
  pub_all_system_statuses_ = this->create_publisher<pacmod_msgs::msg::AllSystemStatuses>("as_tx/all_system_statuses", 20);

  sub_can_tx_ = this->create_subscription<can_msgs::msg::Frame>(
    "can_tx", 100, std::bind(&PACMod3Node::callback_can_tx, this, std::placeholders::_1));

  can_subs_[AccelCmdMsg::CAN_ID] = this->create_subscription<pacmod_msgs::msg::SystemCmdFloat>(
    "as_rx/accel_cmd", 20, std::bind(&PACMod3Node::callback_accel_cmd, this, std::placeholders::_1));
  can_subs_[BrakeCmdMsg::CAN_ID] = this->create_subscription<pacmod_msgs::msg::SystemCmdFloat>(
    "as_rx/brake_cmd", 20, std::bind(&PACMod3Node::callback_brake_cmd, this, std::placeholders::_1));
  can_subs_[ShiftCmdMsg::CAN_ID] = this->create_subscription<pacmod_msgs::msg::SystemCmdInt>(
    "as_rx/shift_cmd", 20, std::bind(&PACMod3Node::callback_shift_cmd, this, std::placeholders::_1));
  can_subs_[SteerCmdMsg::CAN_ID] = this->create_subscription<pacmod_msgs::msg::SteerSystemCmd>(
    "as_rx/steer_cmd", 20, std::bind(&PACMod3Node::callback_steer_cmd, this, std::placeholders::_1));
  can_subs_[TurnSignalCmdMsg::CAN_ID] = this->create_subscription<pacmod_msgs::msg::SystemCmdInt>(
    "as_rx/turn_cmd", 20, std::bind(&PACMod3Node::callback_turn_cmd, this, std::placeholders::_1));

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

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn PACMod3Node::on_deactivate(const lc::State & state)
{
  (void)state;

  pub_can_rx_->on_deactivate();

  for (auto & pub : can_pubs_) {
    pub.second->on_deactivate();
  }

  pub_enabled_->on_deactivate();
  pub_vehicle_speed_ms_->on_deactivate();
  pub_all_system_statuses_->on_deactivate();

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn PACMod3Node::on_cleanup(const lc::State & state)
{
  (void)state;

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

  return LNI::CallbackReturn::SUCCESS;
}

void PACMod3Node::callback_can_tx(const can_msgs::msg::Frame::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Got a message with CAN ID: %u", msg->id);
}

void PACMod3Node::callback_accel_cmd(const pacmod_msgs::msg::SystemCmdFloat::SharedPtr msg)
{
}

void PACMod3Node::callback_brake_cmd(const pacmod_msgs::msg::SystemCmdFloat::SharedPtr msg)
{
}

void PACMod3Node::callback_shift_cmd(const pacmod_msgs::msg::SystemCmdInt::SharedPtr msg)
{
}

void PACMod3Node::callback_steer_cmd(const pacmod_msgs::msg::SteerSystemCmd::SharedPtr msg)
{
}

void PACMod3Node::callback_turn_cmd(const pacmod_msgs::msg::SystemCmdInt::SharedPtr msg)
{
}

}  // namespace PACMod3
}  // namespace Drivers
}  // namespace AS

RCLCPP_COMPONENTS_REGISTER_NODE(AS::Drivers::PACMod3::PACMod3Node)
