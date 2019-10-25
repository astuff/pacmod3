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
  : lc::LifecycleNode("pacmod_node", options)
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

  RCLCPP_INFO(this->get_logger(), "Got vehicle type: %s", vehicle_type_string);
  RCLCPP_INFO(this->get_logger(), "Got frame id: %s", frame_id_);
}

LNI::CallbackReturn PACMod3Node::on_configure(const lc::State & state)
{
  (void)state;

  pubs_["can_rx"] = this->create_publisher<can_msgs::msg::Frame>("can_rx", 100);
  pubs_["parsed_tx/global_rpt"] = this->create_publisher<pacmod_msgs::msg::GlobalRpt>("parsed_tx/global_rpt", 20);

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn PACMod3Node::on_activate(const lc::State & state)
{
  (void)state;

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn PACMod3Node::on_deactivate(const lc::State & state)
{
  (void)state;

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn PACMod3Node::on_cleanup(const lc::State & state)
{
  (void)state;

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn PACMod3Node::on_shutdown(const lc::State & state)
{
  (void)state;

  return LNI::CallbackReturn::SUCCESS;
}

}  // namespace PACMod3
}  // namespace Drivers
}  // namespace AS

RCLCPP_COMPONENTS_REGISTER_NODE(AS::Drivers::PACMod3::PACMod3Node)
