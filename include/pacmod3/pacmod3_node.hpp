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

#ifndef PACMOD3__PACMOD3_NODE_HPP_
#define PACMOD3__PACMOD3_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <can_msgs/msg/frame.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <unordered_map>

#include "pacmod3/pacmod3_common.hpp"
#include "pacmod3/pacmod3_ros_msg_handler.hpp"

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

namespace pacmod3
{

/// \brief PACMod3Node class which can translate messages
/// being sent to or from a PACMod drive-by-wire system.
class PACMod3Node final
  : public lc::LifecycleNode
{
public:
  /// \brief Default constructor
  explicit PACMod3Node(rclcpp::NodeOptions options);
  ~PACMod3Node();

  /// \brief Callback from transition to "configuring" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_configure(const lc::State & state) override;

  /// \brief Callback from transition to "activating" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_activate(const lc::State & state) override;

  /// \brief Callback from transition to "deactivating" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_deactivate(const lc::State & state) override;

  /// \brief Callback from transition to "unconfigured" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_cleanup(const lc::State & state) override;

  /// \brief Callback from transition to "shutdown" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_shutdown(const lc::State & state) override;

  /// \brief Callback from transition to "error" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_error(const lc::State & state) override;

private:
  void callback_can_tx(const can_msgs::msg::Frame::SharedPtr msg);
  void callback_accel_cmd(const pacmod_msgs::msg::SystemCmdFloat::SharedPtr msg);
  void callback_brake_cmd(const pacmod_msgs::msg::SystemCmdFloat::SharedPtr msg);
  void callback_cruise_control_buttons_cmd(const pacmod_msgs::msg::SystemCmdInt::SharedPtr msg);
  void callback_engine_brake_cmd(const pacmod_msgs::msg::SystemCmdInt::SharedPtr msg);
  void callback_hazard_lights_cmd(const pacmod_msgs::msg::SystemCmdBool::SharedPtr msg);
  void callback_headlight_cmd(const pacmod_msgs::msg::SystemCmdInt::SharedPtr msg);
  void callback_horn_cmd(const pacmod_msgs::msg::SystemCmdBool::SharedPtr msg);
  void callback_marker_lamp_cmd(const pacmod_msgs::msg::SystemCmdBool::SharedPtr msg);
  void callback_rear_pass_door_cmd(const pacmod_msgs::msg::SystemCmdInt::SharedPtr msg);
  void callback_shift_cmd(const pacmod_msgs::msg::SystemCmdInt::SharedPtr msg);
  void callback_sprayer_cmd(const pacmod_msgs::msg::SystemCmdBool::SharedPtr msg);
  void callback_steer_cmd(const pacmod_msgs::msg::SteerSystemCmd::SharedPtr msg);
  void callback_turn_cmd(const pacmod_msgs::msg::SystemCmdInt::SharedPtr msg);
  void callback_wiper_cmd(const pacmod_msgs::msg::SystemCmdInt::SharedPtr msg);

  template<class T>
  void lookup_and_encode(const unsigned int & can_id, const T & msg)
  {
    auto cmd = can_subs_.find(can_id);

    if (cmd != can_subs_.end()) {
      cmd->second.second->setData(Pacmod3RxRosMsgHandler::unpackAndEncode(can_id, msg));
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Received a command message for ID 0x%x for which we do not have an encoder.",
        can_id);
    }
  }

  void publish_cmds();
  void publish_all_system_statuses();
  void set_enable(bool enable);

  static constexpr auto SEND_CMD_INTERVAL = std::chrono::milliseconds(33);
  static constexpr auto INTER_MSG_PAUSE = std::chrono::milliseconds(1);

  VehicleType vehicle_type_;
  std::string frame_id_;
  Pacmod3TxRosMsgHandler tx_handler_;
  std::map<unsigned int, std::tuple<bool, bool, bool>> system_statuses;

  std::shared_ptr<rclcpp::TimerBase> system_statuses_timer_;
  std::shared_ptr<lc::LifecyclePublisher<can_msgs::msg::Frame>> pub_can_rx_;
  std::unordered_map<unsigned int, std::shared_ptr<lc::LifecyclePublisherInterface>> can_pubs_;
  std::shared_ptr<lc::LifecyclePublisher<std_msgs::msg::Bool>> pub_enabled_;
  std::shared_ptr<lc::LifecyclePublisher<std_msgs::msg::Float64>> pub_vehicle_speed_ms_;
  std::shared_ptr<lc::LifecyclePublisher<
      pacmod_msgs::msg::AllSystemStatuses>> pub_all_system_statuses_;

  std::shared_ptr<rclcpp::Subscription<can_msgs::msg::Frame>> sub_can_tx_;
  std::unordered_map<unsigned int,
    std::pair<std::shared_ptr<rclcpp::SubscriptionBase>,
    std::shared_ptr<LockedData>>> can_subs_;

  std::shared_ptr<std::thread> pub_thread_;
};

}  // namespace pacmod3

#endif  // PACMOD3__PACMOD3_NODE_HPP_
