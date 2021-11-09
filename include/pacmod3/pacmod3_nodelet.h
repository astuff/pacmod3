// Copyright (c) 2021 AutonomouStuff, LLC
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

#ifndef PACMOD3_PACMOD3_NODELET_H
#define PACMOD3_PACMOD3_NODELET_H

#include "pacmod3/pacmod3_core.h"
#include "pacmod3/pacmod3_ros_msg_handler.h"

#include <map>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <can_msgs/Frame.h>
#include <pacmod3_msgs/SystemCmdBool.h>
#include <pacmod3_msgs/SystemCmdFloat.h>
#include <pacmod3_msgs/SystemCmdInt.h>
#include <pacmod3_msgs/GlobalRpt.h>
#include <pacmod3_msgs/AccelAuxRpt.h>
#include <pacmod3_msgs/AllSystemStatuses.h>
#include <pacmod3_msgs/BrakeAuxRpt.h>
#include <pacmod3_msgs/ComponentRpt.h>
#include <pacmod3_msgs/DateTimeRpt.h>
#include <pacmod3_msgs/DetectedObjectRpt.h>
#include <pacmod3_msgs/DoorRpt.h>
#include <pacmod3_msgs/EngineRpt.h>
#include <pacmod3_msgs/HeadlightAuxRpt.h>
#include <pacmod3_msgs/InteriorLightsRpt.h>
#include <pacmod3_msgs/LatLonHeadingRpt.h>
#include <pacmod3_msgs/MotorRpt1.h>
#include <pacmod3_msgs/MotorRpt2.h>
#include <pacmod3_msgs/MotorRpt3.h>
#include <pacmod3_msgs/OccupancyRpt.h>
#include <pacmod3_msgs/RearLightsRpt.h>
#include <pacmod3_msgs/ShiftAuxRpt.h>
#include <pacmod3_msgs/SteeringAuxRpt.h>
#include <pacmod3_msgs/SteeringCmd.h>
#include <pacmod3_msgs/SystemRptBool.h>
#include <pacmod3_msgs/SystemRptFloat.h>
#include <pacmod3_msgs/SystemRptInt.h>
#include <pacmod3_msgs/TurnAuxRpt.h>
#include <pacmod3_msgs/VehicleDynamicsRpt.h>
#include <pacmod3_msgs/VehicleSpeedRpt.h>
#include <pacmod3_msgs/VinRpt.h>
#include <pacmod3_msgs/WheelSpeedRpt.h>
#include <pacmod3_msgs/WiperAuxRpt.h>
#include <pacmod3_msgs/YawRateRpt.h>

namespace pacmod3
{

  const uint32_t SEND_CMD_INTERVAL = 33;
  const uint32_t INTER_MSG_PAUSE = 1;
  const float PACMOD_UPDATE_FREQ = 30.0;

class Pacmod3Nl : public nodelet::Nodelet
{
private:
  void onInit() override;
  void loadParams();

  void initializeBrakeMotorRptApi();
  void initializeSteeringMotorRptApi();
  void initializeWiperApi();
  void initializeHeadlightApi();
  void initializeHornApi();
  void initializeWheelSpeedApi();
  void initializeParkingBrakeRptApi();
  void initializeDoorRptApi();
  void initializeInteriorLightsRptApi();
  void initializeOccupancyRptApi();
  void initializeRearLightsRptApi();
  void initializeHazardLightApi();

  void initializeLexusSpecificApi();
  void initializeFreightlinerSpecificApi();
  void initializeJapanTaxiSpecificApi();
  void initializeVehicle4SpecificApi();

  void initializeApiForMsg(uint32_t msg_can_id);

  // ROS Callbacks
  void callback_accel_cmd_sub(const pacmod3_msgs::SystemCmdFloat::ConstPtr& msg);
  void callback_brake_cmd_sub(const pacmod3_msgs::SystemCmdFloat::ConstPtr& msg);
  void callback_cruise_control_buttons_set_cmd(const pacmod3_msgs::SystemCmdInt::ConstPtr& msg);
  void callback_dash_controls_left_set_cmd(const pacmod3_msgs::SystemCmdInt::ConstPtr& msg);
  void callback_dash_controls_right_set_cmd(const pacmod3_msgs::SystemCmdInt::ConstPtr& msg);
  void callback_headlight_set_cmd(const pacmod3_msgs::SystemCmdInt::ConstPtr& msg);
  void callback_horn_set_cmd(const pacmod3_msgs::SystemCmdBool::ConstPtr& msg);
  void callback_media_controls_set_cmd(const pacmod3_msgs::SystemCmdInt::ConstPtr& msg);
  void callback_shift_set_cmd(const pacmod3_msgs::SystemCmdInt::ConstPtr& msg);
  void callback_steer_cmd_sub(const pacmod3_msgs::SteeringCmd::ConstPtr& msg);
  void callback_turn_signal_set_cmd(const pacmod3_msgs::SystemCmdInt::ConstPtr& msg);
  void callback_rear_pass_door_set_cmd(const pacmod3_msgs::SystemCmdInt::ConstPtr& msg);
  void callback_wiper_set_cmd(const pacmod3_msgs::SystemCmdInt::ConstPtr& msg);
  void callback_engine_brake_set_cmd(const pacmod3_msgs::SystemCmdInt::ConstPtr& msg);
  void callback_marker_lamp_set_cmd(const pacmod3_msgs::SystemCmdBool::ConstPtr& msg);
  void callback_sprayer_set_cmd(const pacmod3_msgs::SystemCmdBool::ConstPtr& msg);
  void callback_hazard_lights_set_cmd(const pacmod3_msgs::SystemCmdBool::ConstPtr& msg);
  void SystemStatusUpdate(const ros::TimerEvent& event);

  void can_write(const ros::TimerEvent& event);
  void can_read(const can_msgs::Frame::ConstPtr &msg);
  void set_enable(bool val);
  template<class T> void lookup_and_encode(const uint32_t& can_id, const T& msg);

  std::unordered_map<uint32_t, ros::Publisher> pub_tx_list;
  Pacmod3RosMsgHandler handler;

  ros::Publisher enabled_pub;
  ros::Publisher can_rx_pub;
  ros::Publisher all_system_statuses_pub;
  ros::Publisher global_rpt_pub;
  ros::Publisher component_rpt_pub;
  ros::Publisher accel_rpt_pub;
  ros::Publisher brake_rpt_pub;
  ros::Publisher shift_rpt_pub;
  ros::Publisher steer_rpt_pub;
  ros::Publisher turn_rpt_pub;
  ros::Publisher rear_pass_door_rpt_pub;
  ros::Publisher vehicle_speed_pub;
  ros::Publisher vin_rpt_pub;
  ros::Publisher accel_aux_rpt_pub;
  ros::Publisher brake_aux_rpt_pub;
  ros::Publisher shift_aux_rpt_pub;
  ros::Publisher steer_aux_rpt_pub;
  ros::Publisher turn_aux_rpt_pub;

  ros::Subscriber can_tx_sub;
  ros::Subscriber accel_cmd_sub;
  ros::Subscriber brake_cmd_sub;
  ros::Subscriber shift_cmd_sub;
  ros::Subscriber steer_cmd_sub;
  ros::Subscriber turn_cmd_sub;
  ros::Subscriber rear_pass_door_cmd_sub;

  ros::Timer status_update_timer_;
  ros::Timer can_send_timer_;

  // std::thread can_write_thread;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // Vehicle-Specific Subscribers
  std::shared_ptr<ros::Subscriber> wiper_set_cmd_sub,
      headlight_set_cmd_sub,
      horn_set_cmd_sub,
      cruise_control_buttons_set_cmd_sub,
      dash_controls_left_set_cmd_sub,
      dash_controls_right_set_cmd_sub,
      engine_brake_set_cmd_sub,
      hazard_lights_set_cmd_sub,
      marker_lamp_set_cmd_sub,
      media_controls_set_cmd_sub,
      sprayer_set_cmd_sub;


  int dbc_major_version_;

  // Commands that have been received from ROS subscribers
  std::set<uint32_t> received_cmds;

  // Data shared across threads
  std::unordered_map<uint32_t, std::shared_ptr<LockedData>> rx_list;
  std::map<uint32_t, std::tuple<bool, bool, bool>> system_statuses;
  std::mutex sys_status_mutex_;
};

}  // namespace pacmod3


#endif  // PACMOD3_PACMOD3_NODELET_H
