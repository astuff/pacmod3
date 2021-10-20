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

#include "pacmod3/pacmod3_nodelet.h"

#include <memory>
#include <thread>
#include <utility>
#include <vector>

#include <pluginlib/class_list_macros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

namespace pacmod3
{

void Pacmod3Nl::onInit()
{
  nh_ = getMTNodeHandle();
  pnh_ = getMTPrivateNodeHandle();
  loadParams();

  // Publishers
  can_rx_pub = nh_.advertise<can_msgs::Frame>("can_rx", 20);
  enabled_pub = nh_.advertise<std_msgs::Bool>("as_tx/enabled", 20, true);
  vehicle_speed_ms_pub = nh_.advertise<std_msgs::Float64>("as_tx/vehicle_speed", 20);
  all_system_statuses_pub = nh_.advertise<pacmod3_msgs::AllSystemStatuses>("as_tx/all_system_statuses", 20);

  global_rpt_pub = nh_.advertise<pacmod3_msgs::GlobalRpt>("parsed_tx/global_rpt", 20);
  component_rpt_pub = nh_.advertise<pacmod3_msgs::ComponentRpt>("parsed_tx/component_rpt", 20);
  accel_rpt_pub = nh_.advertise<pacmod3_msgs::SystemRptFloat>("parsed_tx/accel_rpt", 20);
  brake_rpt_pub = nh_.advertise<pacmod3_msgs::SystemRptFloat>("parsed_tx/brake_rpt", 20);
  shift_rpt_pub = nh_.advertise<pacmod3_msgs::SystemRptInt>("parsed_tx/shift_rpt", 20);
  steer_rpt_pub = nh_.advertise<pacmod3_msgs::SystemRptFloat>("parsed_tx/steer_rpt", 20);
  turn_rpt_pub = nh_.advertise<pacmod3_msgs::SystemRptInt>("parsed_tx/turn_rpt", 20);
  rear_pass_door_rpt_pub = nh_.advertise<pacmod3_msgs::SystemRptInt>("parsed_tx/rear_pass_door_rpt", 20);
  vehicle_speed_pub = nh_.advertise<pacmod3_msgs::VehicleSpeedRpt>("parsed_tx/vehicle_speed_rpt", 20);
  vin_rpt_pub = nh_.advertise<pacmod3_msgs::VinRpt>("parsed_tx/vin_rpt", 5);
  accel_aux_rpt_pub = nh_.advertise<pacmod3_msgs::AccelAuxRpt>("parsed_tx/accel_aux_rpt", 20);
  brake_aux_rpt_pub = nh_.advertise<pacmod3_msgs::BrakeAuxRpt>("parsed_tx/brake_aux_rpt", 20);
  shift_aux_rpt_pub = nh_.advertise<pacmod3_msgs::ShiftAuxRpt>("parsed_tx/shift_aux_rpt", 20);
  steer_aux_rpt_pub = nh_.advertise<pacmod3_msgs::SteeringAuxRpt>("parsed_tx/steer_aux_rpt", 20);
  turn_aux_rpt_pub = nh_.advertise<pacmod3_msgs::TurnAuxRpt>("parsed_tx/turn_aux_rpt", 20);

  pub_tx_list.emplace(GlobalRptMsg::CAN_ID, std::move(global_rpt_pub));
  pub_tx_list.emplace(ComponentRptMsg::CAN_ID, std::move(component_rpt_pub));
  pub_tx_list.emplace(AccelRptMsg::CAN_ID, std::move(accel_rpt_pub));
  pub_tx_list.emplace(BrakeRptMsg::CAN_ID, std::move(brake_rpt_pub));
  pub_tx_list.emplace(ShiftRptMsg::CAN_ID, std::move(shift_rpt_pub));
  pub_tx_list.emplace(SteerRptMsg::CAN_ID, std::move(steer_rpt_pub));
  pub_tx_list.emplace(TurnSignalRptMsg::CAN_ID, std::move(turn_rpt_pub));
  pub_tx_list.emplace(RearPassDoorRptMsg::CAN_ID, std::move(rear_pass_door_rpt_pub));
  pub_tx_list.emplace(VehicleSpeedRptMsg::CAN_ID, std::move(vehicle_speed_pub));
  pub_tx_list.emplace(VinRptMsg::CAN_ID, std::move(vin_rpt_pub));
  pub_tx_list.emplace(AccelAuxRptMsg::CAN_ID, std::move(accel_aux_rpt_pub));
  pub_tx_list.emplace(BrakeAuxRptMsg::CAN_ID, std::move(brake_aux_rpt_pub));
  pub_tx_list.emplace(ShiftAuxRptMsg::CAN_ID, std::move(shift_aux_rpt_pub));
  pub_tx_list.emplace(SteeringAuxRptMsg::CAN_ID, std::move(steer_aux_rpt_pub));
  pub_tx_list.emplace(TurnAuxRptMsg::CAN_ID, std::move(turn_aux_rpt_pub));

  // Subscribers
  can_tx_sub = nh_.subscribe("can_tx", 20, &Pacmod3Nl::can_read, this);

  accel_cmd_sub = nh_.subscribe("as_rx/accel_cmd", 20, &Pacmod3Nl::callback_accel_cmd_sub, this);
  brake_cmd_sub = nh_.subscribe("as_rx/brake_cmd", 20, &Pacmod3Nl::callback_brake_cmd_sub, this);
  shift_cmd_sub = nh_.subscribe("as_rx/shift_cmd", 20, &Pacmod3Nl::callback_shift_set_cmd, this);
  steer_cmd_sub = nh_.subscribe("as_rx/steer_cmd", 20, &Pacmod3Nl::callback_steer_cmd_sub, this);
  turn_cmd_sub = nh_.subscribe("as_rx/turn_cmd", 20, &Pacmod3Nl::callback_turn_signal_set_cmd, this);
  rear_pass_door_cmd_sub = nh_.subscribe(
    "as_rx/rear_pass_door_cmd", 20, &Pacmod3Nl::callback_rear_pass_door_set_cmd, this);

  rx_list.emplace(
    AccelCmdMsg::CAN_ID,
    std::shared_ptr<LockedData>(new LockedData(AccelCmdMsg::DATA_LENGTH)));
  rx_list.emplace(
    BrakeCmdMsg::CAN_ID,
    std::shared_ptr<LockedData>(new LockedData(BrakeCmdMsg::DATA_LENGTH)));
  rx_list.emplace(
    ShiftCmdMsg::CAN_ID,
    std::shared_ptr<LockedData>(new LockedData(ShiftCmdMsg::DATA_LENGTH)));
  rx_list.emplace(
    SteerCmdMsg::CAN_ID,
    std::shared_ptr<LockedData>(new LockedData(SteerCmdMsg::DATA_LENGTH)));
  rx_list.emplace(
    TurnSignalCmdMsg::CAN_ID,
    std::shared_ptr<LockedData>(new LockedData(TurnSignalCmdMsg::DATA_LENGTH)));
  rx_list.emplace(
    RearPassDoorCmdMsg::CAN_ID,
    std::shared_ptr<LockedData>(new LockedData(RearPassDoorCmdMsg::DATA_LENGTH)));

  if (veh_type == VehicleType::POLARIS_GEM ||
      veh_type == VehicleType::POLARIS_RANGER ||
      veh_type == VehicleType::INTERNATIONAL_PROSTAR_122 ||
      veh_type == VehicleType::FREIGHTLINER_CASCADIA ||
      veh_type == VehicleType::JUPITER_SPIRIT)
  {
    ros::Publisher brake_rpt_detail_1_pub = nh_.advertise<pacmod3_msgs::MotorRpt1>("parsed_tx/brake_rpt_detail_1", 20);
    ros::Publisher brake_rpt_detail_2_pub = nh_.advertise<pacmod3_msgs::MotorRpt2>("parsed_tx/brake_rpt_detail_2", 20);
    ros::Publisher brake_rpt_detail_3_pub = nh_.advertise<pacmod3_msgs::MotorRpt3>("parsed_tx/brake_rpt_detail_3", 20);
    ros::Publisher steering_rpt_detail_1_pub =
      nh_.advertise<pacmod3_msgs::MotorRpt1>("parsed_tx/steer_rpt_detail_1", 20);
    ros::Publisher steering_rpt_detail_2_pub =
      nh_.advertise<pacmod3_msgs::MotorRpt2>("parsed_tx/steer_rpt_detail_2", 20);
    ros::Publisher steering_rpt_detail_3_pub =
      nh_.advertise<pacmod3_msgs::MotorRpt3>("parsed_tx/steer_rpt_detail_3", 20);

    pub_tx_list.emplace(BrakeMotorRpt1Msg::CAN_ID, std::move(brake_rpt_detail_1_pub));
    pub_tx_list.emplace(BrakeMotorRpt2Msg::CAN_ID, std::move(brake_rpt_detail_2_pub));
    pub_tx_list.emplace(BrakeMotorRpt3Msg::CAN_ID, std::move(brake_rpt_detail_3_pub));
    pub_tx_list.emplace(SteerMotorRpt1Msg::CAN_ID, std::move(steering_rpt_detail_1_pub));
    pub_tx_list.emplace(SteerMotorRpt2Msg::CAN_ID, std::move(steering_rpt_detail_2_pub));
    pub_tx_list.emplace(SteerMotorRpt3Msg::CAN_ID, std::move(steering_rpt_detail_3_pub));
  }

  if (veh_type == VehicleType::INTERNATIONAL_PROSTAR_122 || veh_type == VehicleType::FREIGHTLINER_CASCADIA)
  {
    ros::Publisher wiper_rpt_pub = nh_.advertise<pacmod3_msgs::SystemRptInt>("parsed_tx/wiper_rpt", 20);
    ros::Publisher wiper_aux_rpt_pub = nh_.advertise<pacmod3_msgs::WiperAuxRpt>("parsed_tx/wiper_aux_rpt", 20);

    pub_tx_list.emplace(WiperRptMsg::CAN_ID, std::move(wiper_rpt_pub));
    pub_tx_list.emplace(WiperAuxRptMsg::CAN_ID, std::move(wiper_aux_rpt_pub));

    wiper_set_cmd_sub = std::make_shared<ros::Subscriber>(nh_.subscribe(
      "as_rx/wiper_cmd", 20, &Pacmod3Nl::callback_wiper_set_cmd, this));

    rx_list.emplace(
      WiperCmdMsg::CAN_ID,
      std::shared_ptr<LockedData>(new LockedData(WiperCmdMsg::DATA_LENGTH)));
  }

  if (veh_type == VehicleType::LEXUS_RX_450H ||
      veh_type == VehicleType::FREIGHTLINER_CASCADIA ||
      veh_type == VehicleType::JUPITER_SPIRIT ||
      veh_type == VehicleType::VEHICLE_5 ||
      veh_type == VehicleType::VEHICLE_6)
  {
    ros::Publisher date_time_rpt_pub =
      nh_.advertise<pacmod3_msgs::DateTimeRpt>("parsed_tx/date_time_rpt", 20);
    ros::Publisher headlight_rpt_pub =
      nh_.advertise<pacmod3_msgs::SystemRptInt>("parsed_tx/headlight_rpt", 20);
    ros::Publisher horn_rpt_pub =
      nh_.advertise<pacmod3_msgs::SystemRptBool>("parsed_tx/horn_rpt", 20);
    ros::Publisher lat_lon_heading_rpt_pub =
      nh_.advertise<pacmod3_msgs::LatLonHeadingRpt>("parsed_tx/lat_lon_heading_rpt", 20);
    ros::Publisher parking_brake_rpt_pub =
      nh_.advertise<pacmod3_msgs::SystemRptBool>("parsed_tx/parking_brake_status_rpt", 20);
    ros::Publisher wheel_speed_rpt_pub =
      nh_.advertise<pacmod3_msgs::WheelSpeedRpt>("parsed_tx/wheel_speed_rpt", 20);
    ros::Publisher yaw_rate_rpt_pub =
      nh_.advertise<pacmod3_msgs::YawRateRpt>("parsed_tx/yaw_rate_rpt", 20);
    ros::Publisher headlight_aux_rpt_pub =
      nh_.advertise<pacmod3_msgs::HeadlightAuxRpt>("parsed_tx/headlight_aux_rpt", 20);

    pub_tx_list.emplace(DateTimeRptMsg::CAN_ID, std::move(date_time_rpt_pub));
    pub_tx_list.emplace(HeadlightRptMsg::CAN_ID, std::move(headlight_rpt_pub));
    pub_tx_list.emplace(HornRptMsg::CAN_ID, std::move(horn_rpt_pub));
    pub_tx_list.emplace(LatLonHeadingRptMsg::CAN_ID, std::move(lat_lon_heading_rpt_pub));
    pub_tx_list.emplace(ParkingBrakeRptMsg::CAN_ID, std::move(parking_brake_rpt_pub));
    pub_tx_list.emplace(WheelSpeedRptMsg::CAN_ID, std::move(wheel_speed_rpt_pub));
    pub_tx_list.emplace(YawRateRptMsg::CAN_ID, std::move(yaw_rate_rpt_pub));
    pub_tx_list.emplace(HeadlightAuxRptMsg::CAN_ID, std::move(headlight_aux_rpt_pub));

    headlight_set_cmd_sub = std::make_shared<ros::Subscriber>(nh_.subscribe(
      "as_rx/headlight_cmd", 20, &Pacmod3Nl::callback_headlight_set_cmd, this));
    horn_set_cmd_sub = std::make_shared<ros::Subscriber>(nh_.subscribe(
      "as_rx/horn_cmd", 20, &Pacmod3Nl::callback_horn_set_cmd, this));

    rx_list.emplace(
      HeadlightCmdMsg::CAN_ID,
      std::shared_ptr<LockedData>(new LockedData(HeadlightCmdMsg::DATA_LENGTH)));
    rx_list.emplace(
      HornCmdMsg::CAN_ID,
      std::shared_ptr<LockedData>(new LockedData(HornCmdMsg::DATA_LENGTH)));
  }

  if (veh_type == VehicleType::FREIGHTLINER_CASCADIA)
  {
    ros::Publisher cruise_control_buttons_rpt_pub =
      nh_.advertise<pacmod3_msgs::SystemRptInt>("parsed_tx/cruise_control_buttons_rpt", 20);
    ros::Publisher engine_brake_rpt_pub =
      nh_.advertise<pacmod3_msgs::SystemRptInt>("parsed_tx/engine_brake_rpt", 20);
    ros::Publisher engine_rpt_pub =
      nh_.advertise<pacmod3_msgs::EngineRpt>("parsed_tx/engine_rpt", 20);
    ros::Publisher marker_lamp_rpt_pub =
      nh_.advertise<pacmod3_msgs::SystemRptBool>("parsed_tx/marker_lamp_rpt", 20);
    ros::Publisher sprayer_rpt_pub =
      nh_.advertise<pacmod3_msgs::SystemRptBool>("parsed_tx/sprayer_rpt", 20);
    ros::Publisher hazard_lights_rpt_pub =
      nh_.advertise<pacmod3_msgs::SystemRptBool>("parsed_tx/hazard_lights_rpt", 20);

    pub_tx_list.emplace(CruiseControlButtonsRptMsg::CAN_ID, std::move(cruise_control_buttons_rpt_pub));
    pub_tx_list.emplace(EngineBrakeRptMsg::CAN_ID, std::move(engine_brake_rpt_pub));
    pub_tx_list.emplace(EngineRptMsg::CAN_ID, std::move(engine_rpt_pub));
    pub_tx_list.emplace(MarkerLampRptMsg::CAN_ID, std::move(marker_lamp_rpt_pub));
    pub_tx_list.emplace(SprayerRptMsg::CAN_ID, std::move(sprayer_rpt_pub));
    pub_tx_list.emplace(HazardLightRptMsg::CAN_ID, std::move(hazard_lights_rpt_pub));

    cruise_control_buttons_set_cmd_sub = std::make_shared<ros::Subscriber>(
      nh_.subscribe("as_rx/cruise_control_buttons_cmd", 20, &Pacmod3Nl::callback_cruise_control_buttons_set_cmd, this));
    engine_brake_set_cmd_sub = std::make_shared<ros::Subscriber>(
      nh_.subscribe("as_rx/engine_brake_cmd", 20, &Pacmod3Nl::callback_engine_brake_set_cmd, this));
    marker_lamp_set_cmd_sub = std::make_shared<ros::Subscriber>(
      nh_.subscribe("as_rx/marker_lamp_cmd", 20, &Pacmod3Nl::callback_marker_lamp_set_cmd, this));
    sprayer_set_cmd_sub = std::make_shared<ros::Subscriber>(
      nh_.subscribe("as_rx/sprayer_cmd", 20, &Pacmod3Nl::callback_sprayer_set_cmd, this));
    hazard_lights_set_cmd_sub = std::make_shared<ros::Subscriber>(
      nh_.subscribe("as_rx/hazard_lights_cmd", 20, &Pacmod3Nl::callback_hazard_lights_set_cmd, this));

    rx_list.emplace(
      CruiseControlButtonsCmdMsg::CAN_ID,
      std::shared_ptr<LockedData>(new LockedData(CruiseControlButtonsCmdMsg::DATA_LENGTH)));
    rx_list.emplace(
      EngineBrakeCmdMsg::CAN_ID,
      std::shared_ptr<LockedData>(new LockedData(EngineBrakeCmdMsg::DATA_LENGTH)));
    rx_list.emplace(
      MarkerLampCmdMsg::CAN_ID,
      std::shared_ptr<LockedData>(new LockedData(MarkerLampCmdMsg::DATA_LENGTH)));
    rx_list.emplace(
      SprayerCmdMsg::CAN_ID,
      std::shared_ptr<LockedData>(new LockedData(SprayerCmdMsg::DATA_LENGTH)));
    rx_list.emplace(
      HazardLightCmdMsg::CAN_ID,
      std::shared_ptr<LockedData>(new LockedData(HazardLightCmdMsg::DATA_LENGTH)));
  }

  if (veh_type == VehicleType::VEHICLE_4)
  {
    ros::Publisher detected_object_rpt_pub =
      nh_.advertise<pacmod3_msgs::DetectedObjectRpt>("parsed_tx/detected_object_rpt", 20);
    ros::Publisher vehicle_dynamics_rpt_pub =
      nh_.advertise<pacmod3_msgs::VehicleDynamicsRpt>("parsed_tx/vehicle_dynamics_rpt", 20);

    pub_tx_list.emplace(DetectedObjectRptMsg::CAN_ID, std::move(detected_object_rpt_pub));
    pub_tx_list.emplace(VehicleDynamicsRptMsg::CAN_ID, std::move(vehicle_dynamics_rpt_pub));
  }

  if (veh_type == VehicleType::VEHICLE_5)
  {
    ros::Publisher occupancy_rpt_pub =
      nh_.advertise<pacmod3_msgs::OccupancyRpt>("parsed_tx/occupancy_rpt", 20);
    ros::Publisher interior_lights_rpt_pub =
      nh_.advertise<pacmod3_msgs::InteriorLightsRpt>("parsed_tx/interior_lights_rpt", 20);
    ros::Publisher door_rpt_pub =
      nh_.advertise<pacmod3_msgs::DoorRpt>("parsed_tx/door_rpt", 20);
    ros::Publisher rear_lights_rpt_pub =
      nh_.advertise<pacmod3_msgs::RearLightsRpt>("parsed_tx/rear_lights_rpt", 20);

    pub_tx_list.emplace(OccupancyRptMsg::CAN_ID, std::move(occupancy_rpt_pub));
    pub_tx_list.emplace(InteriorLightsRptMsg::CAN_ID, std::move(interior_lights_rpt_pub));
    pub_tx_list.emplace(DoorRptMsg::CAN_ID, std::move(door_rpt_pub));
    pub_tx_list.emplace(RearLightsRptMsg::CAN_ID, std::move(rear_lights_rpt_pub));
  }

  // Initialize Turn Signal with non-0 value
  TurnSignalCmdMsg turn_encoder;
  turn_encoder.encode(false, false, false, pacmod3_msgs::SystemCmdInt::TURN_NONE);
  rx_list[TurnSignalCmdMsg::CAN_ID]->setData(std::move(turn_encoder.data));

  // Set initial state
  set_enable(false);

  // Start timers for repeating tasks
  can_send_timer_ = nh_.createTimer(ros::Duration(1/PACMOD_UPDATE_FREQ), &Pacmod3Nl::can_write, this);
  status_update_timer_ = nh_.createTimer(ros::Duration(1/PACMOD_UPDATE_FREQ), &Pacmod3Nl::SystemStatusUpdate, this);
}

void Pacmod3Nl::loadParams()
{
  // Get and validate parameters
  if (pnh_.getParam("vehicle_type", veh_type_string))
  {
    ROS_INFO("PACMod3 - Got vehicle type of: %s", veh_type_string.c_str());

    if (veh_type_string == "POLARIS_GEM")
      veh_type = POLARIS_GEM;
    else if (veh_type_string == "POLARIS_RANGER")
      veh_type = POLARIS_RANGER;
    else if (veh_type_string == "INTERNATIONAL_PROSTAR_122")
      veh_type = INTERNATIONAL_PROSTAR_122;
    else if (veh_type_string == "LEXUS_RX_450H")
      veh_type = LEXUS_RX_450H;
    else if (veh_type_string == "FREIGHTLINER_CASCADIA")
      veh_type = FREIGHTLINER_CASCADIA;
    else if (veh_type_string == "VEHICLE_4")
      veh_type = VEHICLE_4;
    else if (veh_type_string == "VEHICLE_5")
      veh_type = VEHICLE_5;
    else if (veh_type_string == "VEHICLE_6")
      veh_type = VEHICLE_6;
    else if (veh_type_string == "JUPITER_SPIRIT")
      veh_type = JUPITER_SPIRIT;
    else
    {
      veh_type = VehicleType::POLARIS_GEM;
      ROS_WARN("PACMod3 - An invalid vehicle type was entered. Assuming POLARIS_GEM.");
    }
  }

  pnh_.param<int>("dbc_major_version", dbc_major_version_, 3);
  if (dbc_major_version_ != 3)
  {
    NODELET_ERROR("This driver currently only supports PACMod DBC version 3");
    ros::shutdown();
  }
}

void Pacmod3Nl::callback_accel_cmd_sub(const pacmod3_msgs::SystemCmdFloat::ConstPtr& msg)
{
  lookup_and_encode(AccelCmdMsg::CAN_ID, msg);
}

void Pacmod3Nl::callback_brake_cmd_sub(const pacmod3_msgs::SystemCmdFloat::ConstPtr& msg)
{
  lookup_and_encode(BrakeCmdMsg::CAN_ID, msg);
}

void Pacmod3Nl::callback_cruise_control_buttons_set_cmd(const pacmod3_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(CruiseControlButtonsCmdMsg::CAN_ID, msg);
}

void Pacmod3Nl::callback_dash_controls_left_set_cmd(const pacmod3_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(DashControlsLeftCmdMsg::CAN_ID, msg);
}

void Pacmod3Nl::callback_dash_controls_right_set_cmd(const pacmod3_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(DashControlsRightCmdMsg::CAN_ID, msg);
}

void Pacmod3Nl::callback_headlight_set_cmd(const pacmod3_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(HeadlightCmdMsg::CAN_ID, msg);
}

void Pacmod3Nl::callback_horn_set_cmd(const pacmod3_msgs::SystemCmdBool::ConstPtr& msg)
{
  lookup_and_encode(HornCmdMsg::CAN_ID, msg);
}

void Pacmod3Nl::callback_media_controls_set_cmd(const pacmod3_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(MediaControlsCmdMsg::CAN_ID, msg);
}

void Pacmod3Nl::callback_shift_set_cmd(const pacmod3_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(ShiftCmdMsg::CAN_ID, msg);
}

void Pacmod3Nl::callback_steer_cmd_sub(const pacmod3_msgs::SteeringCmd::ConstPtr& msg)
{
  lookup_and_encode(SteerCmdMsg::CAN_ID, msg);
}

void Pacmod3Nl::callback_turn_signal_set_cmd(const pacmod3_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(TurnSignalCmdMsg::CAN_ID, msg);
}

void Pacmod3Nl::callback_rear_pass_door_set_cmd(const pacmod3_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(RearPassDoorCmdMsg::CAN_ID, msg);
}

void Pacmod3Nl::callback_wiper_set_cmd(const pacmod3_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(WiperCmdMsg::CAN_ID, msg);
}

void Pacmod3Nl::callback_engine_brake_set_cmd(const pacmod3_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(EngineBrakeCmdMsg::CAN_ID, msg);
}

void Pacmod3Nl::callback_marker_lamp_set_cmd(const pacmod3_msgs::SystemCmdBool::ConstPtr& msg)
{
  lookup_and_encode(MarkerLampCmdMsg::CAN_ID, msg);
}

void Pacmod3Nl::callback_sprayer_set_cmd(const pacmod3_msgs::SystemCmdBool::ConstPtr& msg)
{
  lookup_and_encode(SprayerCmdMsg::CAN_ID, msg);
}

void Pacmod3Nl::callback_hazard_lights_set_cmd(const pacmod3_msgs::SystemCmdBool::ConstPtr& msg)
{
  lookup_and_encode(HazardLightCmdMsg::CAN_ID, msg);
}

void Pacmod3Nl::SystemStatusUpdate(const ros::TimerEvent& event)
{
  pacmod3_msgs::AllSystemStatuses ss_msg;

  std::unique_lock<std::mutex> lock(sys_status_mutex_);
  for (auto system = system_statuses.begin(); system != system_statuses.end(); ++system)
  {
    pacmod3_msgs::KeyValuePair kvp;

    if (system->first == AccelRptMsg::CAN_ID)
      kvp.key = "Accelerator";
    else if (system->first == BrakeRptMsg::CAN_ID)
      kvp.key = "Brakes";
    else if (system->first == CruiseControlButtonsRptMsg::CAN_ID)
      kvp.key = "Cruise Control Buttons";
    else if (system->first == DashControlsLeftRptMsg::CAN_ID)
      kvp.key = "Dash Controls Left";
    else if (system->first == DashControlsRightRptMsg::CAN_ID)
      kvp.key = "Dash Controls Right";
    else if (system->first == HazardLightRptMsg::CAN_ID)
      kvp.key = "Hazard Lights";
    else if (system->first == HeadlightRptMsg::CAN_ID)
      kvp.key = "Headlights";
    else if (system->first == HornRptMsg::CAN_ID)
      kvp.key = "Horn";
    else if (system->first == MediaControlsRptMsg::CAN_ID)
      kvp.key = "Media Controls";
    else if (system->first == ParkingBrakeRptMsg::CAN_ID)
      kvp.key = "Parking Brake";
    else if (system->first == ShiftRptMsg::CAN_ID)
      kvp.key = "Shifter";
    else if (system->first == SteerRptMsg::CAN_ID)
      kvp.key = "Steering";
    else if (system->first == TurnSignalRptMsg::CAN_ID)
      kvp.key = "Turn Signals";
    else if (system->first == RearPassDoorRptMsg::CAN_ID)
      kvp.key = "Rear Passenger Door";
    else if (system->first == WiperRptMsg::CAN_ID)
      kvp.key = "Wipers";

    kvp.value = std::get<0>(system->second) ? "True" : "False";

    ss_msg.enabled_status.push_back(kvp);

    kvp.value = std::get<1>(system->second) ? "True" : "False";

    ss_msg.overridden_status.push_back(kvp);

    kvp.value = std::get<2>(system->second) ? "True" : "False";

    ss_msg.fault_status.push_back(kvp);
  }

  all_system_statuses_pub.publish(ss_msg);
}

void Pacmod3Nl::can_write(const ros::TimerEvent& event)
{
  for (const auto& can_id : received_cmds)
  {
    auto data = rx_list[can_id]->getData();

    can_msgs::Frame frame;
    frame.id = can_id;
    frame.is_rtr = false;
    frame.is_extended = false;
    frame.is_error = false;
    frame.dlc = data.size();
    std::move(data.begin(), data.end(), frame.data.begin());

    frame.header.stamp = ros::Time::now();

    can_rx_pub.publish(frame);

    std::this_thread::sleep_for(std::chrono::milliseconds(INTER_MSG_PAUSE));
  }
}

void Pacmod3Nl::can_read(const can_msgs::Frame::ConstPtr &msg)
{
  auto parser_class = Pacmod3TxMsg::make_message(msg->id);
  auto pub = pub_tx_list.find(msg->id);

  // Only parse messages for which we have a parser and a publisher.
  if (parser_class != NULL && pub != pub_tx_list.end())
  {
    parser_class->parse(const_cast<uint8_t *>(&msg->data[0]));
    handler.fillAndPublish(msg->id, "pacmod", pub->second, parser_class);

    if (parser_class->isSystem())
    {
      auto dc_parser = std::dynamic_pointer_cast<SystemRptMsg>(parser_class);

      std::unique_lock<std::mutex> lock(sys_status_mutex_);
      system_statuses[msg->id] = std::make_tuple(dc_parser->enabled,
                                 dc_parser->override_active,
                                 (dc_parser->command_output_fault |
                                  dc_parser->input_output_fault |
                                  dc_parser->output_reported_fault |
                                  dc_parser->pacmod_fault |
                                  dc_parser->vehicle_fault));
    }

    if (msg->id == GlobalRptMsg::CAN_ID)
    {
      auto dc_parser = std::dynamic_pointer_cast<GlobalRptMsg>(parser_class);

      std_msgs::Bool bool_pub_msg;
      bool_pub_msg.data = (dc_parser->enabled);
      enabled_pub.publish(bool_pub_msg);

      if (dc_parser->override_active ||
          dc_parser->fault_active)
        set_enable(false);
    }
    else if (msg->id == VehicleSpeedRptMsg::CAN_ID)
    {
      auto dc_parser = std::dynamic_pointer_cast<VehicleSpeedRptMsg>(parser_class);

      // Now publish by itself
      std_msgs::Float64 veh_spd_ms_msg;
      veh_spd_ms_msg.data = (dc_parser->vehicle_speed);
      vehicle_speed_ms_pub.publish(veh_spd_ms_msg);
    }
  }
}

// Sets the PACMod3 enable flag through CAN.
void Pacmod3Nl::set_enable(bool val)
{
  for (auto & cmd : rx_list)
  {
    // This assumes that all data in rx_list are encoded
    // command messages which means the least significant
    // bit in their first byte will be the enable flag.
    std::vector<uint8_t> current_data = cmd.second->getData();

    if (val)
      current_data[0] |= 0x01;  // Enable true
    else
      current_data[0] &= 0xFE;  // Enable false

    cmd.second->setData(current_data);
  }
}

// Looks up the appropriate LockedData and inserts the command info
template<class T> void Pacmod3Nl::lookup_and_encode(const uint32_t& can_id, const T& msg)
{
  auto rx_it = rx_list.find(can_id);

  if (rx_it != rx_list.end())
  {
    rx_it->second->setData(Pacmod3RxRosMsgHandler::unpackAndEncode(can_id, msg));
    received_cmds.insert(can_id);
  }
  else
  {
    ROS_WARN("Received command message for ID 0x%x for which we did not have an encoder.", can_id);
  }
}

}  // namespace pacmod3

PLUGINLIB_EXPORT_CLASS(pacmod3::Pacmod3Nl, nodelet::Nodelet);