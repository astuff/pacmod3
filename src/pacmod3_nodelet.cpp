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

  handler = std::make_unique<Pacmod3RosMsgHandler>(dbc_major_version_);

  // Publishers
  can_rx_pub = nh_.advertise<can_msgs::Frame>("can_rx", 20);
  enabled_pub = nh_.advertise<std_msgs::Bool>("enabled", 20, true);
  all_system_statuses_pub = nh_.advertise<pacmod3_msgs::AllSystemStatuses>("all_system_statuses", 20);

  global_rpt_pub = nh_.advertise<pacmod3_msgs::GlobalRpt>("global_rpt", 20);
  component_rpt_00_pub = nh_.advertise<pacmod3_msgs::ComponentRpt>("component_rpt_00", 20);
  component_rpt_01_pub = nh_.advertise<pacmod3_msgs::ComponentRpt>("component_rpt_01", 20);
  component_rpt_02_pub = nh_.advertise<pacmod3_msgs::ComponentRpt>("component_rpt_02", 20);
  accel_rpt_pub = nh_.advertise<pacmod3_msgs::SystemRptFloat>("accel_rpt", 20);
  brake_rpt_pub = nh_.advertise<pacmod3_msgs::SystemRptFloat>("brake_rpt", 20);
  shift_rpt_pub = nh_.advertise<pacmod3_msgs::SystemRptInt>("shift_rpt", 20);
  steering_rpt_pub = nh_.advertise<pacmod3_msgs::SystemRptFloat>("steering_rpt", 20);
  turn_rpt_pub = nh_.advertise<pacmod3_msgs::SystemRptInt>("turn_rpt", 20);
  accel_aux_rpt_pub = nh_.advertise<pacmod3_msgs::AccelAuxRpt>("accel_aux_rpt", 20);
  brake_aux_rpt_pub = nh_.advertise<pacmod3_msgs::BrakeAuxRpt>("brake_aux_rpt", 20);
  shift_aux_rpt_pub = nh_.advertise<pacmod3_msgs::ShiftAuxRpt>("shift_aux_rpt", 20);
  steer_aux_rpt_pub = nh_.advertise<pacmod3_msgs::SteeringAuxRpt>("steering_aux_rpt", 20);
  turn_aux_rpt_pub = nh_.advertise<pacmod3_msgs::TurnAuxRpt>("turn_aux_rpt", 20);
  vehicle_speed_pub = nh_.advertise<pacmod3_msgs::VehicleSpeedRpt>("vehicle_speed_rpt", 20);
  vin_rpt_pub = nh_.advertise<pacmod3_msgs::VinRpt>("vin_rpt", 5);

  pub_tx_list.emplace(GLOBAL_RPT_CANID, std::move(global_rpt_pub));
  pub_tx_list.emplace(COMPONENT_RPT_00_CANID, std::move(component_rpt_00_pub));
  pub_tx_list.emplace(COMPONENT_RPT_01_CANID, std::move(component_rpt_01_pub));
  pub_tx_list.emplace(COMPONENT_RPT_02_CANID, std::move(component_rpt_02_pub));
  pub_tx_list.emplace(ACCEL_RPT_CANID, std::move(accel_rpt_pub));
  pub_tx_list.emplace(BRAKE_RPT_CANID, std::move(brake_rpt_pub));
  pub_tx_list.emplace(SHIFT_RPT_CANID, std::move(shift_rpt_pub));
  pub_tx_list.emplace(STEERING_RPT_CANID, std::move(steering_rpt_pub));
  pub_tx_list.emplace(TURN_RPT_CANID, std::move(turn_rpt_pub));
  pub_tx_list.emplace(ACCEL_AUX_RPT_CANID, std::move(accel_aux_rpt_pub));
  pub_tx_list.emplace(BRAKE_AUX_RPT_CANID, std::move(brake_aux_rpt_pub));
  pub_tx_list.emplace(SHIFT_AUX_RPT_CANID, std::move(shift_aux_rpt_pub));
  pub_tx_list.emplace(STEERING_AUX_RPT_CANID, std::move(steer_aux_rpt_pub));
  pub_tx_list.emplace(TURN_AUX_RPT_CANID, std::move(turn_aux_rpt_pub));
  pub_tx_list.emplace(VEHICLE_SPEED_RPT_CANID, std::move(vehicle_speed_pub));
  pub_tx_list.emplace(VIN_RPT_CANID, std::move(vin_rpt_pub));

  // Subscribers
  can_tx_sub = nh_.subscribe("can_tx", 20, &Pacmod3Nl::can_read, this);

  accel_cmd_sub = nh_.subscribe("accel_cmd", 20, &Pacmod3Nl::callback_accel_cmd_sub, this);
  brake_cmd_sub = nh_.subscribe("brake_cmd", 20, &Pacmod3Nl::callback_brake_cmd_sub, this);
  shift_cmd_sub = nh_.subscribe("shift_cmd", 20, &Pacmod3Nl::callback_shift_set_cmd, this);
  steer_cmd_sub = nh_.subscribe("steering_cmd", 20, &Pacmod3Nl::callback_steer_cmd_sub, this);
  turn_cmd_sub = nh_.subscribe("turn_cmd", 20, &Pacmod3Nl::callback_turn_signal_set_cmd, this);

  rx_list.emplace(
    ACCEL_CMD_CANID,
    std::shared_ptr<LockedData>(new LockedData()));
  rx_list.emplace(
    BRAKE_CMD_CANID,
    std::shared_ptr<LockedData>(new LockedData()));
  rx_list.emplace(
    SHIFT_CMD_CANID,
    std::shared_ptr<LockedData>(new LockedData()));
  rx_list.emplace(
    STEERING_CMD_CANID,
    std::shared_ptr<LockedData>(new LockedData()));
  rx_list.emplace(
    TURN_CMD_CANID,
    std::shared_ptr<LockedData>(new LockedData()));

  // Init cmds to send, always init core subsystems with enable false
  received_cmds_.insert(ACCEL_CMD_CANID);
  received_cmds_.insert(BRAKE_CMD_CANID);
  received_cmds_.insert(SHIFT_CMD_CANID);
  received_cmds_.insert(STEERING_CMD_CANID);
  received_cmds_.insert(TURN_CMD_CANID);

  // Initialize Turn Signal with non-0 value
  std::vector<unsigned char> initial_turn_cmd;
  initial_turn_cmd.resize(2);
  initial_turn_cmd[1] = pacmod3_msgs::SystemCmdInt::TURN_NONE;
  rx_list[TURN_CMD_CANID]->setData(std::move(initial_turn_cmd));

  // Set initial state
  set_enable(false);

  // Start timers for repeating tasks
  can_send_timer_ = nh_.createTimer(ros::Duration(1/PACMOD_UPDATE_FREQ), &Pacmod3Nl::can_write, this);
  status_update_timer_ = nh_.createTimer(ros::Duration(1/PACMOD_UPDATE_FREQ), &Pacmod3Nl::SystemStatusUpdate, this);
}

void Pacmod3Nl::loadParams()
{
  // Get and validate parameters
  pnh_.param<int>("dbc_major_version", dbc_major_version_, 3);
  if (dbc_major_version_ < 3 || dbc_major_version_ > 12)
  {
    NODELET_ERROR("This driver currently only supports PACMod DBC version 3 through 12");
    ros::shutdown();
  }
}

void Pacmod3Nl::initializeBrakeMotorRptApi()
{
  ros::Publisher brake_rpt_detail_1_pub = nh_.advertise<pacmod3_msgs::MotorRpt1>("brake_motor_rpt_1", 20);
  ros::Publisher brake_rpt_detail_2_pub = nh_.advertise<pacmod3_msgs::MotorRpt2>("brake_motor_rpt_2", 20);
  ros::Publisher brake_rpt_detail_3_pub = nh_.advertise<pacmod3_msgs::MotorRpt3>("brake_motor_rpt_3", 20);

  pub_tx_list.emplace(BRAKE_MOTOR_RPT_1_CANID, std::move(brake_rpt_detail_1_pub));
  pub_tx_list.emplace(BRAKE_MOTOR_RPT_2_CANID, std::move(brake_rpt_detail_2_pub));
  pub_tx_list.emplace(BRAKE_MOTOR_RPT_3_CANID, std::move(brake_rpt_detail_3_pub));
  NODELET_INFO("Initialized BrakeMotorRpt API");
}

void Pacmod3Nl::initializeSteeringMotorRptApi()
{
  ros::Publisher steering_rpt_detail_1_pub =
    nh_.advertise<pacmod3_msgs::MotorRpt1>("steering_motor_rpt_1", 20);
  ros::Publisher steering_rpt_detail_2_pub =
    nh_.advertise<pacmod3_msgs::MotorRpt2>("steering_motor_rpt_2", 20);
  ros::Publisher steering_rpt_detail_3_pub =
    nh_.advertise<pacmod3_msgs::MotorRpt3>("steering_motor_rpt_3", 20);
  pub_tx_list.emplace(STEERING_MOTOR_RPT_1_CANID, std::move(steering_rpt_detail_1_pub));
  pub_tx_list.emplace(STEERING_MOTOR_RPT_2_CANID, std::move(steering_rpt_detail_2_pub));
  pub_tx_list.emplace(STEERING_MOTOR_RPT_3_CANID, std::move(steering_rpt_detail_3_pub));
  NODELET_INFO("Initialized SteeringMotorRpt API");
}

void Pacmod3Nl::initializeWiperApi()
{
  ros::Publisher wiper_rpt_pub = nh_.advertise<pacmod3_msgs::SystemRptInt>("wiper_rpt", 20);
  ros::Publisher wiper_aux_rpt_pub = nh_.advertise<pacmod3_msgs::WiperAuxRpt>("wiper_aux_rpt", 20);
  pub_tx_list.emplace(WIPER_RPT_CANID, std::move(wiper_rpt_pub));
  pub_tx_list.emplace(WIPER_AUX_RPT_CANID, std::move(wiper_aux_rpt_pub));

  wiper_set_cmd_sub = std::make_shared<ros::Subscriber>(nh_.subscribe(
    "wiper_cmd", 20, &Pacmod3Nl::callback_wiper_set_cmd, this));
  rx_list.emplace(
    WIPER_CMD_CANID,
    std::shared_ptr<LockedData>(new LockedData()));
  NODELET_INFO("Initialized Wiper API");
}

void Pacmod3Nl::initializeHeadlightApi()
{
  ros::Publisher headlight_rpt_pub =
    nh_.advertise<pacmod3_msgs::SystemRptInt>("headlight_rpt", 20);
  ros::Publisher headlight_aux_rpt_pub =
    nh_.advertise<pacmod3_msgs::HeadlightAuxRpt>("headlight_aux_rpt", 20);
  pub_tx_list.emplace(HEADLIGHT_RPT_CANID, std::move(headlight_rpt_pub));
  pub_tx_list.emplace(HEADLIGHT_AUX_RPT_CANID, std::move(headlight_aux_rpt_pub));

  headlight_set_cmd_sub = std::make_shared<ros::Subscriber>(nh_.subscribe(
    "headlight_cmd", 20, &Pacmod3Nl::callback_headlight_set_cmd, this));
  rx_list.emplace(
      HEADLIGHT_CMD_CANID,
      std::shared_ptr<LockedData>(new LockedData()));
  NODELET_INFO("Initialized Headlight API");
}

void Pacmod3Nl::initializeHornApi()
{
  ros::Publisher horn_rpt_pub =
    nh_.advertise<pacmod3_msgs::SystemRptBool>("horn_rpt", 20);
  pub_tx_list.emplace(HORN_RPT_CANID, std::move(horn_rpt_pub));

  horn_set_cmd_sub = std::make_shared<ros::Subscriber>(nh_.subscribe(
    "horn_cmd", 20, &Pacmod3Nl::callback_horn_set_cmd, this));
  rx_list.emplace(
    HORN_CMD_CANID,
    std::shared_ptr<LockedData>(new LockedData()));
  NODELET_INFO("Initialized Horn API");
}

void Pacmod3Nl::initializeWheelSpeedApi()
{
  ros::Publisher wheel_speed_rpt_pub =
    nh_.advertise<pacmod3_msgs::WheelSpeedRpt>("wheel_speed_rpt", 20);
  pub_tx_list.emplace(WHEEL_SPEED_RPT_CANID, std::move(wheel_speed_rpt_pub));
  NODELET_INFO("Initialized WheelSpeed API");
}

void Pacmod3Nl::initializeParkingBrakeRptApi()
{
  ros::Publisher parking_brake_rpt_pub =
    nh_.advertise<pacmod3_msgs::SystemRptBool>("parking_brake_rpt", 20);
  pub_tx_list.emplace(PARKING_BRAKE_RPT_CANID, std::move(parking_brake_rpt_pub));
  NODELET_INFO("Initialized ParkingBrakeRpt API");
}

void Pacmod3Nl::initializeDoorRptApi()
{
  ros::Publisher door_rpt_pub =
    nh_.advertise<pacmod3_msgs::DoorRpt>("door_rpt", 20);
  pub_tx_list.emplace(DOOR_RPT_CANID, std::move(door_rpt_pub));
  NODELET_INFO("Initialized DoorRpt API");
}

void Pacmod3Nl::initializeInteriorLightsRptApi()
{
  ros::Publisher interior_lights_rpt_pub =
    nh_.advertise<pacmod3_msgs::InteriorLightsRpt>("interior_lights_rpt", 20);
  pub_tx_list.emplace(INTERIOR_LIGHTS_RPT_CANID, std::move(interior_lights_rpt_pub));
  NODELET_INFO("Initialized InteriorLights API");
}

void Pacmod3Nl::initializeOccupancyRptApi()
{
  ros::Publisher occupancy_rpt_pub =
    nh_.advertise<pacmod3_msgs::OccupancyRpt>("occupancy_rpt", 20);
  pub_tx_list.emplace(OCCUPANCY_RPT_CANID, std::move(occupancy_rpt_pub));
  NODELET_INFO("Initialized OccupancyRpt API");
}

void Pacmod3Nl::initializeRearLightsRptApi()
{
  ros::Publisher rear_lights_rpt_pub =
    nh_.advertise<pacmod3_msgs::RearLightsRpt>("rear_lights_rpt", 20);
  pub_tx_list.emplace(REAR_LIGHTS_RPT_CANID, std::move(rear_lights_rpt_pub));
  NODELET_INFO("Initialized RearLightsRpt API");
}

void Pacmod3Nl::initializeHazardLightApi()
{
  ros::Publisher hazard_lights_rpt_pub =
    nh_.advertise<pacmod3_msgs::SystemRptBool>("hazard_lights_rpt", 20);
  pub_tx_list.emplace(HAZARD_LIGHTS_RPT_CANID, std::move(hazard_lights_rpt_pub));

  hazard_lights_set_cmd_sub = std::make_shared<ros::Subscriber>(
    nh_.subscribe("hazard_lights_cmd", 20, &Pacmod3Nl::callback_hazard_lights_set_cmd, this));
  rx_list.emplace(
    HAZARD_LIGHTS_CMD_CANID,
    std::shared_ptr<LockedData>(new LockedData()));
  NODELET_INFO("Initialized HazardLight API");
}

void Pacmod3Nl::initializeEStopRptApi()
{
  ros::Publisher estop_rpt_pub =
    nh_.advertise<pacmod3_msgs::EStopRpt>("estop_rpt", 20);
  pub_tx_list.emplace(ESTOP_RPT_CANID, std::move(estop_rpt_pub));
  NODELET_INFO("Initialized EStopRpt API");
}

void Pacmod3Nl::initializeGlobalRpt2Api()
{
  ros::Publisher global_rpt_2_pub =
    nh_.advertise<pacmod3_msgs::GlobalRpt2>("global_rpt_2", 20);
  pub_tx_list.emplace(GLOBAL_RPT_2_CANID, std::move(global_rpt_2_pub));
  NODELET_INFO("Initialized GlobalRpt2 API");
}

void Pacmod3Nl::initializeLexusSpecificApi()
{
  ros::Publisher date_time_rpt_pub =
    nh_.advertise<pacmod3_msgs::DateTimeRpt>("date_time_rpt", 20);
  ros::Publisher lat_lon_heading_rpt_pub =
    nh_.advertise<pacmod3_msgs::LatLonHeadingRpt>("lat_lon_heading_rpt", 20);
  ros::Publisher yaw_rate_rpt_pub =
    nh_.advertise<pacmod3_msgs::YawRateRpt>("yaw_rate_rpt", 20);
  pub_tx_list.emplace(DATE_TIME_RPT_CANID, std::move(date_time_rpt_pub));
  pub_tx_list.emplace(LAT_LON_HEADING_RPT_CANID, std::move(lat_lon_heading_rpt_pub));
  pub_tx_list.emplace(YAW_RATE_RPT_CANID, std::move(yaw_rate_rpt_pub));
  NODELET_INFO("Initialized Lexus-specific API");
}

void Pacmod3Nl::initializeFreightlinerSpecificApi()
{
  ros::Publisher cruise_control_buttons_rpt_pub =
    nh_.advertise<pacmod3_msgs::SystemRptInt>("cruise_control_buttons_rpt", 20);
  ros::Publisher engine_brake_rpt_pub =
    nh_.advertise<pacmod3_msgs::SystemRptInt>("engine_brake_rpt", 20);
  ros::Publisher marker_lamp_rpt_pub =
    nh_.advertise<pacmod3_msgs::SystemRptBool>("marker_lamp_rpt", 20);
  ros::Publisher sprayer_rpt_pub =
    nh_.advertise<pacmod3_msgs::SystemRptBool>("sprayer_rpt", 20);
  pub_tx_list.emplace(CRUISE_CONTROL_BUTTONS_RPT_CANID, std::move(cruise_control_buttons_rpt_pub));
  pub_tx_list.emplace(ENGINE_BRAKE_RPT_CANID, std::move(engine_brake_rpt_pub));
  pub_tx_list.emplace(MARKER_LAMP_RPT_CANID, std::move(marker_lamp_rpt_pub));
  pub_tx_list.emplace(SPRAYER_RPT_CANID, std::move(sprayer_rpt_pub));

  cruise_control_buttons_set_cmd_sub = std::make_shared<ros::Subscriber>(
    nh_.subscribe("cruise_control_buttons_cmd", 20, &Pacmod3Nl::callback_cruise_control_buttons_set_cmd, this));
  engine_brake_set_cmd_sub = std::make_shared<ros::Subscriber>(
    nh_.subscribe("engine_brake_cmd", 20, &Pacmod3Nl::callback_engine_brake_set_cmd, this));
  marker_lamp_set_cmd_sub = std::make_shared<ros::Subscriber>(
    nh_.subscribe("marker_lamp_cmd", 20, &Pacmod3Nl::callback_marker_lamp_set_cmd, this));
  sprayer_set_cmd_sub = std::make_shared<ros::Subscriber>(
    nh_.subscribe("sprayer_cmd", 20, &Pacmod3Nl::callback_sprayer_set_cmd, this));
  rx_list.emplace(
    CRUISE_CONTROL_BUTTONS_CMD_CANID,
    std::shared_ptr<LockedData>(new LockedData()));
  rx_list.emplace(
    ENGINE_BRAKE_CMD_CANID,
    std::shared_ptr<LockedData>(new LockedData()));
  rx_list.emplace(
    MARKER_LAMP_CMD_CANID,
    std::shared_ptr<LockedData>(new LockedData()));
  rx_list.emplace(
    SPRAYER_CMD_CANID,
    std::shared_ptr<LockedData>(new LockedData()));
  NODELET_INFO("Initialized Freightliner-specific API");
}

void Pacmod3Nl::initializeJapanTaxiSpecificApi()
{
  rear_pass_door_rpt_pub = nh_.advertise<pacmod3_msgs::SystemRptInt>("rear_pass_door_rpt", 20);
  pub_tx_list.emplace(REAR_PASS_DOOR_RPT_CANID, std::move(rear_pass_door_rpt_pub));

  rear_pass_door_cmd_sub = nh_.subscribe(
    "rear_pass_door_cmd", 20, &Pacmod3Nl::callback_rear_pass_door_set_cmd, this);
  rx_list.emplace(
    REAR_PASS_DOOR_CMD_CANID,
    std::shared_ptr<LockedData>(new LockedData()));
  NODELET_INFO("Initialized Japan Taxi-specific API");
}

void Pacmod3Nl::initializeFordTransitSpecificApi()
{
  media_controls_rpt_pub = nh_.advertise<pacmod3_msgs::SystemRptInt>("media_controls_rpt", 20);
  pub_tx_list.emplace(MEDIA_CONTROLS_RPT_CANID, std::move(media_controls_rpt_pub));

  media_controls_cmd_sub = nh_.subscribe(
    "media_controls_cmd", 20, &Pacmod3Nl::callback_media_controls_set_cmd, this);
  rx_list.emplace(
    MEDIA_CONTROLS_CMD_CANID,
    std::shared_ptr<LockedData>(new LockedData()));
  NODELET_INFO("Initialized Ford Transit-specific API");
}

void Pacmod3Nl::initializeVehicle4SpecificApi()
{
  ros::Publisher detected_object_rpt_pub =
    nh_.advertise<pacmod3_msgs::DetectedObjectRpt>("detected_object_rpt", 20);
  ros::Publisher vehicle_dynamics_rpt_pub =
    nh_.advertise<pacmod3_msgs::VehicleDynamicsRpt>("vehicle_dynamics_rpt", 20);

  pub_tx_list.emplace(DETECTED_OBJECT_RPT_CANID, std::move(detected_object_rpt_pub));
  pub_tx_list.emplace(VEH_DYNAMICS_RPT_CANID, std::move(vehicle_dynamics_rpt_pub));
  NODELET_INFO("Initialized Vehicle4-specific API");
}

void Pacmod3Nl::initializeApiForMsg(uint32_t msg_can_id)
{
  // Need to initialize pubs/subs for this message group
  switch (msg_can_id)
  {
    case BRAKE_MOTOR_RPT_1_CANID:
    case BRAKE_MOTOR_RPT_2_CANID:
    case BRAKE_MOTOR_RPT_3_CANID:
      {
        initializeBrakeMotorRptApi();
        break;
      }
    case STEERING_MOTOR_RPT_1_CANID:
    case STEERING_MOTOR_RPT_2_CANID:
    case STEERING_MOTOR_RPT_3_CANID:
      {
        initializeSteeringMotorRptApi();
        break;
      }
    case WIPER_RPT_CANID:
    case WIPER_AUX_RPT_CANID:
      {
        initializeWiperApi();
        break;
      }
    case HEADLIGHT_RPT_CANID:
    case HEADLIGHT_AUX_RPT_CANID:
      {
        initializeHeadlightApi();
        break;
      }
    case HORN_RPT_CANID:
      {
        initializeHornApi();
        break;
      }
    case WHEEL_SPEED_RPT_CANID:
      {
        initializeWheelSpeedApi();
        break;
      }
    case PARKING_BRAKE_RPT_CANID:
      {
        initializeParkingBrakeRptApi();
        break;
      }
    case DOOR_RPT_CANID:
      {
        initializeDoorRptApi();
        break;
      }
    case INTERIOR_LIGHTS_RPT_CANID:
      {
        initializeInteriorLightsRptApi();
        break;
      }
    case OCCUPANCY_RPT_CANID:
      {
        initializeOccupancyRptApi();
        break;
      }
    case REAR_LIGHTS_RPT_CANID:
      {
        initializeRearLightsRptApi();
        break;
      }
    case HAZARD_LIGHTS_RPT_CANID:
      {
        initializeHazardLightApi();
        break;
      }
    case ESTOP_RPT_CANID:
      {
        initializeEStopRptApi();
        break;
      }
    case GLOBAL_RPT_2_CANID:
      {
        initializeGlobalRpt2Api();
      }
    case DATE_TIME_RPT_CANID:
    case LAT_LON_HEADING_RPT_CANID:
    case YAW_RATE_RPT_CANID:
      {
        initializeLexusSpecificApi();
        break;
      }
    case CRUISE_CONTROL_BUTTONS_RPT_CANID:
    case ENGINE_BRAKE_RPT_CANID:
    case MARKER_LAMP_RPT_CANID:
    case SPRAYER_RPT_CANID:
      {
        initializeFreightlinerSpecificApi();
        break;
      }
    case MEDIA_CONTROLS_RPT_CANID:
      {
        initializeFordTransitSpecificApi();
        break;
      }
    case DETECTED_OBJECT_RPT_CANID:
    case VEH_DYNAMICS_RPT_CANID:
      {
        initializeVehicle4SpecificApi();
        break;
      }
  }
}

void Pacmod3Nl::callback_accel_cmd_sub(const pacmod3_msgs::SystemCmdFloat::ConstPtr& msg)
{
  lookup_and_encode(ACCEL_CMD_CANID, msg);
}

void Pacmod3Nl::callback_brake_cmd_sub(const pacmod3_msgs::SystemCmdFloat::ConstPtr& msg)
{
  lookup_and_encode(BRAKE_CMD_CANID, msg);
}

void Pacmod3Nl::callback_cruise_control_buttons_set_cmd(const pacmod3_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(CRUISE_CONTROL_BUTTONS_CMD_CANID, msg);
}

void Pacmod3Nl::callback_headlight_set_cmd(const pacmod3_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(HEADLIGHT_CMD_CANID, msg);
}

void Pacmod3Nl::callback_horn_set_cmd(const pacmod3_msgs::SystemCmdBool::ConstPtr& msg)
{
  lookup_and_encode(HORN_CMD_CANID, msg);
}

void Pacmod3Nl::callback_media_controls_set_cmd(const pacmod3_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(MEDIA_CONTROLS_CMD_CANID, msg);
}

void Pacmod3Nl::callback_shift_set_cmd(const pacmod3_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(SHIFT_CMD_CANID, msg);
}

void Pacmod3Nl::callback_steer_cmd_sub(const pacmod3_msgs::SteeringCmd::ConstPtr& msg)
{
  lookup_and_encode(STEERING_CMD_CANID, msg);
}

void Pacmod3Nl::callback_turn_signal_set_cmd(const pacmod3_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(TURN_CMD_CANID, msg);
}

void Pacmod3Nl::callback_rear_pass_door_set_cmd(const pacmod3_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(REAR_PASS_DOOR_CMD_CANID, msg);
}

void Pacmod3Nl::callback_wiper_set_cmd(const pacmod3_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(WIPER_CMD_CANID, msg);
}

void Pacmod3Nl::callback_engine_brake_set_cmd(const pacmod3_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(ENGINE_BRAKE_CMD_CANID, msg);
}

void Pacmod3Nl::callback_marker_lamp_set_cmd(const pacmod3_msgs::SystemCmdBool::ConstPtr& msg)
{
  lookup_and_encode(MARKER_LAMP_CMD_CANID, msg);
}

void Pacmod3Nl::callback_sprayer_set_cmd(const pacmod3_msgs::SystemCmdBool::ConstPtr& msg)
{
  lookup_and_encode(SPRAYER_CMD_CANID, msg);
}

void Pacmod3Nl::callback_hazard_lights_set_cmd(const pacmod3_msgs::SystemCmdBool::ConstPtr& msg)
{
  lookup_and_encode(HAZARD_LIGHTS_CMD_CANID, msg);
}

void Pacmod3Nl::SystemStatusUpdate(const ros::TimerEvent& event)
{
  pacmod3_msgs::AllSystemStatuses ss_msg;

  std::unique_lock<std::mutex> lock(sys_status_mutex_);
  for (auto system = system_statuses.begin(); system != system_statuses.end(); ++system)
  {
    pacmod3_msgs::KeyValuePair kvp;

    if (system->first == ACCEL_RPT_CANID)
      kvp.key = "Accelerator";
    else if (system->first == BRAKE_RPT_CANID)
      kvp.key = "Brakes";
    else if (system->first == CRUISE_CONTROL_BUTTONS_RPT_CANID)
      kvp.key = "Cruise Control Buttons";
    else if (system->first == HAZARD_LIGHTS_RPT_CANID)
      kvp.key = "Hazard Lights";
    else if (system->first == HEADLIGHT_RPT_CANID)
      kvp.key = "Headlights";
    else if (system->first == HORN_RPT_CANID)
      kvp.key = "Horn";
    else if (system->first == MEDIA_CONTROLS_RPT_CANID)
      kvp.key = "Media Controls";
    else if (system->first == PARKING_BRAKE_RPT_CANID)
      kvp.key = "Parking Brake";
    else if (system->first == SHIFT_RPT_CANID)
      kvp.key = "Shifter";
    else if (system->first == STEERING_RPT_CANID)
      kvp.key = "Steering";
    else if (system->first == TURN_RPT_CANID)
      kvp.key = "Turn Signals";
    else if (system->first == REAR_PASS_DOOR_RPT_CANID)
      kvp.key = "Rear Passenger Door";
    else if (system->first == WIPER_RPT_CANID)
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
  for (const auto& can_id : received_cmds_)
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
  // auto parser_class = Pacmod3TxMsg::make_message(msg->id);
  auto pub = pub_tx_list.find(msg->id);

  if (pub == pub_tx_list.end())
  {
    initializeApiForMsg(msg->id);
  }

  // Only parse messages for which we have a publisher.
  if (pub != pub_tx_list.end())
  {
    // parser_class->parse(const_cast<uint8_t *>(&msg->data[0]));
    handler->ParseAndPublish(*msg, pub->second);

    // TODO: Re-enable this code after refactoring it, or maybe it's not needed at all?
    // if (parser_class->isSystem())
    // {
    //   auto dc_parser = std::dynamic_pointer_cast<SystemRptMsg>(parser_class);

    //   std::unique_lock<std::mutex> lock(sys_status_mutex_);
    //   system_statuses[msg->id] = std::make_tuple(dc_parser->enabled,
    //                              dc_parser->override_active,
    //                              (dc_parser->command_output_fault |
    //                               dc_parser->input_output_fault |
    //                               dc_parser->output_reported_fault |
    //                               dc_parser->pacmod_fault |
    //                               dc_parser->vehicle_fault));
    // }

    if (msg->id == GLOBAL_RPT_CANID)
    {
      pacmod3_msgs::GlobalRpt global_rpt_msg;
      handler->ParseType(*msg, global_rpt_msg);

      std_msgs::Bool bool_msg;
      bool_msg.data = global_rpt_msg.enabled;
      enabled_pub.publish(bool_msg);

      // Auto-disable
      if (global_rpt_msg.override_active ||
          global_rpt_msg.pacmod_sys_fault_active ||
          global_rpt_msg.config_fault_active)
      {
        set_enable(false);
      }
    }
    if (msg->id == GLOBAL_RPT_2_CANID)
    {
      pacmod3_msgs::GlobalRpt2 global_rpt_2_msg;
      handler->ParseType(*msg, global_rpt_2_msg);

      std_msgs::Bool bool_msg;
      bool_msg.data = global_rpt_2_msg.system_enabled;
      enabled_pub.publish(bool_msg);

      // Auto-disable
      if (global_rpt_2_msg.system_override_active ||
          global_rpt_2_msg.system_fault_active)
      {
        set_enable(false);
      }
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
template<class RosMsgType>
void Pacmod3Nl::lookup_and_encode(const uint32_t& can_id, const RosMsgType& msg)
{
  auto rx_it = rx_list.find(can_id);

  if (rx_it != rx_list.end())
  {
    can_msgs::Frame packed_frame = handler->Encode(can_id, msg);

    std::vector<unsigned char> new_data;
    new_data.resize(8);
    std::move(packed_frame.data.begin(), packed_frame.data.end(), new_data.begin());
    new_data.resize(packed_frame.dlc);

    rx_it->second->setData(new_data);
    received_cmds_.insert(can_id);
  }
  else
  {
    ROS_WARN("Received command message for ID 0x%x for which we did not have an encoder.", can_id);
  }
}

}  // namespace pacmod3

PLUGINLIB_EXPORT_CLASS(pacmod3::Pacmod3Nl, nodelet::Nodelet);
