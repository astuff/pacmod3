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

#include <pacmod3/pacmod3_ros_msg_handler.h>
#include <signal.h>
#include <queue>
#include <thread>
#include <unistd.h>
#include <time.h>
#include <map>
#include <unordered_map>
#include <tuple>
#include <string>
#include <vector>
#include <algorithm>

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <can_msgs/Frame.h>

using namespace AS::Drivers::PACMod3;  // NOLINT

std::unordered_map<uint32_t, ros::Publisher> pub_tx_list;
Pacmod3TxRosMsgHandler handler;

ros::Publisher vehicle_speed_ms_pub;
ros::Publisher enabled_pub;
ros::Publisher can_rx_pub;
ros::Publisher all_system_statuses_pub;

std::unordered_map<uint32_t, std::shared_ptr<LockedData>> rx_list;
std::map<uint32_t, std::tuple<bool, bool, bool>> system_statuses;

constexpr auto SEND_CMD_INTERVAL = std::chrono::milliseconds(33);
constexpr auto INTER_MSG_PAUSE = std::chrono::milliseconds(1);

// Sets the PACMod3 enable flag through CAN.
void set_enable(bool val)
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
template<class T>
void lookup_and_encode(const uint32_t& can_id, const T& msg)
{
  auto rx_it = rx_list.find(can_id);

  if (rx_it != rx_list.end())
    rx_it->second->setData(Pacmod3RxRosMsgHandler::unpackAndEncode(can_id, msg));
  else
    ROS_WARN("Received command message for ID 0x%x for which we did not have an encoder.", can_id);
}

// Listens for incoming requests to change the position of the throttle pedal
void callback_accel_cmd_sub(const pacmod_msgs::SystemCmdFloat::ConstPtr& msg)
{
  lookup_and_encode(AccelCmdMsg::CAN_ID, msg);
}

// Listens for incoming requests to change the position of the brake pedal
void callback_brake_cmd_sub(const pacmod_msgs::SystemCmdFloat::ConstPtr& msg)
{
  lookup_and_encode(BrakeCmdMsg::CAN_ID, msg);
}

void callback_cruise_control_buttons_set_cmd(const pacmod_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(CruiseControlButtonsCmdMsg::CAN_ID, msg);
}

void callback_dash_controls_left_set_cmd(const pacmod_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(DashControlsLeftCmdMsg::CAN_ID, msg);
}

void callback_dash_controls_right_set_cmd(const pacmod_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(DashControlsRightCmdMsg::CAN_ID, msg);
}

// Listens for incoming requests to change the state of the headlights
void callback_headlight_set_cmd(const pacmod_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(HeadlightCmdMsg::CAN_ID, msg);
}

// Listens for incoming requests to change the state of the horn
void callback_horn_set_cmd(const pacmod_msgs::SystemCmdBool::ConstPtr& msg)
{
  lookup_and_encode(HornCmdMsg::CAN_ID, msg);
}

void callback_media_controls_set_cmd(const pacmod_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(MediaControlsCmdMsg::CAN_ID, msg);
}

// Listens for incoming requests to change the gear shifter state
void callback_shift_set_cmd(const pacmod_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(ShiftCmdMsg::CAN_ID, msg);
}

// Listens for incoming requests to change the position of the steering wheel with a speed limit
void callback_steer_cmd_sub(const pacmod_msgs::SteerSystemCmd::ConstPtr& msg)
{
  lookup_and_encode(SteerCmdMsg::CAN_ID, msg);
}

// Listens for incoming requests to change the state of the turn signals
void callback_turn_signal_set_cmd(const pacmod_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(TurnSignalCmdMsg::CAN_ID, msg);
}

// Listens for incoming requests to change the state of the rear-pass-door signals
void callback_rear_pass_door_set_cmd(const pacmod_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(RearPassDoorCmdMsg::CAN_ID, msg);
}

// Listens for incoming requests to change the state of the windshield wipers
void callback_wiper_set_cmd(const pacmod_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(WiperCmdMsg::CAN_ID, msg);
}

void callback_engine_brake_set_cmd(const pacmod_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(EngineBrakeCmdMsg::CAN_ID, msg);
}

void callback_marker_lamp_set_cmd(const pacmod_msgs::SystemCmdBool::ConstPtr& msg)
{
  lookup_and_encode(MarkerLampCmdMsg::CAN_ID, msg);
}

void callback_sprayer_set_cmd(const pacmod_msgs::SystemCmdBool::ConstPtr& msg)
{
  lookup_and_encode(SprayerCmdMsg::CAN_ID, msg);
}

void callback_hazard_lights_set_cmd(const pacmod_msgs::SystemCmdBool::ConstPtr& msg)
{
  lookup_and_encode(HazardLightCmdMsg::CAN_ID, msg);
}

void can_write()
{
  while (ros::ok())
  {
    auto next_time = std::chrono::steady_clock::now() + SEND_CMD_INTERVAL;

    // Write all the data that we have received.
    for (const auto& element : rx_list)
    {
      auto data = element.second->getData();

      can_msgs::Frame frame;
      frame.id = element.first;
      frame.is_rtr = false;
      frame.is_extended = false;
      frame.is_error = false;
      frame.dlc = data.size();
      std::move(data.begin(), data.end(), frame.data.begin());

      frame.header.stamp = ros::Time::now();

      can_rx_pub.publish(frame);

      std::this_thread::sleep_for(INTER_MSG_PAUSE);
    }

    std::this_thread::sleep_until(next_time);
  }
}

void can_read(const can_msgs::Frame::ConstPtr &msg)
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

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pacmod3");
  ros::AsyncSpinner spinner(2);
  ros::NodeHandle n;
  ros::NodeHandle priv("~");
  ros::Rate loop_rate(30);  // PACMod3 is sending at ~30Hz.
  std::string veh_type_string = "POLARIS_GEM";
  VehicleType veh_type = VehicleType::POLARIS_GEM;

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


  // Wait for time to be valid
  ros::Time::waitForValid();

  // Get and validate parameters
  if (priv.getParam("vehicle_type", veh_type_string))
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

  // Advertise published messages
  can_rx_pub = n.advertise<can_msgs::Frame>("can_rx", 20);
  ros::Publisher global_rpt_pub = n.advertise<pacmod_msgs::GlobalRpt>("parsed_tx/global_rpt", 20);
  ros::Publisher component_rpt_pub = n.advertise<pacmod_msgs::ComponentRpt>("parsed_tx/component_rpt", 20);
  ros::Publisher accel_rpt_pub = n.advertise<pacmod_msgs::SystemRptFloat>("parsed_tx/accel_rpt", 20);
  ros::Publisher brake_rpt_pub = n.advertise<pacmod_msgs::SystemRptFloat>("parsed_tx/brake_rpt", 20);
  ros::Publisher shift_rpt_pub = n.advertise<pacmod_msgs::SystemRptInt>("parsed_tx/shift_rpt", 20);
  ros::Publisher steer_rpt_pub = n.advertise<pacmod_msgs::SystemRptFloat>("parsed_tx/steer_rpt", 20);
  ros::Publisher turn_rpt_pub = n.advertise<pacmod_msgs::SystemRptInt>("parsed_tx/turn_rpt", 20);
  ros::Publisher rear_pass_door_rpt_pub = n.advertise<pacmod_msgs::SystemRptInt>("parsed_tx/rear_pass_door_rpt", 20);
  ros::Publisher vehicle_speed_pub = n.advertise<pacmod_msgs::VehicleSpeedRpt>("parsed_tx/vehicle_speed_rpt", 20);
  ros::Publisher vin_rpt_pub = n.advertise<pacmod_msgs::VinRpt>("parsed_tx/vin_rpt", 5);
  ros::Publisher accel_aux_rpt_pub = n.advertise<pacmod_msgs::AccelAuxRpt>("parsed_tx/accel_aux_rpt", 20);
  ros::Publisher brake_aux_rpt_pub = n.advertise<pacmod_msgs::BrakeAuxRpt>("parsed_tx/brake_aux_rpt", 20);
  ros::Publisher shift_aux_rpt_pub = n.advertise<pacmod_msgs::ShiftAuxRpt>("parsed_tx/shift_aux_rpt", 20);
  ros::Publisher steer_aux_rpt_pub = n.advertise<pacmod_msgs::SteerAuxRpt>("parsed_tx/steer_aux_rpt", 20);
  ros::Publisher turn_aux_rpt_pub = n.advertise<pacmod_msgs::TurnAuxRpt>("parsed_tx/turn_aux_rpt", 20);

  enabled_pub = n.advertise<std_msgs::Bool>("as_tx/enabled", 20, true);
  vehicle_speed_ms_pub = n.advertise<std_msgs::Float64>("as_tx/vehicle_speed", 20);
  all_system_statuses_pub = n.advertise<pacmod_msgs::AllSystemStatuses>("as_tx/all_system_statuses", 20);

  std::string frame_id = "pacmod";

  // Populate handler list
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
  pub_tx_list.emplace(SteerAuxRptMsg::CAN_ID, std::move(steer_aux_rpt_pub));
  pub_tx_list.emplace(TurnAuxRptMsg::CAN_ID, std::move(turn_aux_rpt_pub));

  // Subscribe to messages
  ros::Subscriber can_tx_sub = n.subscribe("can_tx", 20, can_read);

  ros::Subscriber accel_cmd_sub = n.subscribe("as_rx/accel_cmd", 20, callback_accel_cmd_sub);
  ros::Subscriber brake_cmd_sub = n.subscribe("as_rx/brake_cmd", 20, callback_brake_cmd_sub);
  ros::Subscriber shift_cmd_sub = n.subscribe("as_rx/shift_cmd", 20, callback_shift_set_cmd);
  ros::Subscriber steer_cmd_sub = n.subscribe("as_rx/steer_cmd", 20, callback_steer_cmd_sub);
  ros::Subscriber turn_cmd_sub = n.subscribe("as_rx/turn_cmd", 20, callback_turn_signal_set_cmd);
  ros::Subscriber rear_pass_door_cmd_sub = n.subscribe("as_rx/rear_pass_door_cmd", 20, callback_rear_pass_door_set_cmd);

  // Populate rx list
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
      veh_type == VehicleType::FREIGHTLINER_CASCADIA)
  {
    ros::Publisher brake_rpt_detail_1_pub = n.advertise<pacmod_msgs::MotorRpt1>("parsed_tx/brake_rpt_detail_1", 20);
    ros::Publisher brake_rpt_detail_2_pub = n.advertise<pacmod_msgs::MotorRpt2>("parsed_tx/brake_rpt_detail_2", 20);
    ros::Publisher brake_rpt_detail_3_pub = n.advertise<pacmod_msgs::MotorRpt3>("parsed_tx/brake_rpt_detail_3", 20);
    ros::Publisher steering_rpt_detail_1_pub = n.advertise<pacmod_msgs::MotorRpt1>("parsed_tx/steer_rpt_detail_1", 20);
    ros::Publisher steering_rpt_detail_2_pub = n.advertise<pacmod_msgs::MotorRpt2>("parsed_tx/steer_rpt_detail_2", 20);
    ros::Publisher steering_rpt_detail_3_pub = n.advertise<pacmod_msgs::MotorRpt3>("parsed_tx/steer_rpt_detail_3", 20);

    pub_tx_list.emplace(BrakeMotorRpt1Msg::CAN_ID, std::move(brake_rpt_detail_1_pub));
    pub_tx_list.emplace(BrakeMotorRpt2Msg::CAN_ID, std::move(brake_rpt_detail_2_pub));
    pub_tx_list.emplace(BrakeMotorRpt3Msg::CAN_ID, std::move(brake_rpt_detail_3_pub));
    pub_tx_list.emplace(SteerMotorRpt1Msg::CAN_ID, std::move(steering_rpt_detail_1_pub));
    pub_tx_list.emplace(SteerMotorRpt2Msg::CAN_ID, std::move(steering_rpt_detail_2_pub));
    pub_tx_list.emplace(SteerMotorRpt3Msg::CAN_ID, std::move(steering_rpt_detail_3_pub));
  }

  if (veh_type == VehicleType::INTERNATIONAL_PROSTAR_122 || veh_type == VehicleType::FREIGHTLINER_CASCADIA)
  {
    ros::Publisher wiper_rpt_pub = n.advertise<pacmod_msgs::SystemRptInt>("parsed_tx/wiper_rpt", 20);
    ros::Publisher wiper_aux_rpt_pub = n.advertise<pacmod_msgs::WiperAuxRpt>("parsed_tx/wiper_aux_rpt", 20);

    pub_tx_list.emplace(WiperRptMsg::CAN_ID, std::move(wiper_rpt_pub));
    pub_tx_list.emplace(WiperAuxRptMsg::CAN_ID, std::move(wiper_aux_rpt_pub));

    wiper_set_cmd_sub = std::make_shared<ros::Subscriber>(n.subscribe("as_rx/wiper_cmd", 20, callback_wiper_set_cmd));

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
      n.advertise<pacmod_msgs::DateTimeRpt>("parsed_tx/date_time_rpt", 20);
    ros::Publisher headlight_rpt_pub =
      n.advertise<pacmod_msgs::SystemRptInt>("parsed_tx/headlight_rpt", 20);
    ros::Publisher horn_rpt_pub =
      n.advertise<pacmod_msgs::SystemRptBool>("parsed_tx/horn_rpt", 20);
    ros::Publisher lat_lon_heading_rpt_pub =
      n.advertise<pacmod_msgs::LatLonHeadingRpt>("parsed_tx/lat_lon_heading_rpt", 20);
    ros::Publisher parking_brake_rpt_pub =
      n.advertise<pacmod_msgs::SystemRptBool>("parsed_tx/parking_brake_status_rpt", 20);
    ros::Publisher wheel_speed_rpt_pub =
      n.advertise<pacmod_msgs::WheelSpeedRpt>("parsed_tx/wheel_speed_rpt", 20);
    ros::Publisher yaw_rate_rpt_pub =
      n.advertise<pacmod_msgs::YawRateRpt>("parsed_tx/yaw_rate_rpt", 20);
    ros::Publisher headlight_aux_rpt_pub =
      n.advertise<pacmod_msgs::HeadlightAuxRpt>("parsed_tx/headlight_aux_rpt", 20);

    pub_tx_list.emplace(DateTimeRptMsg::CAN_ID, std::move(date_time_rpt_pub));
    pub_tx_list.emplace(HeadlightRptMsg::CAN_ID, std::move(headlight_rpt_pub));
    pub_tx_list.emplace(HornRptMsg::CAN_ID, std::move(horn_rpt_pub));
    pub_tx_list.emplace(LatLonHeadingRptMsg::CAN_ID, std::move(lat_lon_heading_rpt_pub));
    pub_tx_list.emplace(ParkingBrakeRptMsg::CAN_ID, std::move(parking_brake_rpt_pub));
    pub_tx_list.emplace(WheelSpeedRptMsg::CAN_ID, std::move(wheel_speed_rpt_pub));
    pub_tx_list.emplace(YawRateRptMsg::CAN_ID, std::move(yaw_rate_rpt_pub));
    pub_tx_list.emplace(HeadlightAuxRptMsg::CAN_ID, std::move(headlight_aux_rpt_pub));

    headlight_set_cmd_sub =
      std::make_shared<ros::Subscriber>(n.subscribe("as_rx/headlight_cmd", 20, callback_headlight_set_cmd));
    horn_set_cmd_sub =
      std::make_shared<ros::Subscriber>(n.subscribe("as_rx/horn_cmd", 20, callback_horn_set_cmd));

    rx_list.emplace(
      HeadlightCmdMsg::CAN_ID,
      std::shared_ptr<LockedData>(new LockedData(HeadlightCmdMsg::DATA_LENGTH)));
    rx_list.emplace(
      HornCmdMsg::CAN_ID,
      std::shared_ptr<LockedData>(new LockedData(HeadlightCmdMsg::DATA_LENGTH)));
  }

  if (veh_type == VehicleType::FREIGHTLINER_CASCADIA)
  {
    ros::Publisher cruise_control_buttons_rpt_pub =
      n.advertise<pacmod_msgs::SystemRptInt>("parsed_tx/cruise_control_buttons_rpt", 20);
    ros::Publisher engine_brake_rpt_pub =
      n.advertise<pacmod_msgs::SystemRptInt>("parsed_tx/engine_brake_rpt", 20);
    ros::Publisher engine_rpt_pub =
      n.advertise<pacmod_msgs::EngineRpt>("parsed_tx/engine_rpt", 20);
    ros::Publisher marker_lamp_rpt_pub =
      n.advertise<pacmod_msgs::SystemRptBool>("parsed_tx/marker_lamp_rpt", 20);
    ros::Publisher sprayer_rpt_pub =
      n.advertise<pacmod_msgs::SystemRptBool>("parsed_tx/sprayer_rpt", 20);
    ros::Publisher hazard_lights_rpt_pub =
      n.advertise<pacmod_msgs::SystemRptBool>("parsed_tx/hazard_lights_rpt", 20);

    pub_tx_list.emplace(CruiseControlButtonsRptMsg::CAN_ID, std::move(cruise_control_buttons_rpt_pub));
    pub_tx_list.emplace(EngineBrakeRptMsg::CAN_ID, std::move(engine_brake_rpt_pub));
    pub_tx_list.emplace(EngineRptMsg::CAN_ID, std::move(engine_rpt_pub));
    pub_tx_list.emplace(MarkerLampRptMsg::CAN_ID, std::move(marker_lamp_rpt_pub));
    pub_tx_list.emplace(SprayerRptMsg::CAN_ID, std::move(sprayer_rpt_pub));
    pub_tx_list.emplace(HazardLightRptMsg::CAN_ID, std::move(hazard_lights_rpt_pub));

    cruise_control_buttons_set_cmd_sub = std::make_shared<ros::Subscriber>(
      n.subscribe("as_rx/cruise_control_buttons_cmd", 20, callback_cruise_control_buttons_set_cmd));
    engine_brake_set_cmd_sub = std::make_shared<ros::Subscriber>(
      n.subscribe("as_rx/engine_brake_cmd", 20, callback_engine_brake_set_cmd));
    marker_lamp_set_cmd_sub = std::make_shared<ros::Subscriber>(
      n.subscribe("as_rx/marker_lamp_cmd", 20, callback_marker_lamp_set_cmd));
    sprayer_set_cmd_sub = std::make_shared<ros::Subscriber>(
      n.subscribe("as_rx/sprayer_cmd", 20, callback_sprayer_set_cmd));
    hazard_lights_set_cmd_sub = std::make_shared<ros::Subscriber>(
      n.subscribe("as_rx/hazard_lights_cmd", 20, callback_hazard_lights_set_cmd));

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
      n.advertise<pacmod_msgs::DetectedObjectRpt>("parsed_tx/detected_object_rpt", 20);
    ros::Publisher vehicle_dynamics_rpt_pub =
      n.advertise<pacmod_msgs::VehicleDynamicsRpt>("parsed_tx/vehicle_dynamics_rpt", 20);

    pub_tx_list.emplace(DetectedObjectRptMsg::CAN_ID, std::move(detected_object_rpt_pub));
    pub_tx_list.emplace(VehicleDynamicsRptMsg::CAN_ID, std::move(vehicle_dynamics_rpt_pub));
  }

  if (veh_type == VehicleType::VEHICLE_5)
  {
    ros::Publisher occupancy_rpt_pub =
      n.advertise<pacmod_msgs::OccupancyRpt>("parsed_tx/occupancy_rpt", 20);
    ros::Publisher interior_lights_rpt_pub =
      n.advertise<pacmod_msgs::InteriorLightsRpt>("parsed_tx/interior_lights_rpt", 20);
    ros::Publisher door_rpt_pub =
      n.advertise<pacmod_msgs::DoorRpt>("parsed_tx/door_rpt", 20);
    ros::Publisher rear_lights_rpt_pub =
      n.advertise<pacmod_msgs::RearLightsRpt>("parsed_tx/rear_lights_rpt", 20);

    pub_tx_list.emplace(OccupancyRptMsg::CAN_ID, std::move(occupancy_rpt_pub));
    pub_tx_list.emplace(InteriorLightsRptMsg::CAN_ID, std::move(interior_lights_rpt_pub));
    pub_tx_list.emplace(DoorRptMsg::CAN_ID, std::move(door_rpt_pub));
    pub_tx_list.emplace(RearLightsRptMsg::CAN_ID, std::move(rear_lights_rpt_pub));
  }

  // Initialize Turn Signal with non-0 value
  TurnSignalCmdMsg turn_encoder;
  turn_encoder.encode(false, false, false, false, pacmod_msgs::SystemCmdInt::TURN_NONE);
  rx_list[TurnSignalCmdMsg::CAN_ID]->setData(std::move(turn_encoder.data));

  // Set initial state
  set_enable(false);

  // Start CAN sending thread.
  std::thread can_write_thread(can_write);
  // Start callback spinner.
  spinner.start();

  while (ros::ok())
  {
    pacmod_msgs::AllSystemStatuses ss_msg;

    for (auto system = system_statuses.begin(); system != system_statuses.end(); ++system)
    {
      pacmod_msgs::KeyValuePair kvp;

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

    loop_rate.sleep();
  }

  // Make sure it's disabled when node shuts down
  set_enable(false);

  can_write_thread.join();

  return 0;
}
