/*
* Unpublished Copyright (c) 2009-2017 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod3 v3 ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include <pacmod3_ros_msg_handler.h>
#include <signal.h>
#include <queue>
#include <thread>
#include <unistd.h>
#include <time.h>
#include <map>
#include <unordered_map>
#include <tuple>

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <can_msgs/Frame.h>

using namespace AS::Drivers::PACMod3;

std::string veh_type_string = "POLARIS_GEM";
VehicleType veh_type = VehicleType::POLARIS_GEM;
std::unordered_map<int64_t, ros::Publisher> pub_tx_list;
Pacmod3TxRosMsgHandler handler;

//Vehicle-Specific Publishers
ros::Publisher cruise_control_buttons_rpt_pub;
ros::Publisher dash_controls_left_rpt_pub;
ros::Publisher dash_controls_right_rpt_pub;
ros::Publisher media_controls_rpt_pub;
ros::Publisher wiper_rpt_pub;
ros::Publisher wiper_aux_rpt_pub;
ros::Publisher headlight_rpt_pub;
ros::Publisher headlight_aux_rpt_pub;
ros::Publisher horn_rpt_pub;
ros::Publisher wheel_speed_rpt_pub;
ros::Publisher steering_pid_rpt_1_pub;
ros::Publisher steering_pid_rpt_2_pub;
ros::Publisher steering_pid_rpt_3_pub;
ros::Publisher steering_pid_rpt_4_pub;
ros::Publisher lat_lon_heading_rpt_pub;
ros::Publisher date_time_rpt_pub;
ros::Publisher parking_brake_rpt_pub;
ros::Publisher yaw_rate_rpt_pub;
ros::Publisher steering_rpt_detail_1_pub;
ros::Publisher steering_rpt_detail_2_pub;
ros::Publisher steering_rpt_detail_3_pub;
ros::Publisher brake_rpt_detail_1_pub;
ros::Publisher brake_rpt_detail_2_pub;
ros::Publisher brake_rpt_detail_3_pub;
ros::Publisher detected_object_rpt_pub;
ros::Publisher vehicle_specific_rpt_1_pub;
ros::Publisher vehicle_dynamics_rpt_pub;
ros::Publisher occupancy_rpt_pub;
ros::Publisher interior_lights_rpt_pub;
ros::Publisher door_rpt_pub;
ros::Publisher rear_lights_rpt_pub;

// Advertise published messages
ros::Publisher global_rpt_pub;
ros::Publisher vin_rpt_pub;
ros::Publisher turn_rpt_pub;
ros::Publisher shift_rpt_pub;
ros::Publisher accel_rpt_pub;
ros::Publisher steer_rpt_pub;
ros::Publisher brake_rpt_pub;
ros::Publisher vehicle_speed_pub;
ros::Publisher vehicle_speed_ms_pub;
ros::Publisher enabled_pub;
ros::Publisher can_rx_pub;
ros::Publisher accel_aux_rpt_pub;
ros::Publisher brake_aux_rpt_pub;
ros::Publisher shift_aux_rpt_pub;
ros::Publisher steer_aux_rpt_pub;
ros::Publisher turn_aux_rpt_pub;
ros::Publisher all_system_statuses_pub;

std::unordered_map<long long, std::shared_ptr<LockedData>> rx_list;
std::map<long long, std::tuple<bool, bool, bool>> system_statuses;

bool global_keep_going = true;
std::mutex keep_going_mut;

// Sets the PACMod3 enable flag through CAN.
void set_enable(bool val)
{
  for (auto rx_it = rx_list.begin(); rx_it != rx_list.end(); rx_it++)
  {
    // This assumes that all data in rx_list are encoded
    // command messages which means the least significant
    // bit in their first byte will be the enable flag.
    std::vector<uint8_t> current_data = rx_it->second->getData();

    if (val)
      current_data[0] |= 0x01; // Enable true
    else
      current_data[0] &= 0xFE; // Enable false

    rx_it->second->setData(current_data);
  }
}

// Looks up the appropriate LockedData and inserts the command info
void lookup_and_encode(const int64_t& can_id, const pacmod_msgs::SystemCmdBool::ConstPtr& msg)
{
  auto rx_it = rx_list.find(can_id);

  if (rx_it != rx_list.end())
    rx_it->second->setData(Pacmod3RxRosMsgHandler::unpackAndEncode(can_id, msg));
  else
    ROS_WARN("Received command message for ID 0x%lx for which we did not have an encoder.", can_id);
}

void lookup_and_encode(const int64_t& can_id, const pacmod_msgs::SystemCmdInt::ConstPtr& msg)
{
  auto rx_it = rx_list.find(can_id);

  if (rx_it != rx_list.end())
    rx_it->second->setData(Pacmod3RxRosMsgHandler::unpackAndEncode(can_id, msg));
  else
    ROS_WARN("Received command message for ID 0x%lx for which we did not have an encoder.", can_id);
}

void lookup_and_encode(const int64_t& can_id, const pacmod_msgs::SystemCmdFloat::ConstPtr& msg)
{
  auto rx_it = rx_list.find(can_id);

  if (rx_it != rx_list.end())
    rx_it->second->setData(Pacmod3RxRosMsgHandler::unpackAndEncode(can_id, msg));
  else
    ROS_WARN("Received command message for ID 0x%lx for which we did not have an encoder.", can_id);
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

//Listens for incoming requests to change the state of the headlights
void callback_headlight_set_cmd(const pacmod_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(HeadlightCmdMsg::CAN_ID, msg);
}

//Listens for incoming requests to change the state of the horn
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
  int64_t can_id = SteerCmdMsg::CAN_ID;
  auto rx_it = rx_list.find(can_id);

  if (rx_it != rx_list.end())
    rx_it->second->setData(Pacmod3RxRosMsgHandler::unpackAndEncode(can_id, msg));
  else
    ROS_WARN("Received command message for ID 0x%lx for which we did not have an encoder.", can_id);
}

// Listens for incoming requests to change the state of the turn signals
void callback_turn_signal_set_cmd(const pacmod_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(TurnSignalCmdMsg::CAN_ID, msg);
}

// Listens for incoming requests to change the state of the windshield wipers
void callback_wiper_set_cmd(const pacmod_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(WiperCmdMsg::CAN_ID, msg);
}

void send_can(long id, const std::vector<unsigned char>& vec)
{
  can_msgs::Frame frame;
  frame.id = id;
  frame.is_rtr = false;
  frame.is_extended = false;
  frame.is_error = false;
  frame.dlc = 8;
  std::copy(vec.begin(), vec.end(), frame.data.begin());

  frame.header.stamp = ros::Time::now();

  can_rx_pub.publish(frame);
}

void can_write()
{
  std::vector<unsigned char> data;

  const std::chrono::milliseconds loop_pause = std::chrono::milliseconds(33);
  const std::chrono::milliseconds inter_msg_pause = std::chrono::milliseconds(1);
  bool keep_going = true;

  //Set local to global value before looping.
  keep_going_mut.lock();
  keep_going = global_keep_going;
  keep_going_mut.unlock();

  while (keep_going)
  {
    // Write all the data that we have received.
    for (const auto& element : rx_list)
    {
      send_can(element.first, element.second->getData());
      std::this_thread::sleep_for(inter_msg_pause);
    }

    std::chrono::system_clock::time_point next_time = std::chrono::system_clock::now();
    next_time += loop_pause;
    std::this_thread::sleep_until(next_time);

    //Set local to global immediately before next loop.
    keep_going_mut.lock();
    keep_going = global_keep_going;
    keep_going_mut.unlock();
  }
}

void can_read(const can_msgs::Frame::ConstPtr &msg)
{
  auto parser_class = Pacmod3TxMsg::make_message(msg->id);
  auto pub = pub_tx_list.find(msg->id);

  // Only parse messages for which we have a parser and a publisher.
  if (parser_class != NULL && pub != pub_tx_list.end())
  {
    parser_class->parse(const_cast<unsigned char *>(&msg->data[0]));
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
  ros::Rate loop_rate(30); //PACMod3 is sending at ~30Hz.

  //Vehicle-Specific Subscribers
  std::shared_ptr<ros::Subscriber> wiper_set_cmd_sub,
                                   headlight_set_cmd_sub,
                                   horn_set_cmd_sub,
                                   cruise_control_buttons_set_cmd_sub,
                                   dash_controls_left_set_cmd_sub,
                                   dash_controls_right_set_cmd_sub,
                                   media_controls_set_cmd_sub;

  // Wait for time to be valid
  while (ros::Time::now().nsec == 0);

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
    else if (veh_type_string == "VEHICLE_4")
      veh_type = VEHICLE_4;
    else if (veh_type_string == "VEHICLE_5")
      veh_type = VEHICLE_5;
    else
    {
      veh_type = VehicleType::POLARIS_GEM;
      ROS_WARN("PACMod3 - An invalid vehicle type was entered. Assuming POLARIS_GEM.");
    }
  }

  // Advertise published messages
  can_rx_pub = n.advertise<can_msgs::Frame>("can_rx", 20);
  global_rpt_pub = n.advertise<pacmod_msgs::GlobalRpt>("parsed_tx/global_rpt", 20);
  accel_rpt_pub = n.advertise<pacmod_msgs::SystemRptFloat>("parsed_tx/accel_rpt", 20);
  brake_rpt_pub = n.advertise<pacmod_msgs::SystemRptFloat>("parsed_tx/brake_rpt", 20);
  shift_rpt_pub = n.advertise<pacmod_msgs::SystemRptInt>("parsed_tx/shift_rpt", 20);
  steer_rpt_pub = n.advertise<pacmod_msgs::SystemRptFloat>("parsed_tx/steer_rpt", 20);
  turn_rpt_pub = n.advertise<pacmod_msgs::SystemRptInt>("parsed_tx/turn_rpt", 20);
  vehicle_speed_pub = n.advertise<pacmod_msgs::VehicleSpeedRpt>("parsed_tx/vehicle_speed_rpt", 20);
  vin_rpt_pub = n.advertise<pacmod_msgs::VinRpt>("parsed_tx/vin_rpt", 5);
  accel_aux_rpt_pub = n.advertise<pacmod_msgs::AccelAuxRpt>("parsed_tx/accel_aux_rpt", 20);
  brake_aux_rpt_pub = n.advertise<pacmod_msgs::BrakeAuxRpt>("parsed_tx/brake_aux_rpt", 20);
  shift_aux_rpt_pub = n.advertise<pacmod_msgs::ShiftAuxRpt>("parsed_tx/shift_aux_rpt", 20);
  steer_aux_rpt_pub = n.advertise<pacmod_msgs::SteerAuxRpt>("parsed_tx/steer_aux_rpt", 20);
  turn_aux_rpt_pub = n.advertise<pacmod_msgs::TurnAuxRpt>("parsed_tx/turn_aux_rpt", 20);

  enabled_pub = n.advertise<std_msgs::Bool>("as_tx/enabled", 20, true);
  vehicle_speed_ms_pub = n.advertise<std_msgs::Float64>("as_tx/vehicle_speed", 20);
  all_system_statuses_pub = n.advertise<pacmod_msgs::AllSystemStatuses>("as_tx/all_system_statuses", 20);

  std::string frame_id = "pacmod";

  //Populate handler list
  pub_tx_list.insert(std::make_pair(GlobalRptMsg::CAN_ID, global_rpt_pub));
  pub_tx_list.insert(std::make_pair(AccelRptMsg::CAN_ID, accel_rpt_pub));
  pub_tx_list.insert(std::make_pair(BrakeRptMsg::CAN_ID, brake_rpt_pub));
  pub_tx_list.insert(std::make_pair(ShiftRptMsg::CAN_ID, shift_rpt_pub));
  pub_tx_list.insert(std::make_pair(SteerRptMsg::CAN_ID, steer_rpt_pub));
  pub_tx_list.insert(std::make_pair(TurnSignalRptMsg::CAN_ID, turn_rpt_pub));
  pub_tx_list.insert(std::make_pair(VehicleSpeedRptMsg::CAN_ID, vehicle_speed_pub));
  pub_tx_list.insert(std::make_pair(VinRptMsg::CAN_ID, vin_rpt_pub));
  pub_tx_list.insert(std::make_pair(AccelAuxRptMsg::CAN_ID, accel_aux_rpt_pub));
  pub_tx_list.insert(std::make_pair(BrakeAuxRptMsg::CAN_ID, brake_aux_rpt_pub));
  pub_tx_list.insert(std::make_pair(ShiftAuxRptMsg::CAN_ID, shift_aux_rpt_pub));
  pub_tx_list.insert(std::make_pair(SteerAuxRptMsg::CAN_ID, steer_aux_rpt_pub));
  pub_tx_list.insert(std::make_pair(TurnAuxRptMsg::CAN_ID, turn_aux_rpt_pub));

  // Subscribe to messages
  ros::Subscriber can_tx_sub = n.subscribe("can_tx", 20, can_read);

  ros::Subscriber accel_cmd_sub = n.subscribe("as_rx/accel_cmd", 20, callback_accel_cmd_sub);
  ros::Subscriber brake_cmd_sub = n.subscribe("as_rx/brake_cmd", 20, callback_brake_cmd_sub);
  ros::Subscriber shift_cmd_sub = n.subscribe("as_rx/shift_cmd", 20, callback_shift_set_cmd);  
  ros::Subscriber steer_cmd_sub = n.subscribe("as_rx/steer_cmd", 20, callback_steer_cmd_sub);
  ros::Subscriber turn_cmd_sub = n.subscribe("as_rx/turn_cmd", 20, callback_turn_signal_set_cmd);  

  // Populate rx list
  std::shared_ptr<LockedData> accel_data(new LockedData);
  std::shared_ptr<LockedData> brake_data(new LockedData);
  std::shared_ptr<LockedData> shift_data(new LockedData);
  std::shared_ptr<LockedData> steer_data(new LockedData);
  std::shared_ptr<LockedData> turn_data(new LockedData);

  rx_list.insert(std::make_pair(AccelCmdMsg::CAN_ID, accel_data));
  rx_list.insert(std::make_pair(BrakeCmdMsg::CAN_ID, brake_data));
  rx_list.insert(std::make_pair(ShiftCmdMsg::CAN_ID, shift_data));
  rx_list.insert(std::make_pair(SteerCmdMsg::CAN_ID, steer_data));
  rx_list.insert(std::make_pair(TurnSignalCmdMsg::CAN_ID, turn_data));

  if (veh_type == VehicleType::POLARIS_GEM ||
      veh_type == VehicleType::POLARIS_RANGER ||
      veh_type == VehicleType::INTERNATIONAL_PROSTAR_122)
  {
    brake_rpt_detail_1_pub = n.advertise<pacmod_msgs::MotorRpt1>("parsed_tx/brake_rpt_detail_1", 20);
    brake_rpt_detail_2_pub = n.advertise<pacmod_msgs::MotorRpt2>("parsed_tx/brake_rpt_detail_2", 20);
    brake_rpt_detail_3_pub = n.advertise<pacmod_msgs::MotorRpt3>("parsed_tx/brake_rpt_detail_3", 20);
    steering_rpt_detail_1_pub = n.advertise<pacmod_msgs::MotorRpt1>("parsed_tx/steer_rpt_detail_1", 20);
    steering_rpt_detail_2_pub = n.advertise<pacmod_msgs::MotorRpt2>("parsed_tx/steer_rpt_detail_2", 20);
    steering_rpt_detail_3_pub = n.advertise<pacmod_msgs::MotorRpt3>("parsed_tx/steer_rpt_detail_3", 20);

    pub_tx_list.insert(std::make_pair(BrakeMotorRpt1Msg::CAN_ID, brake_rpt_detail_1_pub));
    pub_tx_list.insert(std::make_pair(BrakeMotorRpt2Msg::CAN_ID, brake_rpt_detail_2_pub));
    pub_tx_list.insert(std::make_pair(BrakeMotorRpt3Msg::CAN_ID, brake_rpt_detail_3_pub));
    pub_tx_list.insert(std::make_pair(SteerMotorRpt1Msg::CAN_ID, steering_rpt_detail_1_pub));
    pub_tx_list.insert(std::make_pair(SteerMotorRpt2Msg::CAN_ID, steering_rpt_detail_2_pub));
    pub_tx_list.insert(std::make_pair(SteerMotorRpt3Msg::CAN_ID, steering_rpt_detail_3_pub));
  }

  if (veh_type == VehicleType::INTERNATIONAL_PROSTAR_122)
  {
    wiper_rpt_pub = n.advertise<pacmod_msgs::SystemRptInt>("parsed_tx/wiper_rpt", 20);
    wiper_aux_rpt_pub = n.advertise<pacmod_msgs::WiperAuxRpt>("parsed_tx/wiper_aux_rpt", 20);

    pub_tx_list.insert(std::make_pair(WiperRptMsg::CAN_ID, wiper_rpt_pub));
    pub_tx_list.insert(std::make_pair(WiperAuxRptMsg::CAN_ID, wiper_aux_rpt_pub));

    wiper_set_cmd_sub = std::shared_ptr<ros::Subscriber>(new ros::Subscriber(n.subscribe("as_rx/wiper_cmd", 20, callback_wiper_set_cmd)));

    std::shared_ptr<LockedData> wiper_data(new LockedData);
    rx_list.insert(std::make_pair(WiperCmdMsg::CAN_ID, wiper_data));
  }

  if (veh_type == VehicleType::LEXUS_RX_450H ||
      veh_type == VehicleType::VEHICLE_5)
  {
    date_time_rpt_pub = n.advertise<pacmod_msgs::DateTimeRpt>("parsed_tx/date_time_rpt", 20);
    headlight_rpt_pub = n.advertise<pacmod_msgs::SystemRptInt>("parsed_tx/headlight_rpt", 20);
    horn_rpt_pub = n.advertise<pacmod_msgs::SystemRptBool>("parsed_tx/horn_rpt", 20);
    lat_lon_heading_rpt_pub = n.advertise<pacmod_msgs::LatLonHeadingRpt>("parsed_tx/lat_lon_heading_rpt", 20);
    parking_brake_rpt_pub = n.advertise<pacmod_msgs::SystemRptBool>("parsed_tx/parking_brake_status_rpt", 20);
    wheel_speed_rpt_pub = n.advertise<pacmod_msgs::WheelSpeedRpt>("parsed_tx/wheel_speed_rpt", 20);
    yaw_rate_rpt_pub = n.advertise<pacmod_msgs::YawRateRpt>("parsed_tx/yaw_rate_rpt", 20);
    headlight_aux_rpt_pub = n.advertise<pacmod_msgs::HeadlightAuxRpt>("parsed_tx/headlight_aux_rpt", 20);

    pub_tx_list.insert(std::make_pair(DateTimeRptMsg::CAN_ID, date_time_rpt_pub));
    pub_tx_list.insert(std::make_pair(HeadlightRptMsg::CAN_ID, headlight_rpt_pub));
    pub_tx_list.insert(std::make_pair(HornRptMsg::CAN_ID, horn_rpt_pub));
    pub_tx_list.insert(std::make_pair(LatLonHeadingRptMsg::CAN_ID, lat_lon_heading_rpt_pub));
    pub_tx_list.insert(std::make_pair(ParkingBrakeRptMsg::CAN_ID, parking_brake_rpt_pub));
    pub_tx_list.insert(std::make_pair(WheelSpeedRptMsg::CAN_ID, wheel_speed_rpt_pub));
    pub_tx_list.insert(std::make_pair(YawRateRptMsg::CAN_ID, yaw_rate_rpt_pub));
    pub_tx_list.insert(std::make_pair(HeadlightAuxRptMsg::CAN_ID, headlight_aux_rpt_pub));

    headlight_set_cmd_sub = std::shared_ptr<ros::Subscriber>(new ros::Subscriber(n.subscribe("as_rx/headlight_cmd", 20, callback_headlight_set_cmd)));
    horn_set_cmd_sub = std::shared_ptr<ros::Subscriber>(new ros::Subscriber(n.subscribe("as_rx/horn_cmd", 20, callback_horn_set_cmd)));

    std::shared_ptr<LockedData> headlight_data(new LockedData);
    std::shared_ptr<LockedData> horn_data(new LockedData);

    rx_list.insert(std::make_pair(HeadlightCmdMsg::CAN_ID, headlight_data));
    rx_list.insert(std::make_pair(HornCmdMsg::CAN_ID, horn_data));
  }

  if (veh_type == VehicleType::VEHICLE_4)
  {
    detected_object_rpt_pub = n.advertise<pacmod_msgs::DetectedObjectRpt>("parsed_tx/detected_object_rpt", 20);
    vehicle_dynamics_rpt_pub = n.advertise<pacmod_msgs::VehicleDynamicsRpt>("parsed_tx/vehicle_dynamics_rpt", 20);

    pub_tx_list.insert(std::make_pair(DetectedObjectRptMsg::CAN_ID, detected_object_rpt_pub));
    pub_tx_list.insert(std::make_pair(VehicleDynamicsRptMsg::CAN_ID, vehicle_dynamics_rpt_pub));
  }

  if (veh_type == VehicleType::LEXUS_RX_450H)
  {
    steering_pid_rpt_1_pub = n.advertise<pacmod_msgs::SteeringPIDRpt1>("parsed_tx/steer_pid_rpt_1", 20);
    steering_pid_rpt_2_pub = n.advertise<pacmod_msgs::SteeringPIDRpt2>("parsed_tx/steer_pid_rpt_2", 20);
    steering_pid_rpt_3_pub = n.advertise<pacmod_msgs::SteeringPIDRpt3>("parsed_tx/steer_pid_rpt_3", 20);
    steering_pid_rpt_4_pub = n.advertise<pacmod_msgs::SteeringPIDRpt4>("parsed_tx/steer_pid_rpt_4", 20);

    pub_tx_list.insert(std::make_pair(SteeringPIDRpt1Msg::CAN_ID, steering_pid_rpt_1_pub));
    pub_tx_list.insert(std::make_pair(SteeringPIDRpt2Msg::CAN_ID, steering_pid_rpt_2_pub));
    pub_tx_list.insert(std::make_pair(SteeringPIDRpt3Msg::CAN_ID, steering_pid_rpt_3_pub));
    pub_tx_list.insert(std::make_pair(SteeringPIDRpt4Msg::CAN_ID, steering_pid_rpt_4_pub));
  }

  if (veh_type == VehicleType::VEHICLE_5)
  {
    // cruise_control_buttons_rpt_pub = n.advertise<pacmod_msgs::SystemRptInt>("parsed_tx/cruise_control_buttons_rpt", 20);
    // dash_controls_left_rpt_pub = n.advertise<pacmod_msgs::SystemRptInt>("parsed_tx/dash_controls_left_rpt", 20);
    // dash_controls_right_rpt_pub = n.advertise<pacmod_msgs::SystemRptInt>("parsed_tx/dash_controls_right_rpt", 20);
    // media_controls_rpt_pub = n.advertise<pacmod_msgs::SystemRptInt>("parsed_tx/media_controls_rpt", 20);
    occupancy_rpt_pub = n.advertise<pacmod_msgs::OccupancyRpt>("parsed_tx/occupancy_rpt", 20);
    interior_lights_rpt_pub = n.advertise<pacmod_msgs::InteriorLightsRpt>("parsed_tx/interior_lights_rpt", 20);
    door_rpt_pub = n.advertise<pacmod_msgs::DoorRpt>("parsed_tx/door_rpt", 20);
    rear_lights_rpt_pub = n.advertise<pacmod_msgs::RearLightsRpt>("parsed_tx/rear_lights_rpt", 20);

    // pub_tx_list.insert(std::make_pair(CruiseControlButtonsRptMsg::CAN_ID, cruise_control_buttons_rpt_pub));
    // pub_tx_list.insert(std::make_pair(DashControlsLeftRptMsg::CAN_ID, dash_controls_left_rpt_pub));
    // pub_tx_list.insert(std::make_pair(DashControlsRightRptMsg::CAN_ID, dash_controls_right_rpt_pub));
    // pub_tx_list.insert(std::make_pair(MediaControlsRptMsg::CAN_ID, media_controls_rpt_pub));
    pub_tx_list.insert(std::make_pair(OccupancyRptMsg::CAN_ID, occupancy_rpt_pub));
    pub_tx_list.insert(std::make_pair(InteriorLightsRptMsg::CAN_ID, interior_lights_rpt_pub));
    pub_tx_list.insert(std::make_pair(DoorRptMsg::CAN_ID, door_rpt_pub));
    pub_tx_list.insert(std::make_pair(RearLightsRptMsg::CAN_ID, rear_lights_rpt_pub));

    // cruise_control_buttons_set_cmd_sub = std::shared_ptr<ros::Subscriber>(new ros::Subscriber(n.subscribe("as_rx/cruise_control_buttons_cmd", 20, callback_cruise_control_buttons_set_cmd)));
    // dash_controls_left_set_cmd_sub = std::shared_ptr<ros::Subscriber>(new ros::Subscriber(n.subscribe("as_rx/dash_controls_left_cmd", 20, callback_dash_controls_left_set_cmd)));
    // dash_controls_right_set_cmd_sub = std::shared_ptr<ros::Subscriber>(new ros::Subscriber(n.subscribe("as_rx/dash_controls_right_cmd", 20, callback_dash_controls_right_set_cmd)));
    // media_controls_set_cmd_sub = std::shared_ptr<ros::Subscriber>(new ros::Subscriber(n.subscribe("as_rx/media_controls_cmd", 20, callback_media_controls_set_cmd)));

    // std::shared_ptr<LockedData> cruise_control_buttons_data(new LockedData);
    // std::shared_ptr<LockedData> dash_controls_left_data(new LockedData);
    // std::shared_ptr<LockedData> dash_controls_right_data(new LockedData);
    // std::shared_ptr<LockedData> media_controls_data(new LockedData);

    // rx_list.insert(std::make_pair(CruiseControlButtonsCmdMsg::CAN_ID, cruise_control_buttons_data));
    // rx_list.insert(std::make_pair(DashControlsLeftCmdMsg::CAN_ID, dash_controls_left_data));
    // rx_list.insert(std::make_pair(DashControlsRightCmdMsg::CAN_ID, dash_controls_right_data));
    // rx_list.insert(std::make_pair(MediaControlsCmdMsg::CAN_ID, media_controls_data));
  }

  // Initialize rx_list with all 0s
  for (auto rx_it = rx_list.begin(); rx_it != rx_list.end(); rx_it++)
  {
    if (rx_it->first == TurnSignalCmdMsg::CAN_ID)
    {
      // Turn signals have non-0 initial value.
      TurnSignalCmdMsg turn_encoder;
      turn_encoder.encode(false, false, false, pacmod_msgs::SystemCmdInt::TURN_NONE);
      rx_it->second->setData(turn_encoder.data);
    }
    else
    {
      std::vector<uint8_t> empty_vec;
      empty_vec.assign(8, 0);
      rx_it->second->setData(empty_vec);
    }
  }

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

  keep_going_mut.lock();
  global_keep_going = false;
  keep_going_mut.unlock();

  can_write_thread.join();

  return 0;
}

