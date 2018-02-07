/*
* Unpublished Copyright (c) 2009-2017 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod3 v3 ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include <pacmod3_ros_msg_handler.h>
#include <signal.h>
#include <queue>
#include <condition_variable>
#include <thread>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <algorithm>
#include <unordered_map>

#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <can_msgs/Frame.h>

using namespace AS::Drivers::PACMod3;

double last_global_rpt_msg_received = 0.0;
const double watchdog_timeout = 0.3;
std::string veh_type_string = "POLARIS_GEM";
VehicleType veh_type = VehicleType::POLARIS_GEM;
std::unordered_map<int64_t, ros::Publisher> pub_tx_list;
Pacmod3TxRosMsgHandler handler;

//Vehicle-Specific Publishers
ros::Publisher wiper_rpt_pub;
ros::Publisher headlight_rpt_pub;
ros::Publisher horn_rpt_pub;
ros::Publisher steer_rpt_2_pub;
ros::Publisher steer_rpt_3_pub;
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

//Vehicle-Specific Subscribers
std::shared_ptr<ros::Subscriber> wiper_set_cmd_sub,
                                 headlight_set_cmd_sub,
                                 horn_set_cmd_sub;

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

// System encoders
AccelCmdMsg accel_encoder;
BrakeCmdMsg brake_encoder;
HornCmdMsg horn_encoder;
ShiftCmdMsg shift_encoder;
SteerCmdMsg steer_encoder;
TurnSignalCmdMsg turn_encoder;
WiperCmdMsg wiper_encoder;

std::vector<Pacmod3RxMsg> encoders;

std::unordered_map<long long, std::shared_ptr<LockedData>> rx_list;
std::unordered_map<long long, long long> rpt_cmd_list;

bool enable_state = false;
std::mutex enable_mut;
bool override_state = false;
std::mutex override_mut;

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

  for (auto encoder = encoders.begin(); encoder != encoders.end(); ++encoder)
  {
    encoder->recent_state_change = true;
    encoder->state_change_debounce_cnt = 0;
  }
}

// Listens for incoming requests to enable the PACMod3
void callback_pacmod_enable(const std_msgs::Bool::ConstPtr& msg)
{
  set_enable(msg->data);
}

// Listens for incoming requests to change the position of the throttle pedal
void callback_accel_cmd_sub(const pacmod_msgs::SystemCmdFloat::ConstPtr& msg)
{
  int64_t can_id = AccelCmdMsg::CAN_ID;
  auto rx_it = rx_list.find(can_id);

  if (rx_it != rx_list.end())
  {
    rx_it->second->setData(Pacmod3RxRosMsgHandler::unpackAndEncode(can_id, msg));
  }
  else
  {
    ROS_WARN("Received command message for ID 0x%lx for which we did not have an encoder.", can_id);
  }
}

// Listens for incoming requests to change the position of the brake pedal
void callback_brake_cmd_sub(const pacmod_msgs::SystemCmdFloat::ConstPtr& msg)
{
  int64_t can_id = BrakeCmdMsg::CAN_ID;
  auto rx_it = rx_list.find(can_id);

  if (rx_it != rx_list.end())
  {
    rx_it->second->setData(Pacmod3RxRosMsgHandler::unpackAndEncode(can_id, msg));
  }
  else
  {
    ROS_WARN("Received command message for ID 0x%lx for which we did not have an encoder.", can_id);
  }
}

//Listens for incoming requests to change the state of the headlights
void callback_headlight_set_cmd(const pacmod_msgs::SystemCmdInt::ConstPtr& msg)
{
  int64_t can_id = HeadlightCmdMsg::CAN_ID;
  auto rx_it = rx_list.find(can_id);

  if (rx_it != rx_list.end())
  {
    rx_it->second->setData(Pacmod3RxRosMsgHandler::unpackAndEncode(can_id, msg));
  }
  else
  {
    ROS_WARN("Received command message for ID 0x%lx for which we did not have an encoder.", can_id);
  }
}

//Listens for incoming requests to change the state of the horn
void callback_horn_set_cmd(const pacmod_msgs::SystemCmdInt::ConstPtr& msg)
{
  int64_t can_id = HornCmdMsg::CAN_ID;
  auto rx_it = rx_list.find(can_id);

  if (rx_it != rx_list.end())
  {
    rx_it->second->setData(Pacmod3RxRosMsgHandler::unpackAndEncode(can_id, msg));
  }
  else
  {
    ROS_WARN("Received command message for ID 0x%lx for which we did not have an encoder.", can_id);
  }
}

// Listens for incoming requests to change the gear shifter state
void callback_shift_set_cmd(const pacmod_msgs::SystemCmdInt::ConstPtr& msg)
{
  int64_t can_id = ShiftCmdMsg::CAN_ID;
  auto rx_it = rx_list.find(can_id);

  if (rx_it != rx_list.end())
  {
    rx_it->second->setData(Pacmod3RxRosMsgHandler::unpackAndEncode(can_id, msg));
  }
  else
  {
    ROS_WARN("Received command message for ID 0x%lx for which we did not have an encoder.", can_id);
  }
}

// Listens for incoming requests to change the position of the steering wheel with a speed limit
void callback_steer_cmd_sub(const pacmod_msgs::SteerSystemCmd::ConstPtr& msg)
{
  int64_t can_id = SteerCmdMsg::CAN_ID;
  auto rx_it = rx_list.find(can_id);

  if (rx_it != rx_list.end())
  {
    rx_it->second->setData(Pacmod3RxRosMsgHandler::unpackAndEncode(can_id, msg));
  }
  else
  {
    ROS_WARN("Received command message for ID 0x%lx for which we did not have an encoder.", can_id);
  }
}

// Listens for incoming requests to change the state of the turn signals
void callback_turn_signal_set_cmd(const pacmod_msgs::SystemCmdInt::ConstPtr& msg)
{
  int64_t can_id = TurnSignalCmdMsg::CAN_ID;
  auto rx_it = rx_list.find(can_id);

  if (rx_it != rx_list.end())
  {
    rx_it->second->setData(Pacmod3RxRosMsgHandler::unpackAndEncode(can_id, msg));
  }
  else
  {
    ROS_WARN("Received command message for ID 0x%lx for which we did not have an encoder.", can_id);
  }
}

// Listens for incoming requests to change the state of the windshield wipers
void callback_wiper_set_cmd(const pacmod_msgs::SystemCmdInt::ConstPtr& msg)
{
  int64_t can_id = WiperCmdMsg::CAN_ID;
  auto rx_it = rx_list.find(can_id);

  if (rx_it != rx_list.end())
  {
    rx_it->second->setData(Pacmod3RxRosMsgHandler::unpackAndEncode(can_id, msg));
  }
  else
  {
    ROS_WARN("Received command message for ID 0x%lx for which we did not have an encoder.", can_id);
  }
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
  // Update the state change debounce counts.
  for (auto encoder = encoders.begin(); encoder != encoders.end(); ++encoder)
  {
    if (encoder->recent_state_change)
    {
      encoder->state_change_debounce_cnt++;
      
      if (encoder->state_change_debounce_cnt > STATE_CHANGE_DEBOUNCE_THRESHOLD)
        encoder->recent_state_change = false;
    }
  }

  std_msgs::Bool bool_pub_msg;
  auto parser_class = Pacmod3TxMsg::make_message(msg->id);
  auto pub = pub_tx_list.find(msg->id);

  // Only parse messages for which we have a parser and a publisher.
  if (parser_class != NULL && pub != pub_tx_list.end())
  {
    parser_class->parse(const_cast<unsigned char *>(&msg->data[0]));
    handler.fillAndPublish(msg->id, "pacmod", pub->second, parser_class);

    if (msg->id == GlobalRptMsg::CAN_ID)
    {
      auto dc_parser = std::dynamic_pointer_cast<GlobalRptMsg>(parser_class);

      bool_pub_msg.data = (dc_parser->enabled);
      enabled_pub.publish(bool_pub_msg);

      if (dc_parser->override_active)
        set_enable(false);

      enable_mut.lock();
      enable_state = dc_parser->enabled;
      enable_mut.unlock();
    }
    else if (msg->id == VehicleSpeedRptMsg::CAN_ID)
    {
      auto dc_parser = std::dynamic_pointer_cast<VehicleSpeedRptMsg>(parser_class);

      // Now publish by itself
      std_msgs::Float64 veh_spd_ms_msg;
      veh_spd_ms_msg.data = (dc_parser->vehicle_speed)*0.44704;
      vehicle_speed_ms_pub.publish(veh_spd_ms_msg);
    }
  
    // If a system is disabled, set that system's command
    // to be the current report value. This ensures that
    // when we enable, we are in the same state as the vehicle.

    // Find the cmd value for this rpt.
    auto cmd = rpt_cmd_list.find(msg->id);

    //TODO: This assumes that the driver is the only thing sending
    //commands to PACMod Systems. Someday, fix this.
    if (cmd != rpt_cmd_list.end())
    {
      // Find the data we need to set.
      auto rx_it = rx_list.find(cmd->second);

      if (rx_it != rx_list.end())
      {
        bool cmd_says_enabled = ((rx_it->second->getData()[0] & 0x01) > 0);
        bool cmd_says_ignore_overrides = ((rx_it->second->getData()[0] & 0x02) > 0);

        if (msg->id == AccelRptMsg::CAN_ID)
        {
          auto dc_parser = std::dynamic_pointer_cast<AccelRptMsg>(parser_class);

          if (dc_parser->enabled != cmd_says_enabled &&
              !accel_encoder.recent_state_change)
          {
            accel_encoder.encode(dc_parser->enabled, cmd_says_ignore_overrides, dc_parser->output);
            rx_it->second->setData(accel_encoder.data);
            accel_encoder.recent_state_change = true;
            accel_encoder.state_change_debounce_cnt = 0;
          }
          else
          {
            accel_encoder.encode(cmd_says_enabled, cmd_says_ignore_overrides, dc_parser->output);
            rx_it->second->setData(accel_encoder.data);
          }
        }
        else if (msg->id == BrakeRptMsg::CAN_ID)
        {
          auto dc_parser = std::dynamic_pointer_cast<BrakeRptMsg>(parser_class);

          if (dc_parser->enabled != cmd_says_enabled &&
              !brake_encoder.recent_state_change)
          {
            brake_encoder.encode(dc_parser->enabled, cmd_says_ignore_overrides, dc_parser->output);
            rx_it->second->setData(brake_encoder.data);
            brake_encoder.recent_state_change = true;
            brake_encoder.state_change_debounce_cnt = 0;
          }
          else
          {
            brake_encoder.encode(cmd_says_enabled, cmd_says_ignore_overrides, dc_parser->output);
            rx_it->second->setData(brake_encoder.data);
          }
        }
        else if (msg->id == HornRptMsg::CAN_ID)
        {
          auto dc_parser = std::dynamic_pointer_cast<HornRptMsg>(parser_class);

          if (dc_parser->enabled != cmd_says_enabled &&
              !horn_encoder.recent_state_change)
          {
            horn_encoder.encode(dc_parser->enabled, cmd_says_ignore_overrides, dc_parser->output);
            rx_it->second->setData(horn_encoder.data);
            horn_encoder.recent_state_change = true;
            horn_encoder.state_change_debounce_cnt = 0;
          }
          else
          {
            horn_encoder.encode(cmd_says_enabled, cmd_says_ignore_overrides, dc_parser->output);
            rx_it->second->setData(horn_encoder.data);
          }
        }
        else if (msg->id == ShiftRptMsg::CAN_ID)
        {
          auto dc_parser = std::dynamic_pointer_cast<ShiftRptMsg>(parser_class);

          if (dc_parser->enabled != cmd_says_enabled &&
              !shift_encoder.recent_state_change)
          {
            shift_encoder.encode(dc_parser->enabled, cmd_says_ignore_overrides, dc_parser->output);
            rx_it->second->setData(shift_encoder.data);
            shift_encoder.recent_state_change = true;
            shift_encoder.state_change_debounce_cnt = 0;
          }
          else
          {
            shift_encoder.encode(cmd_says_enabled, cmd_says_ignore_overrides, dc_parser->output);
            rx_it->second->setData(shift_encoder.data);
          }
        }
        else if (msg->id == SteerRptMsg::CAN_ID)
        {
          auto dc_parser = std::dynamic_pointer_cast<SteerRptMsg>(parser_class);
          int16_t raw_turn_rate = ((int16_t)rx_it->second->getData()[3] << 8) | rx_it->second->getData()[4];
          float cmd_turn_rate = ((float)raw_turn_rate * 1000.0);

          if (dc_parser->enabled != cmd_says_enabled &&
              !steer_encoder.recent_state_change)
          {
            steer_encoder.encode(dc_parser->enabled, cmd_says_ignore_overrides, dc_parser->output, cmd_turn_rate);
            rx_it->second->setData(steer_encoder.data);
            steer_encoder.recent_state_change = true;
            steer_encoder.state_change_debounce_cnt = 0;
          }
          else
          {
            steer_encoder.encode(cmd_says_enabled, cmd_says_ignore_overrides, dc_parser->output, cmd_turn_rate);
            rx_it->second->setData(steer_encoder.data);
          }
        }
        else if (msg->id == TurnSignalRptMsg::CAN_ID)
        {
          auto dc_parser = std::dynamic_pointer_cast<TurnSignalRptMsg>(parser_class);

          if (dc_parser->enabled != cmd_says_enabled &&
              !turn_encoder.recent_state_change)
          {
            turn_encoder.encode(dc_parser->enabled, cmd_says_ignore_overrides, dc_parser->output);
            rx_it->second->setData(turn_encoder.data);
            turn_encoder.recent_state_change = true;
            turn_encoder.state_change_debounce_cnt = 0;
          }
          else
          {
            turn_encoder.encode(cmd_says_enabled, cmd_says_ignore_overrides, dc_parser->output);
            rx_it->second->setData(turn_encoder.data);
          }
        }
        else if (msg->id == WiperRptMsg::CAN_ID)
        {
          auto dc_parser = std::dynamic_pointer_cast<WiperRptMsg>(parser_class);

          if (dc_parser->enabled != cmd_says_enabled &&
              !wiper_encoder.recent_state_change)
          {
            wiper_encoder.encode(dc_parser->enabled, cmd_says_ignore_overrides, dc_parser->output);
            rx_it->second->setData(wiper_encoder.data);
            wiper_encoder.recent_state_change = true;
            wiper_encoder.state_change_debounce_cnt = 0;
          }
          else
          {
            wiper_encoder.encode(cmd_says_enabled, cmd_says_ignore_overrides, dc_parser->output);
            rx_it->second->setData(wiper_encoder.data);
          }
        }
      }
    }
  }
}

int main(int argc, char *argv[])
{ 
  ros::init(argc, argv, "pacmod3");
  ros::AsyncSpinner spinner(2);
  ros::NodeHandle n;
  ros::NodeHandle priv("~");
  ros::Rate loop_rate(1.0); //PACMod3 is sending at ~30Hz.

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
    else
    {
      veh_type = VehicleType::POLARIS_GEM;
      ROS_WARN("PACMod3 - An invalid vehicle type was entered. Assuming POLARIS_GEM.");
    }
  }

  // Load up the encoders vector
  encoders.push_back(accel_encoder);
  encoders.push_back(brake_encoder);
  encoders.push_back(horn_encoder);
  encoders.push_back(shift_encoder);
  encoders.push_back(steer_encoder);
  encoders.push_back(turn_encoder);
  encoders.push_back(wiper_encoder);

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

  enabled_pub = n.advertise<std_msgs::Bool>("as_tx/enabled", 20, true);
  vehicle_speed_ms_pub = n.advertise<std_msgs::Float64>("as_tx/vehicle_speed", 20);

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

  // Subscribe to messages
  ros::Subscriber can_tx_sub = n.subscribe("can_tx", 20, can_read);
  ros::Subscriber enable_sub = n.subscribe("as_rx/enable", 20, callback_pacmod_enable);

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

    pub_tx_list.insert(std::make_pair(WiperRptMsg::CAN_ID, wiper_rpt_pub));

    wiper_set_cmd_sub = std::shared_ptr<ros::Subscriber>(new ros::Subscriber(n.subscribe("as_rx/wiper_cmd", 20, callback_wiper_set_cmd)));

    std::shared_ptr<LockedData> wiper_data(new LockedData);
    rx_list.insert(std::make_pair(WiperCmdMsg::CAN_ID, wiper_data));
  }

  if (veh_type == VehicleType::LEXUS_RX_450H)
  {
    date_time_rpt_pub = n.advertise<pacmod_msgs::DateTimeRpt>("parsed_tx/date_time_rpt", 20);
    headlight_rpt_pub = n.advertise<pacmod_msgs::SystemRptInt>("parsed_tx/headlight_rpt", 20);
    horn_rpt_pub = n.advertise<pacmod_msgs::SystemRptBool>("parsed_tx/horn_rpt", 20);
    lat_lon_heading_rpt_pub = n.advertise<pacmod_msgs::LatLonHeadingRpt>("parsed_tx/lat_lon_heading_rpt", 20);
    parking_brake_rpt_pub = n.advertise<pacmod_msgs::SystemRptBool>("parsed_tx/parking_brake_status_rpt", 20);
    steer_rpt_2_pub = n.advertise<pacmod_msgs::SystemRptFloat>("parsed_tx/steer_rpt_2", 20);
    steer_rpt_3_pub = n.advertise<pacmod_msgs::SystemRptFloat>("parsed_tx/steer_rpt_3", 20);
    steering_pid_rpt_1_pub = n.advertise<pacmod_msgs::SteeringPIDRpt1>("parsed_tx/steer_pid_rpt_1", 20);
    steering_pid_rpt_2_pub = n.advertise<pacmod_msgs::SteeringPIDRpt2>("parsed_tx/steer_pid_rpt_2", 20);
    steering_pid_rpt_3_pub = n.advertise<pacmod_msgs::SteeringPIDRpt3>("parsed_tx/steer_pid_rpt_3", 20);
    steering_pid_rpt_4_pub = n.advertise<pacmod_msgs::SteeringPIDRpt4>("parsed_tx/steer_pid_rpt_4", 20);
    wheel_speed_rpt_pub = n.advertise<pacmod_msgs::WheelSpeedRpt>("parsed_tx/wheel_speed_rpt", 20);
    yaw_rate_rpt_pub = n.advertise<pacmod_msgs::YawRateRpt>("parsed_tx/yaw_rate_rpt", 20);

    pub_tx_list.insert(std::make_pair(DateTimeRptMsg::CAN_ID, date_time_rpt_pub));
    pub_tx_list.insert(std::make_pair(HeadlightRptMsg::CAN_ID, headlight_rpt_pub));
    pub_tx_list.insert(std::make_pair(HornRptMsg::CAN_ID, horn_rpt_pub));
    pub_tx_list.insert(std::make_pair(LatLonHeadingRptMsg::CAN_ID, lat_lon_heading_rpt_pub));
    pub_tx_list.insert(std::make_pair(ParkingBrakeRptMsg::CAN_ID, parking_brake_rpt_pub));
    pub_tx_list.insert(std::make_pair(SteerRpt2Msg::CAN_ID, steer_rpt_2_pub));
    pub_tx_list.insert(std::make_pair(SteerRpt3Msg::CAN_ID, steer_rpt_3_pub));
    pub_tx_list.insert(std::make_pair(SteeringPIDRpt1Msg::CAN_ID, steering_pid_rpt_1_pub));
    pub_tx_list.insert(std::make_pair(SteeringPIDRpt2Msg::CAN_ID, steering_pid_rpt_2_pub));
    pub_tx_list.insert(std::make_pair(SteeringPIDRpt3Msg::CAN_ID, steering_pid_rpt_3_pub));
    pub_tx_list.insert(std::make_pair(SteeringPIDRpt4Msg::CAN_ID, steering_pid_rpt_4_pub));
    pub_tx_list.insert(std::make_pair(WheelSpeedRptMsg::CAN_ID, wheel_speed_rpt_pub));
    pub_tx_list.insert(std::make_pair(YawRateRptMsg::CAN_ID, yaw_rate_rpt_pub));

    headlight_set_cmd_sub = std::shared_ptr<ros::Subscriber>(new ros::Subscriber(n.subscribe("as_rx/headlight_cmd", 20, callback_headlight_set_cmd)));
    horn_set_cmd_sub = std::shared_ptr<ros::Subscriber>(new ros::Subscriber(n.subscribe("as_rx/horn_cmd", 20, callback_horn_set_cmd)));

    std::shared_ptr<LockedData> headlight_data(new LockedData);
    std::shared_ptr<LockedData> horn_data(new LockedData);

    rx_list.insert(std::make_pair(HeadlightCmdMsg::CAN_ID, headlight_data));
    rx_list.insert(std::make_pair(HornCmdMsg::CAN_ID, horn_data));
  }

  // Populate report/command list.
  rpt_cmd_list.insert(std::make_pair(AccelRptMsg::CAN_ID, AccelCmdMsg::CAN_ID));
  rpt_cmd_list.insert(std::make_pair(BrakeRptMsg::CAN_ID, BrakeCmdMsg::CAN_ID));
  rpt_cmd_list.insert(std::make_pair(TurnSignalRptMsg::CAN_ID, TurnSignalCmdMsg::CAN_ID));
  rpt_cmd_list.insert(std::make_pair(ShiftRptMsg::CAN_ID, ShiftCmdMsg::CAN_ID));
  rpt_cmd_list.insert(std::make_pair(SteerRptMsg::CAN_ID, SteerCmdMsg::CAN_ID));

  if (veh_type == VehicleType::INTERNATIONAL_PROSTAR_122)
  {
    rpt_cmd_list.insert(std::make_pair(WiperRptMsg::CAN_ID, WiperCmdMsg::CAN_ID));
  }
  else if (veh_type == VehicleType::LEXUS_RX_450H)
  {
    rpt_cmd_list.insert(std::make_pair(HornRptMsg::CAN_ID, HornCmdMsg::CAN_ID));
  }

  // Initialize rx_list with all 0s
  for (auto rx_it = rx_list.begin(); rx_it != rx_list.end(); rx_it++)
  {
    std::vector<uint8_t> empty_vec;
    empty_vec.assign(8, 0);
    rx_it->second->setData(empty_vec);
  }

  // Set initial state
  set_enable(false);
    
  // Start CAN sending thread.
  std::thread can_write_thread(can_write);
  // Start callback spinner.
  spinner.start();

  ros::waitForShutdown();

  // Make sure it's disabled when node shuts down
  set_enable(false);

  keep_going_mut.lock();
  global_keep_going = false;
  keep_going_mut.unlock();

  can_write_thread.join();

  return 0;
}

