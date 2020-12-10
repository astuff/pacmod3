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

void callback_global_cmd_sub(const pacmod_msgs::GlobalCmd::ConstPtr& msg)
{
  lookup_and_encode(GlobalCmdMsg::CAN_ID, msg);
}

void callback_supervisory_ctrl_cmd_sub(const pacmod_msgs::SupervisoryCtrl::ConstPtr& msg)
{
  lookup_and_encode(SupervisoryCtrlMsg::CAN_ID, msg);
}

void callback_user_notification_set_cmd(const pacmod_msgs::NotificationCmd::ConstPtr& msg)
{
  lookup_and_encode(UserNotificationCmdMsg::CAN_ID, msg);
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

void callback_brake_deccel_cmd_sub(const pacmod_msgs::BrakeDeccelCmd::ConstPtr& msg)
{
  lookup_and_encode(BrakeDeccelCmdMsg::CAN_ID, msg);
}

void callback_cabin_climate_set_cmd(const pacmod_msgs::CabinClimateCmd::ConstPtr& msg)
{
  lookup_and_encode(CabinClimateCmdMsg::CAN_ID, msg);
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

void callback_engine_brake_set_cmd(const pacmod_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(EngineBrakeCmdMsg::CAN_ID, msg);
}

void callback_hazard_lights_set_cmd(const pacmod_msgs::SystemCmdBool::ConstPtr& msg)
{
  lookup_and_encode(HazardLightCmdMsg::CAN_ID, msg);
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

void callback_marker_lamp_set_cmd(const pacmod_msgs::SystemCmdBool::ConstPtr& msg)
{
  lookup_and_encode(MarkerLampCmdMsg::CAN_ID, msg);
}

void callback_media_controls_set_cmd(const pacmod_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(MediaControlsCmdMsg::CAN_ID, msg);
}

void callback_parking_brake_cmd(const pacmod_msgs::SystemCmdBool::ConstPtr& msg)
{
  lookup_and_encode(ParkingBrakeCmdMsg::CAN_ID, msg);
}

// Listens for incoming requests to change the state of the rear-pass-door signals
void callback_rear_pass_door_set_cmd(const pacmod_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(RearPassDoorCmdMsg::CAN_ID, msg);
}

void callback_safety_brake_set_cmd(const pacmod_msgs::SafetyBrakeCmd::ConstPtr& msg)
{
  lookup_and_encode(SafetyBrakeCmdMsg::CAN_ID, msg);
}

void callback_safety_func_set_cmd(const pacmod_msgs::SafetyFuncCmd::ConstPtr& msg)
{
  lookup_and_encode(SafetyFuncCmdMsg::CAN_ID, msg);
}

// Listens for incoming requests to change the gear shifter state
void callback_shift_set_cmd(const pacmod_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(ShiftCmdMsg::CAN_ID, msg);
}

void callback_sprayer_set_cmd(const pacmod_msgs::SystemCmdBool::ConstPtr& msg)
{
  lookup_and_encode(SprayerCmdMsg::CAN_ID, msg);
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

// Listens for incoming requests to change the state of the windshield wipers
void callback_wiper_set_cmd(const pacmod_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(WiperCmdMsg::CAN_ID, msg);
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
                                  dc_parser->vehicle_fault |
                                  dc_parser->command_timeout));
    }

    if (msg->id == GlobalRptMsg::CAN_ID)
    {
      auto dc_parser = std::dynamic_pointer_cast<GlobalRptMsg>(parser_class);

      std_msgs::Bool bool_pub_msg;
      bool_pub_msg.data = (dc_parser->enabled);
      enabled_pub.publish(bool_pub_msg);

      if (dc_parser->override_active ||
          dc_parser->pacmod_sys_fault_active)
        set_enable(false);
    }
    else if (msg->id == GlobalRpt2Msg::CAN_ID)
    {
      auto dc_parser = std::dynamic_pointer_cast<GlobalRpt2Msg>(parser_class);

      std_msgs::Bool bool_pub_msg;
      bool_pub_msg.data = (dc_parser->system_enabled);
      enabled_pub.publish(bool_pub_msg);

      if (dc_parser->system_override_active ||
          dc_parser->system_fault_active)
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
  bool safety_ecu_flag = false;
  VehicleType veh_type = VehicleType::POLARIS_GEM;
  ReportPresent reports_present{0,0,0,0,0,0,0,0,0};
  CommandPresent commands_present{0,0,0};

  // Vehicle-Specific Subscribers
  std::shared_ptr<ros::Subscriber>
      global_cmd_sub,
      user_notification_cmd_sub,

      accel_cmd_sub,
      brake_cmd_sub,
      shift_cmd_sub,
      steer_cmd_sub,
      cabin_climate_set_cmd_sub, 
      cruise_control_buttons_set_cmd_sub,
      dash_controls_left_set_cmd_sub,
      dash_controls_right_set_cmd_sub,
      engine_brake_set_cmd_sub,
      hazard_lights_set_cmd_sub,
      headlight_set_cmd_sub,
      horn_set_cmd_sub,
      marker_lamp_set_cmd_sub,
      media_controls_set_cmd_sub,
      parking_brake_set_cmd_sub,
      sprayer_set_cmd_sub,
      rear_pass_door_cmd_sub,
      turn_cmd_sub,
      wiper_set_cmd_sub,
      
      safety_func_cmd_sub,
      supervisory_ctrl_cmd_sub,
      safety_brake_cmd_sub;


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
    else if (veh_type_string == "HEXAGON_TRACTOR")
      veh_type = HEXAGON_TRACTOR;
    else if (veh_type_string == "FORD_RANGER")
      veh_type = FORD_RANGER;
    else if (veh_type_string == "VEHICLE_FTT")
      veh_type = VEHICLE_FTT;
    else if (veh_type_string == "VEHICLE_HCV")
      veh_type = VEHICLE_HCV;
    else
    {
      veh_type = VehicleType::POLARIS_GEM;
      ROS_WARN("PACMod3 - An invalid vehicle type was entered. Assuming POLARIS_GEM.");
    }
  }

  if (priv.getParam("use_safety_ecu", safety_ecu_flag))
  {
    if(safety_ecu_flag)
      ROS_INFO("PACMod3 - Safety ECU is present in this system");
    else
      ROS_INFO("PACMod3 - Safety ECU is NOT present in this system");    
  }

  // Set vehicle based messages
  switch (veh_type)
  {
    case VehicleType::FREIGHTLINER_CASCADIA:
      reports_present.GLOBAL = 1;
      reports_present.ACCEL = 1;
      reports_present.BRAKE = 1;
      reports_present.SHIFT = 1;
      reports_present.STEER = 1;
      reports_present.ACCEL_AUX = 1;
      reports_present.BRAKE_AUX = 1;
      reports_present.SHIFT_AUX = 1;
      reports_present.STEER_AUX = 1;
      reports_present.BRAKE_MOTOR_1 = 1;
      reports_present.BRAKE_MOTOR_2 = 1;
      reports_present.BRAKE_MOTOR_3 = 1;
      reports_present.STEER_MOTOR_1 = 1;
      reports_present.STEER_MOTOR_2 = 1;
      reports_present.STEER_MOTOR_3 = 1;
      reports_present.CRUISE_CONTROL = 1;
      reports_present.ENGINE_BRAKE = 1;
      reports_present.HAZARDS = 1;
      reports_present.HEADLIGHTS = 1;
      reports_present.HEADLIGHTS_AUX = 1;
      reports_present.HORN = 1;
      reports_present.MARKER_LAMP = 1;
      reports_present.PARKING_BRAKE = 1;
      reports_present.REAR_PASS_DOOR = 1;
      reports_present.SPRAY = 1;
      reports_present.TURN = 1;
      reports_present.TURN_AUX = 1;
      reports_present.WIPER = 1;
      reports_present.WIPER_AUX = 1;
      reports_present.DATE_TIME = 1;
      reports_present.ENGINE = 1;
      reports_present.LAT_LON_HEADING = 1;
      reports_present.WHEEL_SPEED = 1;
      reports_present.YAW_RATE = 1;

      reports_present.VEHICLE_SPEED = 1;
      reports_present.VIN = 1;

      commands_present.ACCEL = 1;
      commands_present.BRAKE = 1;
      commands_present.SHIFT = 1;
      commands_present.STEER = 1;
      commands_present.CRUISE_CONTROL = 1;
      commands_present.ENGINE_BRAKE = 1;
      commands_present.HAZARDS = 1;
      commands_present.HEADLIGHTS = 1;
      commands_present.HORN = 1;
      commands_present.MARKER_LAMP = 1;
      commands_present.REAR_PASS_DOOR = 1;
      commands_present.SPRAY = 1;
      commands_present.TURN = 1;
      commands_present.WIPER = 1;
      break;

    case VehicleType::INTERNATIONAL_PROSTAR_122:
      reports_present.GLOBAL = 1;
      reports_present.ACCEL = 1;
      reports_present.BRAKE = 1;
      reports_present.SHIFT = 1;
      reports_present.STEER = 1;
      reports_present.ACCEL_AUX = 1;
      reports_present.BRAKE_AUX = 1;
      reports_present.SHIFT_AUX = 1;
      reports_present.STEER_AUX = 1;
      reports_present.BRAKE_MOTOR_1 = 1;
      reports_present.BRAKE_MOTOR_2 = 1;
      reports_present.BRAKE_MOTOR_3 = 1;
      reports_present.STEER_MOTOR_1 = 1;
      reports_present.STEER_MOTOR_2 = 1;
      reports_present.STEER_MOTOR_3 = 1;
      reports_present.REAR_PASS_DOOR = 1;
      reports_present.TURN = 1;
      reports_present.TURN_AUX = 1;
      reports_present.WIPER = 1;
      reports_present.WIPER_AUX = 1;

      reports_present.VEHICLE_SPEED = 1;
      reports_present.VIN = 1;

      commands_present.ACCEL = 1;
      commands_present.BRAKE = 1;
      commands_present.SHIFT = 1;
      commands_present.STEER = 1;
      commands_present.TURN = 1;
      commands_present.REAR_PASS_DOOR = 1;
      commands_present.WIPER = 1;
      break;

    case VehicleType::JUPITER_SPIRIT:
      reports_present.GLOBAL = 1;
      reports_present.ACCEL = 1;
      reports_present.BRAKE = 1;
      reports_present.SHIFT = 1;
      reports_present.STEER = 1;
      reports_present.ACCEL_AUX = 1;
      reports_present.BRAKE_AUX = 1;
      reports_present.SHIFT_AUX = 1;
      reports_present.STEER_AUX = 1;
      reports_present.HEADLIGHTS = 1;
      reports_present.HEADLIGHTS_AUX = 1;
      reports_present.HORN = 1;
      reports_present.PARKING_BRAKE = 1;
      reports_present.REAR_PASS_DOOR = 1;
      reports_present.TURN = 1;
      reports_present.TURN_AUX = 1;
      reports_present.DATE_TIME = 1;
      reports_present.LAT_LON_HEADING = 1;
      reports_present.WHEEL_SPEED = 1;
      reports_present.YAW_RATE = 1;

      reports_present.VEHICLE_SPEED = 1;
      reports_present.VIN = 1;

      commands_present.ACCEL = 1;
      commands_present.BRAKE = 1;
      commands_present.SHIFT = 1;
      commands_present.STEER = 1;
      commands_present.HEADLIGHTS = 1;
      commands_present.HORN = 1;
      commands_present.TURN = 1;
      commands_present.REAR_PASS_DOOR = 1;
      break;

    case VehicleType::LEXUS_RX_450H:
      reports_present.GLOBAL = 1;
      reports_present.ACCEL = 1;
      reports_present.BRAKE = 1;
      reports_present.SHIFT = 1;
      reports_present.STEER = 1;
      reports_present.ACCEL_AUX = 1;
      reports_present.BRAKE_AUX = 1;
      reports_present.SHIFT_AUX = 1;
      reports_present.STEER_AUX = 1;
      reports_present.HEADLIGHTS = 1;
      reports_present.HEADLIGHTS_AUX = 1;
      reports_present.HORN = 1;
      reports_present.PARKING_BRAKE = 1;
      reports_present.REAR_PASS_DOOR = 1;
      reports_present.TURN = 1;
      reports_present.TURN_AUX = 1;
      reports_present.DATE_TIME = 1;
      reports_present.LAT_LON_HEADING = 1;
      reports_present.WHEEL_SPEED = 1;
      reports_present.YAW_RATE = 1;
      reports_present.VEHICLE_SPEED = 1;

      commands_present.ACCEL = 1;
      commands_present.BRAKE = 1;
      commands_present.SHIFT = 1;
      commands_present.STEER = 1;
      commands_present.HEADLIGHTS = 1;
      commands_present.HORN = 1;
      commands_present.TURN = 1;
      commands_present.REAR_PASS_DOOR = 1;
      break;

    case VehicleType::POLARIS_GEM:
      reports_present.GLOBAL = 1;
      reports_present.ACCEL = 1;
      reports_present.BRAKE = 1;
      reports_present.SHIFT = 1;
      reports_present.STEER = 1;
      reports_present.ACCEL_AUX = 1;
      reports_present.BRAKE_AUX = 1;
      reports_present.SHIFT_AUX = 1;
      reports_present.STEER_AUX = 1;
      reports_present.BRAKE_MOTOR_1 = 1;
      reports_present.BRAKE_MOTOR_2 = 1;
      reports_present.BRAKE_MOTOR_3 = 1;
      reports_present.STEER_MOTOR_1 = 1;
      reports_present.STEER_MOTOR_2 = 1;
      reports_present.STEER_MOTOR_3 = 1;
      reports_present.REAR_PASS_DOOR = 1;
      reports_present.TURN = 1;
      reports_present.TURN_AUX = 1;
      reports_present.VEHICLE_SPEED = 1;
      reports_present.VIN = 1;

      commands_present.ACCEL = 1;
      commands_present.BRAKE = 1;
      commands_present.SHIFT = 1;
      commands_present.STEER = 1;
      commands_present.TURN = 1;
      commands_present.REAR_PASS_DOOR = 1;
      break;

    case VehicleType::POLARIS_RANGER:
      reports_present.GLOBAL = 1;
      reports_present.ESTOP = 1;
      reports_present.WATCHDOG = 1;
      reports_present.ACCEL = 1;
      reports_present.BRAKE = 1;
      reports_present.SHIFT = 1;
      reports_present.STEER = 1;
      reports_present.ACCEL_AUX = 1;
      reports_present.BRAKE_AUX = 1;
      reports_present.SHIFT_AUX = 1;
      reports_present.COMP_0 = 1;
      reports_present.COMP_1 = 1;
      reports_present.COMP_2 = 1;
      reports_present.SOFTW_0 = 1;
      reports_present.SOFTW_1 = 1;
      reports_present.SOFTW_2 = 1;
      reports_present.BRAKE_MOTOR_1 = 1;
      reports_present.BRAKE_MOTOR_2 = 1;
      reports_present.BRAKE_MOTOR_3 = 1;
      reports_present.STEER_MOTOR_1 = 1;
      reports_present.STEER_MOTOR_2 = 1;
      reports_present.STEER_MOTOR_3 = 1;
      reports_present.VEHICLE_SPEED = 1;

      commands_present.GLOBAL = 1;
      commands_present.USER_NOTIFICATION = 1;
      commands_present.ACCEL = 1;
      commands_present.BRAKE = 1;
      commands_present.SHIFT = 1;
      commands_present.STEER = 1;
      break;

    case VehicleType::HEXAGON_TRACTOR:
      reports_present.GLOBAL_2 = 1;
      reports_present.ESTOP = 1;
      reports_present.WATCHDOG = 1;
      reports_present.ACCEL = 1;
      reports_present.BRAKE = 1;
      reports_present.SHIFT = 1;
      reports_present.STEER = 1;
      reports_present.ACCEL_AUX = 1;
      reports_present.BRAKE_AUX = 1;
      reports_present.COMP_0 = 1;
      reports_present.COMP_1 = 1;
      reports_present.COMP_2 = 1;
      reports_present.COMP_3 = 1;
      reports_present.HAZARDS = 1;
      reports_present.HEADLIGHTS = 1;
      reports_present.HORN = 1;
      reports_present.PARKING_BRAKE = 1;
      reports_present.SPRAY = 1;
      reports_present.TURN = 1;
      reports_present.WIPER = 1;
      reports_present.ENGINE = 1;
      reports_present.VEHICLE_SPEED = 1;
      reports_present.VIN = 1;
      
      commands_present.GLOBAL = 1;
      commands_present.ACCEL = 1;
      commands_present.BRAKE = 1;
      commands_present.SHIFT = 1;
      commands_present.STEER = 1;
      commands_present.HAZARDS = 1;
      commands_present.HEADLIGHTS = 1;
      commands_present.HORN = 1;
      commands_present.PARKING_BRAKE = 1;
      commands_present.SPRAY = 1;
      commands_present.TURN = 1;
      commands_present.WIPER = 1;
      break;

    case VehicleType::FORD_RANGER:
      reports_present.GLOBAL_2 = 1;
      reports_present.ESTOP = 1;
      reports_present.WATCHDOG = 1;
      reports_present.ACCEL = 1;
      reports_present.BRAKE = 1;
      reports_present.SHIFT = 1;
      reports_present.STEER = 1;
      reports_present.ACCEL_AUX = 1;
      reports_present.BRAKE_AUX = 1;
      reports_present.SHIFT_AUX = 1;
      reports_present.STEER_AUX = 1;
      reports_present.COMP_0 = 1;
      reports_present.COMP_1 = 1;
      reports_present.COMP_2 = 1;
      reports_present.COMP_3 = 1;
      reports_present.SOFTW_0 = 1;
      reports_present.SOFTW_1 = 1;
      reports_present.SOFTW_2 = 1;
      reports_present.SOFTW_3 = 1;
      reports_present.BRAKE_MOTOR_1 = 1;
      reports_present.BRAKE_MOTOR_2 = 1;
      reports_present.BRAKE_MOTOR_3 = 1;
      reports_present.STEER_MOTOR_1 = 1;
      reports_present.STEER_MOTOR_2 = 1;
      reports_present.STEER_MOTOR_3 = 1;
      reports_present.CABIN_CLIMATE = 1;
      reports_present.HAZARDS = 1;
      reports_present.HEADLIGHTS = 1;
      reports_present.HEADLIGHTS_AUX = 1;
      reports_present.HORN = 1;
      reports_present.PARKING_BRAKE = 1;
      reports_present.PARKING_BRAKE_AUX = 1;
      reports_present.TURN = 1;
      reports_present.TURN_AUX = 1;
      reports_present.WIPER = 1;
      reports_present.WIPER_AUX = 1;

      reports_present.DOOR = 1;
      reports_present.ENGINE = 1;
      reports_present.INTERIOR_LIGHTS = 1;
      reports_present.OCCUPANCY = 1;
      reports_present.TIRE_PRESSURE = 1;
      reports_present.WHEEL_SPEED = 1;
      reports_present.VEHICLE_SPEED = 1;

      commands_present.GLOBAL = 1;
      commands_present.ACCEL = 1;
      commands_present.BRAKE = 1;
      commands_present.SHIFT = 1;
      commands_present.STEER = 1;
      commands_present.CABIN_CLIMATE = 1;
      commands_present.HAZARDS = 1;
      commands_present.HEADLIGHTS = 1;
      commands_present.HORN = 1;
      commands_present.PARKING_BRAKE = 1;
      commands_present.TURN = 1;
      commands_present.WIPER = 1;
      break;

    case VehicleType::VEHICLE_FTT:
      reports_present.GLOBAL_2 = 1;
      reports_present.ESTOP = 1;
      reports_present.WATCHDOG_2 = 1;
      reports_present.ACCEL = 1;
      reports_present.BRAKE = 1;
      reports_present.SHIFT = 1;
      reports_present.STEER = 1;
      reports_present.ACCEL_AUX = 1;
      reports_present.BRAKE_AUX = 1;
      reports_present.SHIFT_AUX = 1;
      reports_present.STEER_AUX = 1;
      reports_present.COMP_0 = 1;
      reports_present.COMP_1 = 1;
      reports_present.COMP_2 = 1;
      reports_present.COMP_3 = 1;
      reports_present.SOFTW_0 = 1;
      reports_present.SOFTW_1 = 1;
      reports_present.SOFTW_2 = 1;
      reports_present.SOFTW_3 = 1;
      reports_present.ACCEL_CMD_LIMIT = 1;
      reports_present.BRAKE_CMD_LIMIT = 1;
      reports_present.STEER_CMD_LIMIT = 1;
      reports_present.CRUISE_CONTROL = 1;
      reports_present.DASH_RIGHT = 1;
      reports_present.HAZARDS = 1;
      reports_present.HEADLIGHTS = 1;
      reports_present.HEADLIGHTS_AUX = 1;
      reports_present.HORN = 1;
      reports_present.MEDIA_CONTROLS = 1;
      reports_present.TURN = 1;
      reports_present.TURN_AUX = 1;

      reports_present.ANG_VEL = 1;
      reports_present.DOOR = 1;
      reports_present.ENGINE = 1;
      reports_present.INTERIOR_LIGHTS = 1;
      reports_present.OCCUPANCY = 1;
      reports_present.WHEEL_SPEED = 1;

      reports_present.VEHICLE_SPEED = 1;

      commands_present.GLOBAL = 1;
      commands_present.USER_NOTIFICATION = 1;
      commands_present.ACCEL = 1;
      commands_present.BRAKE = 1;
      commands_present.SHIFT = 1;
      commands_present.STEER = 1;
      commands_present.HAZARDS = 1;
      commands_present.HEADLIGHTS = 1;
      commands_present.HORN = 1;
      commands_present.MEDIA_CONTROLS = 1;
      commands_present.TURN = 1;
      break;

    case VehicleType::VEHICLE_HCV:
      reports_present.GLOBAL_2 = 1;
      reports_present.ESTOP = 1;
      reports_present.WATCHDOG_2 = 1;
      reports_present.ACCEL = 1;
      reports_present.BRAKE = 1;
      reports_present.SHIFT = 1;
      reports_present.STEER = 1;
      reports_present.ACCEL_AUX = 1;
      reports_present.BRAKE_AUX = 1;
      reports_present.SHIFT_AUX = 1;
      reports_present.STEER_AUX = 1;
      reports_present.COMP_0 = 1;
      reports_present.COMP_1 = 1;
      reports_present.COMP_2 = 1;
      reports_present.SOFTW_0 = 1;
      reports_present.SOFTW_1 = 1;
      reports_present.SOFTW_2 = 1;
      reports_present.STEER_CMD_LIMIT = 1;
      reports_present.HAZARDS = 1;
      reports_present.HEADLIGHTS = 1;
      reports_present.HEADLIGHTS_AUX = 1;
      reports_present.HORN = 1;
      reports_present.TURN = 1;
      reports_present.WHEEL_SPEED = 1;
      reports_present.VEHICLE_SPEED = 1;

      commands_present.GLOBAL = 1;
      commands_present.USER_NOTIFICATION = 1;
      commands_present.ACCEL = 1;
      commands_present.BRAKE = 1;
      commands_present.SHIFT = 1;
      commands_present.STEER = 1;
      commands_present.HAZARDS = 1;
      commands_present.HEADLIGHTS = 1;
      commands_present.HORN = 1;
      commands_present.TURN = 1;
      break;

    case VehicleType::VEHICLE_4:
      reports_present.GLOBAL = 1;
      reports_present.ACCEL = 1;
      reports_present.BRAKE = 1;
      reports_present.SHIFT = 1;
      reports_present.STEER = 1;
      reports_present.ACCEL_AUX = 1;
      reports_present.BRAKE_AUX = 1;
      reports_present.SHIFT_AUX = 1;
      reports_present.STEER_AUX = 1;
      reports_present.REAR_PASS_DOOR = 1;
      reports_present.TURN = 1;
      reports_present.TURN_AUX = 1;
      reports_present.VEHICLE_SPEED = 1;
      reports_present.DETECTED_OBJECT = 1;
      reports_present.VIN = 1;

      commands_present.ACCEL = 1;
      commands_present.BRAKE = 1;
      commands_present.SHIFT = 1;
      commands_present.STEER = 1;
      commands_present.TURN = 1;
      commands_present.REAR_PASS_DOOR = 1;
      break;

    case VehicleType::VEHICLE_5:
      reports_present.GLOBAL = 1;
      reports_present.ACCEL = 1;
      reports_present.BRAKE = 1;
      reports_present.SHIFT = 1;
      reports_present.STEER = 1;
      reports_present.ACCEL_AUX = 1;
      reports_present.BRAKE_AUX = 1;
      reports_present.SHIFT_AUX = 1;
      reports_present.STEER_AUX = 1;
      reports_present.HEADLIGHTS = 1;
      reports_present.HEADLIGHTS_AUX = 1;
      reports_present.HORN = 1;
      reports_present.PARKING_BRAKE = 1;
      reports_present.REAR_PASS_DOOR = 1;
      reports_present.TURN = 1;
      reports_present.TURN_AUX = 1;
      reports_present.DOOR = 1;
      reports_present.INTERIOR_LIGHTS = 1;
      reports_present.OCCUPANCY = 1;
      reports_present.REAR_LIGHTS = 1;
      reports_present.DATE_TIME = 1;
      reports_present.LAT_LON_HEADING = 1;
      reports_present.WHEEL_SPEED = 1;
      reports_present.YAW_RATE = 1;
      reports_present.VEHICLE_SPEED = 1;
      reports_present.VIN = 1;

      commands_present.ACCEL = 1;
      commands_present.BRAKE = 1;
      commands_present.SHIFT = 1;
      commands_present.STEER = 1;
      commands_present.TURN = 1;
      commands_present.REAR_PASS_DOOR = 1;
      break;

    case VehicleType::VEHICLE_6:
      reports_present.GLOBAL = 1;
      reports_present.ACCEL = 1;
      reports_present.BRAKE = 1;
      reports_present.SHIFT = 1;
      reports_present.STEER = 1;
      reports_present.ACCEL_AUX = 1;
      reports_present.BRAKE_AUX = 1;
      reports_present.SHIFT_AUX = 1;
      reports_present.STEER_AUX = 1;
      reports_present.HEADLIGHTS = 1;
      reports_present.HEADLIGHTS_AUX = 1;
      reports_present.HORN = 1;
      reports_present.PARKING_BRAKE = 1;
      reports_present.REAR_PASS_DOOR = 1;
      reports_present.TURN = 1;
      reports_present.TURN_AUX = 1;
      reports_present.DATE_TIME = 1;
      reports_present.LAT_LON_HEADING = 1;
      reports_present.WHEEL_SPEED = 1;
      reports_present.YAW_RATE = 1;
      reports_present.VEHICLE_SPEED = 1;
      reports_present.VIN = 1;

      commands_present.ACCEL = 1;
      commands_present.BRAKE = 1;
      commands_present.SHIFT = 1;
      commands_present.STEER = 1;
      commands_present.HEADLIGHTS = 1;
      commands_present.HORN = 1;
      commands_present.TURN = 1;
      commands_present.REAR_PASS_DOOR = 1;
      break;

    default:
      break;
  }

  // Advertise published messages
  can_rx_pub = n.advertise<can_msgs::Frame>("can_rx", 20);

  enabled_pub = n.advertise<std_msgs::Bool>("as_tx/enabled", 20, true);
  all_system_statuses_pub = n.advertise<pacmod_msgs::AllSystemStatuses>("as_tx/all_system_statuses", 20);
  std::string frame_id = "pacmod";

  ros::Subscriber can_tx_sub = n.subscribe("can_tx", 20, can_read);
  
  // For platforms using Safety ECU System  
  if (safety_ecu_flag)
  {
    ros::Publisher safety_func_rpt_pub = n.advertise<pacmod_msgs::SafetyFuncRpt>("parsed_tx/safety_func_rpt", 20);
    ros::Publisher safety_brake_rpt_pub = n.advertise<pacmod_msgs::SafetyBrakeRpt>("parsed_tx/safety_brake_rpt", 20);

    pub_tx_list.emplace(SafetyFuncRptMsg::CAN_ID, std::move(safety_func_rpt_pub));
    pub_tx_list.emplace(SafetyBrakeRptMsg::CAN_ID, std::move(safety_brake_rpt_pub));

    safety_func_cmd_sub =
      std::make_shared<ros::Subscriber>(n.subscribe("as_rx/safety_func_cmd", 20, callback_safety_func_set_cmd));
    supervisory_ctrl_cmd_sub =
      std::make_shared<ros::Subscriber>(n.subscribe("as_rx/supervisory_ctrl_cmd", 20, callback_supervisory_ctrl_cmd_sub));
    safety_brake_cmd_sub =
      std::make_shared<ros::Subscriber>(n.subscribe("as_rx/safety_brake_cmd", 20, callback_safety_brake_set_cmd));

    rx_list.emplace(
      SafetyFuncCmdMsg::CAN_ID,
      std::shared_ptr<LockedData>(new LockedData(SafetyFuncCmdMsg::DATA_LENGTH)));
    rx_list.emplace(
      SupervisoryCtrlMsg::CAN_ID,
      std::shared_ptr<LockedData>(new LockedData(SupervisoryCtrlMsg::DATA_LENGTH)));
    rx_list.emplace(
      SafetyBrakeCmdMsg::CAN_ID,
      std::shared_ptr<LockedData>(new LockedData(SafetyBrakeCmdMsg::DATA_LENGTH)));
  }

  // Publish messages
  if (reports_present.GLOBAL)
  {
    ros::Publisher global_rpt_pub = n.advertise<pacmod_msgs::GlobalRpt>("parsed_tx/global_rpt", 20);
    pub_tx_list.emplace(GlobalRptMsg::CAN_ID, std::move(global_rpt_pub));
  }

  if (reports_present.GLOBAL_2)
  {
    ros::Publisher global_rpt2_pub = n.advertise<pacmod_msgs::GlobalRpt2>("parsed_tx/global_rpt2", 20);
    pub_tx_list.emplace(GlobalRpt2Msg::CAN_ID, std::move(global_rpt2_pub));
  }

  if (reports_present.ESTOP)
  {
    ros::Publisher estop_rpt_pub = n.advertise<pacmod_msgs::EStopRpt>("parsed_tx/estop_rpt", 20);
    pub_tx_list.emplace(EStopRptMsg::CAN_ID, std::move(estop_rpt_pub));
  }

  if (reports_present.WATCHDOG)
  {
    ros::Publisher watchdog_rpt_pub = 
      n.advertise<pacmod_msgs::WatchdogRpt>("parsed_tx/watchdog_rpt", 20);
    pub_tx_list.emplace(WatchdogRptMsg::CAN_ID, std::move(watchdog_rpt_pub));
  }

  if (reports_present.WATCHDOG_2)
  {
    ros::Publisher watchdog_rpt2_pub = 
      n.advertise<pacmod_msgs::WatchdogRpt2>("parsed_tx/watchdog_rpt2", 20);
    pub_tx_list.emplace(WatchdogRpt2Msg::CAN_ID, std::move(watchdog_rpt2_pub));
  }

  if (reports_present.ACCEL)
  {
    ros::Publisher accel_rpt_pub = n.advertise<pacmod_msgs::SystemRptFloat>("parsed_tx/accel_rpt", 20);
    pub_tx_list.emplace(AccelRptMsg::CAN_ID, std::move(accel_rpt_pub));
  }

  if (reports_present.BRAKE)
  {
    ros::Publisher brake_rpt_pub = n.advertise<pacmod_msgs::SystemRptFloat>("parsed_tx/brake_rpt", 20);
    pub_tx_list.emplace(BrakeRptMsg::CAN_ID, std::move(brake_rpt_pub));
  }

  if (reports_present.SHIFT)
  {
    ros::Publisher shift_rpt_pub = n.advertise<pacmod_msgs::SystemRptInt>("parsed_tx/shift_rpt", 20);
    pub_tx_list.emplace(ShiftRptMsg::CAN_ID, std::move(shift_rpt_pub));
  }

  if (reports_present.STEER)
  {
    ros::Publisher steer_rpt_pub = n.advertise<pacmod_msgs::SystemRptFloat>("parsed_tx/steer_rpt", 20);
    pub_tx_list.emplace(SteerRptMsg::CAN_ID, std::move(steer_rpt_pub));
  }

  if (reports_present.ACCEL_AUX)
  {
    ros::Publisher accel_aux_rpt_pub = n.advertise<pacmod_msgs::AccelAuxRpt>("parsed_tx/accel_aux_rpt", 20);
    pub_tx_list.emplace(AccelAuxRptMsg::CAN_ID, std::move(accel_aux_rpt_pub));
  }

  if (reports_present.BRAKE_AUX)
  {
    ros::Publisher brake_aux_rpt_pub = n.advertise<pacmod_msgs::BrakeAuxRpt>("parsed_tx/brake_aux_rpt", 20);
    pub_tx_list.emplace(BrakeAuxRptMsg::CAN_ID, std::move(brake_aux_rpt_pub));
  }

  if (reports_present.SHIFT_AUX)
  {
    ros::Publisher shift_aux_rpt_pub =
      n.advertise<pacmod_msgs::ShiftAuxRpt>("parsed_tx/shift_aux_rpt", 20);
    pub_tx_list.emplace(ShiftAuxRptMsg::CAN_ID, std::move(shift_aux_rpt_pub));
  }

  if (reports_present.STEER_AUX)
  {
    ros::Publisher steer_aux_rpt_pub =
      n.advertise<pacmod_msgs::SteerAuxRpt>("parsed_tx/steer_aux_rpt", 20);
    pub_tx_list.emplace(SteerAuxRptMsg::CAN_ID, std::move(steer_aux_rpt_pub));  
  }

  if (reports_present.COMP_0)
  {
    ros::Publisher component_rpt0_pub = 
      n.advertise<pacmod_msgs::ComponentRpt>("parsed_tx/component_rpt0", 20);
    pub_tx_list.emplace(ComponentRptMsg00::CAN_ID, std::move(component_rpt0_pub));
  }

  if (reports_present.COMP_1)
  {
    ros::Publisher component_rpt1_pub = 
      n.advertise<pacmod_msgs::ComponentRpt>("parsed_tx/component_rpt1", 20);
    pub_tx_list.emplace(ComponentRptMsg01::CAN_ID, std::move(component_rpt1_pub));
  }

  if (reports_present.COMP_2)
  {
    ros::Publisher component_rpt2_pub = 
      n.advertise<pacmod_msgs::ComponentRpt>("parsed_tx/component_rpt2", 20);
    pub_tx_list.emplace(ComponentRptMsg02::CAN_ID, std::move(component_rpt2_pub));
  }

  if (reports_present.COMP_3)
  {
    ros::Publisher component_rpt3_pub = 
      n.advertise<pacmod_msgs::ComponentRpt>("parsed_tx/component_rpt3", 20);
    pub_tx_list.emplace(ComponentRptMsg03::CAN_ID, std::move(component_rpt3_pub));
  }

  if (reports_present.COMP_4)
  {
    ros::Publisher component_rpt4_pub = 
      n.advertise<pacmod_msgs::ComponentRpt>("parsed_tx/component_rpt4", 20);
    pub_tx_list.emplace(ComponentRptMsg04::CAN_ID, std::move(component_rpt4_pub));
  }

  if (reports_present.SOFTW_0)
  {
    ros::Publisher software_ver_rpt0_pub = 
      n.advertise<pacmod_msgs::SoftwareVersionRpt>("parsed_tx/software_ver_rpt0", 20);
    pub_tx_list.emplace(SoftwareVerRptMsg00::CAN_ID, std::move(software_ver_rpt0_pub));
  }

  if (reports_present.SOFTW_1)
  {
    ros::Publisher software_ver_rpt1_pub = 
      n.advertise<pacmod_msgs::SoftwareVersionRpt>("parsed_tx/software_ver_rpt1", 20);
    pub_tx_list.emplace(SoftwareVerRptMsg01::CAN_ID, std::move(software_ver_rpt1_pub));
  }

  if (reports_present.SOFTW_2)
  {
    ros::Publisher software_ver_rpt2_pub = 
      n.advertise<pacmod_msgs::SoftwareVersionRpt>("parsed_tx/software_ver_rpt2", 20);
    pub_tx_list.emplace(SoftwareVerRptMsg02::CAN_ID, std::move(software_ver_rpt2_pub));
  }

  if (reports_present.SOFTW_3)
  {
    ros::Publisher software_ver_rpt3_pub = 
      n.advertise<pacmod_msgs::SoftwareVersionRpt>("parsed_tx/software_ver_rpt3", 20);
    pub_tx_list.emplace(SoftwareVerRptMsg03::CAN_ID, std::move(software_ver_rpt3_pub));
  }

  if (reports_present.SOFTW_4)
  {
    ros::Publisher software_ver_rpt4_pub = 
      n.advertise<pacmod_msgs::SoftwareVersionRpt>("parsed_tx/software_ver_rpt4", 20);
    pub_tx_list.emplace(SoftwareVerRptMsg04::CAN_ID, std::move(software_ver_rpt4_pub));
  }

  if (reports_present.ACCEL_CMD_LIMIT)
  {
    ros::Publisher accel_cmd_limit_rpt_pub = 
      n.advertise<pacmod_msgs::SystemCmdLimitRpt>("parsed_tx/accel_cmd_limit_rpt", 20);
    pub_tx_list.emplace(AccelCmdLimitRptMsg::CAN_ID, std::move(accel_cmd_limit_rpt_pub));
  }

  if (reports_present.BRAKE_CMD_LIMIT)
  {
    ros::Publisher brake_cmd_limit_rpt_pub = 
      n.advertise<pacmod_msgs::SystemCmdLimitRpt>("parsed_tx/brake_cmd_limit_rpt", 20);
    pub_tx_list.emplace(BrakeCmdLimitRptMsg::CAN_ID, std::move(brake_cmd_limit_rpt_pub));
  }

  if (reports_present.STEER_CMD_LIMIT)
  {
    ros::Publisher steer_cmd_limit_rpt_pub = 
      n.advertise<pacmod_msgs::SteerCmdLimitRpt>("parsed_tx/steer_cmd_limit_rpt", 20);
    pub_tx_list.emplace(SteerCmdLimitRptMsg::CAN_ID, std::move(steer_cmd_limit_rpt_pub));
  }

  if (reports_present.BRAKE_MOTOR_1)
  {
    ros::Publisher brake_rpt_detail_1_pub =
      n.advertise<pacmod_msgs::MotorRpt1>("parsed_tx/brake_rpt_detail_1", 20);
    pub_tx_list.emplace(BrakeMotorRpt1Msg::CAN_ID, std::move(brake_rpt_detail_1_pub));
  }

  if (reports_present.BRAKE_MOTOR_2)
  {
    ros::Publisher brake_rpt_detail_2_pub =
      n.advertise<pacmod_msgs::MotorRpt2>("parsed_tx/brake_rpt_detail_2", 20);
    pub_tx_list.emplace(BrakeMotorRpt2Msg::CAN_ID, std::move(brake_rpt_detail_2_pub));
  }

  if (reports_present.BRAKE_MOTOR_3)
  {
    ros::Publisher brake_rpt_detail_3_pub =
      n.advertise<pacmod_msgs::MotorRpt3>("parsed_tx/brake_rpt_detail_3", 20);
    pub_tx_list.emplace(BrakeMotorRpt3Msg::CAN_ID, std::move(brake_rpt_detail_3_pub));
  }

  if (reports_present.STEER_MOTOR_1)
  {
    ros::Publisher steering_rpt_detail_1_pub =
      n.advertise<pacmod_msgs::MotorRpt1>("parsed_tx/steer_rpt_detail_1", 20);
    pub_tx_list.emplace(SteerMotorRpt1Msg::CAN_ID, std::move(steering_rpt_detail_1_pub));
  }

  if (reports_present.STEER_MOTOR_2)
  {
    ros::Publisher steering_rpt_detail_2_pub =
      n.advertise<pacmod_msgs::MotorRpt2>("parsed_tx/steer_rpt_detail_2", 20);
    pub_tx_list.emplace(SteerMotorRpt2Msg::CAN_ID, std::move(steering_rpt_detail_2_pub));
  }

  if (reports_present.STEER_MOTOR_3)
  {
    ros::Publisher steering_rpt_detail_3_pub = 
      n.advertise<pacmod_msgs::MotorRpt3>("parsed_tx/steer_rpt_detail_3", 20);
    pub_tx_list.emplace(SteerMotorRpt3Msg::CAN_ID, std::move(steering_rpt_detail_3_pub));
  }

  if (reports_present.CABIN_CLIMATE)
  {
    ros::Publisher cabin_climate_rpt_pub =
      n.advertise<pacmod_msgs::CabinClimateRpt>("parsed_tx/cabin_climate_rpt", 20);
    pub_tx_list.emplace(CabinClimateRptMsg::CAN_ID, std::move(cabin_climate_rpt_pub));
  }

  if (reports_present.CRUISE_CONTROL)
  {
    ros::Publisher cruise_control_buttons_rpt_pub =
      n.advertise<pacmod_msgs::SystemRptInt>("parsed_tx/cruise_control_buttons_rpt", 20);
    pub_tx_list.emplace(CruiseControlButtonsRptMsg::CAN_ID, std::move(cruise_control_buttons_rpt_pub));
  }

  if (reports_present.DASH_RIGHT)
  {
    ros::Publisher dash_control_right_rpt_pub =
      n.advertise<pacmod_msgs::SystemRptInt>("parsed_tx/dash_control_right_rpt", 20);
    pub_tx_list.emplace(DashControlsRightRptMsg::CAN_ID, std::move(dash_control_right_rpt_pub));
  }

  if (reports_present.ENGINE_BRAKE)
  {
    ros::Publisher engine_brake_rpt_pub =
      n.advertise<pacmod_msgs::SystemRptInt>("parsed_tx/engine_brake_rpt", 20);
    pub_tx_list.emplace(EngineBrakeRptMsg::CAN_ID, std::move(engine_brake_rpt_pub));
  }

  if (reports_present.HAZARDS)
  {
    ros::Publisher hazard_lights_rpt_pub =
      n.advertise<pacmod_msgs::SystemRptBool>("parsed_tx/hazard_lights_rpt", 20);
    pub_tx_list.emplace(HazardLightRptMsg::CAN_ID, std::move(hazard_lights_rpt_pub));
  }

  if (reports_present.HEADLIGHTS)
  {
    ros::Publisher headlight_rpt_pub =
      n.advertise<pacmod_msgs::SystemRptInt>("parsed_tx/headlight_rpt", 20);
    pub_tx_list.emplace(HeadlightRptMsg::CAN_ID, std::move(headlight_rpt_pub));
  }

  if (reports_present.HEADLIGHTS_AUX)
  {
    ros::Publisher headlight_aux_rpt_pub =
      n.advertise<pacmod_msgs::HeadlightAuxRpt>("parsed_tx/headlight_aux_rpt", 20);
    pub_tx_list.emplace(HeadlightAuxRptMsg::CAN_ID, std::move(headlight_aux_rpt_pub));
  }

  if (reports_present.HORN)
  {
    ros::Publisher horn_rpt_pub =
      n.advertise<pacmod_msgs::SystemRptBool>("parsed_tx/horn_rpt", 20);
    pub_tx_list.emplace(HornRptMsg::CAN_ID, std::move(horn_rpt_pub));
  }

  if (reports_present.MARKER_LAMP)
  {
    ros::Publisher marker_lamp_rpt_pub =
      n.advertise<pacmod_msgs::SystemRptBool>("parsed_tx/marker_lamp_rpt", 20);
    pub_tx_list.emplace(MarkerLampRptMsg::CAN_ID, std::move(marker_lamp_rpt_pub));
  }

  if (reports_present.MEDIA_CONTROLS)
  {
    ros::Publisher media_control_rpt_pub =
      n.advertise<pacmod_msgs::SystemRptInt>("parsed_tx/media_control_rpt", 20);
    pub_tx_list.emplace(MediaControlsRptMsg::CAN_ID, std::move(media_control_rpt_pub));
  }

  if (reports_present.PARKING_BRAKE)
  {
    ros::Publisher parking_brake_rpt_pub =
      n.advertise<pacmod_msgs::SystemRptBool>("parsed_tx/parking_brake_rpt", 20);
    pub_tx_list.emplace(ParkingBrakeRptMsg::CAN_ID, std::move(parking_brake_rpt_pub));
  }

  if (reports_present.PARKING_BRAKE_AUX)
  {
    ros::Publisher parking_brake_aux_rpt_pub =
      n.advertise<pacmod_msgs::ParkingBrakeAuxRpt>("parsed_tx/parkin_brake_aux_rpt", 20);
    pub_tx_list.emplace(ParkingBrakeAuxRptMsg::CAN_ID, std::move(parking_brake_aux_rpt_pub));
  }

  if (reports_present.REAR_PASS_DOOR)
  {
    ros::Publisher rear_pass_door_rpt_pub = 
      n.advertise<pacmod_msgs::SystemRptInt>("parsed_tx/rear_pass_door_rpt", 20);
    pub_tx_list.emplace(RearPassDoorRptMsg::CAN_ID, std::move(rear_pass_door_rpt_pub));
  }

  if (reports_present.SPRAY)
  {
    ros::Publisher sprayer_rpt_pub =
      n.advertise<pacmod_msgs::SystemRptBool>("parsed_tx/sprayer_rpt", 20);
    pub_tx_list.emplace(SprayerRptMsg::CAN_ID, std::move(sprayer_rpt_pub));
  }

  if (reports_present.TURN)
  {
    ros::Publisher turn_rpt_pub = n.advertise<pacmod_msgs::SystemRptInt>("parsed_tx/turn_rpt", 20);
    pub_tx_list.emplace(TurnSignalRptMsg::CAN_ID, std::move(turn_rpt_pub));
  }

  if (reports_present.TURN_AUX)
  {
    ros::Publisher turn_aux_rpt_pub =
      n.advertise<pacmod_msgs::TurnAuxRpt>("parsed_tx/turn_aux_rpt", 20);
    pub_tx_list.emplace(TurnAuxRptMsg::CAN_ID, std::move(turn_aux_rpt_pub));
  }

  if (reports_present.WIPER)
  {
    ros::Publisher wiper_rpt_pub =
      n.advertise<pacmod_msgs::SystemRptInt>("parsed_tx/wiper_rpt", 20);
    pub_tx_list.emplace(WiperRptMsg::CAN_ID, std::move(wiper_rpt_pub));
  }

  if (reports_present.WIPER_AUX)
  {
    ros::Publisher wiper_aux_rpt_pub =
      n.advertise<pacmod_msgs::WiperAuxRpt>("parsed_tx/wiper_aux_rpt", 20);
    pub_tx_list.emplace(WiperAuxRptMsg::CAN_ID, std::move(wiper_aux_rpt_pub));
  }

  if (reports_present.ANG_VEL)
  {
    ros::Publisher ang_vel_rpt_pub =
      n.advertise<pacmod_msgs::AngVelRpt>("parsed_tx/ang_vel_rpt", 20);
    pub_tx_list.emplace(AngVelRptMsg::CAN_ID, std::move(ang_vel_rpt_pub));
  }

  if (reports_present.DATE_TIME)
  {
    ros::Publisher date_time_rpt_pub =
      n.advertise<pacmod_msgs::DateTimeRpt>("parsed_tx/date_time_rpt", 20);
    pub_tx_list.emplace(DateTimeRptMsg::CAN_ID, std::move(date_time_rpt_pub));
  }

  if (reports_present.DETECTED_OBJECT)
  {
    ros::Publisher detected_object_rpt_pub =
      n.advertise<pacmod_msgs::DetectedObjectRpt>("parsed_tx/detected_object_rpt", 20);
    pub_tx_list.emplace(DetectedObjectRptMsg::CAN_ID, std::move(detected_object_rpt_pub));
  }

  if (reports_present.DOOR)
  {
    ros::Publisher door_rpt_pub =
      n.advertise<pacmod_msgs::DoorRpt>("parsed_tx/door_rpt", 20);
    pub_tx_list.emplace(DoorRptMsg::CAN_ID, std::move(door_rpt_pub));
  }

  if (reports_present.ENGINE)
  {
    ros::Publisher engine_rpt_pub =
      n.advertise<pacmod_msgs::EngineRpt>("parsed_tx/engine_rpt", 20);
    pub_tx_list.emplace(EngineRptMsg::CAN_ID, std::move(engine_rpt_pub));
  }
  
  if (reports_present.INTERIOR_LIGHTS)
  {
    ros::Publisher interior_lights_rpt_pub =
      n.advertise<pacmod_msgs::InteriorLightsRpt>("parsed_tx/interior_lights_rpt", 20);
    pub_tx_list.emplace(InteriorLightsRptMsg::CAN_ID, std::move(interior_lights_rpt_pub));
  }

  if (reports_present.LAT_LON_HEADING)
  {
    ros::Publisher lat_lon_heading_rpt_pub =
      n.advertise<pacmod_msgs::LatLonHeadingRpt>("parsed_tx/lat_lon_heading_rpt", 20);
    pub_tx_list.emplace(LatLonHeadingRptMsg::CAN_ID, std::move(lat_lon_heading_rpt_pub));
  }

  if (reports_present.OCCUPANCY)
  {
    ros::Publisher occupancy_rpt_pub =
      n.advertise<pacmod_msgs::OccupancyRpt>("parsed_tx/occupancy_rpt", 20);
    pub_tx_list.emplace(OccupancyRptMsg::CAN_ID, std::move(occupancy_rpt_pub));
  }
  
  if (reports_present.REAR_LIGHTS)
  {
    ros::Publisher rear_lights_rpt_pub =
      n.advertise<pacmod_msgs::RearLightsRpt>("parsed_tx/rear_lights_rpt", 20);
    pub_tx_list.emplace(RearLightsRptMsg::CAN_ID, std::move(rear_lights_rpt_pub));
  }
  
  if (reports_present.TIRE_PRESSURE)
  {
    ros::Publisher tire_pressure_rpt_pub = n.advertise<pacmod_msgs::TirePressureRpt>("parsed_tx/tire_pressure_rpt", 20);
    pub_tx_list.emplace(TirePressureRptMsg::CAN_ID, std::move(tire_pressure_rpt_pub));
  }
  
  if (reports_present.WHEEL_SPEED)
  {
    ros::Publisher wheel_speed_rpt_pub =
      n.advertise<pacmod_msgs::WheelSpeedRpt>("parsed_tx/wheel_speed_rpt", 20);
    pub_tx_list.emplace(WheelSpeedRptMsg::CAN_ID, std::move(wheel_speed_rpt_pub));
  }

  if (reports_present.YAW_RATE)
  {
    ros::Publisher yaw_rate_rpt_pub =
      n.advertise<pacmod_msgs::YawRateRpt>("parsed_tx/yaw_rate_rpt", 20);
    pub_tx_list.emplace(YawRateRptMsg::CAN_ID, std::move(yaw_rate_rpt_pub));
  }

  if (reports_present.VEHICLE_SPEED)
  {
    ros::Publisher vehicle_speed_pub = n.advertise<pacmod_msgs::VehicleSpeedRpt>("parsed_tx/vehicle_speed_rpt", 20);
    pub_tx_list.emplace(VehicleSpeedRptMsg::CAN_ID, std::move(vehicle_speed_pub));
  }
  
  if (reports_present.VIN)
  {
    ros::Publisher vin_rpt_pub = n.advertise<pacmod_msgs::VinRpt>("parsed_tx/vin_rpt", 5);
    pub_tx_list.emplace(VinRptMsg::CAN_ID, std::move(vin_rpt_pub));
  }

  // Subscribe to messages
  if (commands_present.GLOBAL)
  {
    global_cmd_sub = std::make_shared<ros::Subscriber>(
      n.subscribe("as_rx/global_cmd", 20, callback_global_cmd_sub));
    rx_list.emplace(
      GlobalCmdMsg::CAN_ID,
      std::shared_ptr<LockedData>(new LockedData(GlobalCmdMsg::DATA_LENGTH)));
  }

  if (commands_present.USER_NOTIFICATION)
  {
    user_notification_cmd_sub = std::make_shared<ros::Subscriber>(
      n.subscribe("as_rx/user_notification_cmd", 20, callback_user_notification_set_cmd));
    rx_list.emplace(
      UserNotificationCmdMsg::CAN_ID,
      std::shared_ptr<LockedData>(new LockedData(UserNotificationCmdMsg::DATA_LENGTH)));
  }

  if (commands_present.ACCEL)
  {
    accel_cmd_sub = std::make_shared<ros::Subscriber>(
      n.subscribe("as_rx/accel_cmd", 20, callback_accel_cmd_sub));
    rx_list.emplace(
      AccelCmdMsg::CAN_ID,
      std::shared_ptr<LockedData>(new LockedData(AccelCmdMsg::DATA_LENGTH)));
  }

  if (commands_present.BRAKE)
  {
    brake_cmd_sub = std::make_shared<ros::Subscriber>(
      n.subscribe("as_rx/brake_cmd", 20, callback_brake_cmd_sub));
    rx_list.emplace(
      BrakeCmdMsg::CAN_ID,
      std::shared_ptr<LockedData>(new LockedData(BrakeCmdMsg::DATA_LENGTH)));
  }

  if (commands_present.SHIFT)
  {
    shift_cmd_sub = std::make_shared<ros::Subscriber>(
      n.subscribe("as_rx/shift_cmd", 20, callback_shift_set_cmd));
    rx_list.emplace(
      ShiftCmdMsg::CAN_ID,
      std::shared_ptr<LockedData>(new LockedData(ShiftCmdMsg::DATA_LENGTH)));
  }

  if (commands_present.STEER)
  {
    steer_cmd_sub = std::make_shared<ros::Subscriber>(
      n.subscribe("as_rx/steer_cmd", 20, callback_steer_cmd_sub));
    rx_list.emplace(
      SteerCmdMsg::CAN_ID,
      std::shared_ptr<LockedData>(new LockedData(SteerCmdMsg::DATA_LENGTH)));
  }

  if (commands_present.CABIN_CLIMATE)
  {
    cabin_climate_set_cmd_sub = std::make_shared<ros::Subscriber>(
      n.subscribe("as_rx/cabin_climate_cmd", 20, callback_cabin_climate_set_cmd));
    rx_list.emplace(
      CabinClimateCmdMsg::CAN_ID,
      std::shared_ptr<LockedData>(new LockedData(CabinClimateCmdMsg::DATA_LENGTH)));
  }

  if (commands_present.CRUISE_CONTROL)
  {
    cruise_control_buttons_set_cmd_sub = std::make_shared<ros::Subscriber>(
      n.subscribe("as_rx/cruise_control_buttons_cmd", 20, callback_cruise_control_buttons_set_cmd));
    rx_list.emplace(
      CruiseControlButtonsCmdMsg::CAN_ID,
      std::shared_ptr<LockedData>(new LockedData(CruiseControlButtonsCmdMsg::DATA_LENGTH)));
  }

  if (commands_present.DASH_RIGHT)
  {
    dash_controls_right_set_cmd_sub = std::make_shared<ros::Subscriber>(
      n.subscribe("as_rx/dash_controls_right_cmd", 20, callback_dash_controls_right_set_cmd));
    rx_list.emplace(
      DashControlsRightCmdMsg::CAN_ID,
      std::shared_ptr<LockedData>(new LockedData(DashControlsRightCmdMsg::DATA_LENGTH)));
  }

  if (commands_present.ENGINE_BRAKE)
  {
    engine_brake_set_cmd_sub = std::make_shared<ros::Subscriber>(
      n.subscribe("as_rx/engine_brake_cmd", 20, callback_engine_brake_set_cmd));
    rx_list.emplace(
      EngineBrakeCmdMsg::CAN_ID,
      std::shared_ptr<LockedData>(new LockedData(EngineBrakeCmdMsg::DATA_LENGTH)));
  }

  if (commands_present.HAZARDS)
  {
    hazard_lights_set_cmd_sub = std::make_shared<ros::Subscriber>(
      n.subscribe("as_rx/hazard_lights_cmd", 20, callback_hazard_lights_set_cmd));
    rx_list.emplace(
      HazardLightCmdMsg::CAN_ID,
      std::shared_ptr<LockedData>(new LockedData(HazardLightCmdMsg::DATA_LENGTH)));
  }

  if (commands_present.HEADLIGHTS)
  {
    headlight_set_cmd_sub = std::make_shared<ros::Subscriber>(
      n.subscribe("as_rx/headlight_cmd", 20, callback_headlight_set_cmd));
    rx_list.emplace(
      HeadlightCmdMsg::CAN_ID,
      std::shared_ptr<LockedData>(new LockedData(HeadlightCmdMsg::DATA_LENGTH)));
  }

  if (commands_present.HORN)
  {
    horn_set_cmd_sub = std::make_shared<ros::Subscriber>(
      n.subscribe("as_rx/horn_cmd", 20, callback_horn_set_cmd));
    rx_list.emplace(
      HornCmdMsg::CAN_ID,
      std::shared_ptr<LockedData>(new LockedData(HornCmdMsg::DATA_LENGTH)));
  }

  if (commands_present.MARKER_LAMP)
  {
    marker_lamp_set_cmd_sub = std::make_shared<ros::Subscriber>(
      n.subscribe("as_rx/marker_lamp_cmd", 20, callback_marker_lamp_set_cmd));
    rx_list.emplace(
      MarkerLampCmdMsg::CAN_ID,
      std::shared_ptr<LockedData>(new LockedData(MarkerLampCmdMsg::DATA_LENGTH)));
  }

  if (commands_present.MEDIA_CONTROLS)
  {
    media_controls_set_cmd_sub =
      std::make_shared<ros::Subscriber>(n.subscribe("as_rx/media_controls_cmd", 20, callback_media_controls_set_cmd));
    rx_list.emplace(
      MediaControlsCmdMsg::CAN_ID,
      std::shared_ptr<LockedData>(new LockedData(MediaControlsCmdMsg::DATA_LENGTH)));
  }

  if (commands_present.PARKING_BRAKE)
  {
    parking_brake_set_cmd_sub = std::make_shared<ros::Subscriber>(
      n.subscribe("as_rx/parking_brake_cmd", 20, callback_parking_brake_cmd));
    rx_list.emplace(
      ParkingBrakeCmdMsg::CAN_ID,
      std::shared_ptr<LockedData>(new LockedData(ParkingBrakeCmdMsg::DATA_LENGTH)));
  }

  if (commands_present.REAR_PASS_DOOR)
  {
    rear_pass_door_cmd_sub = std::make_shared<ros::Subscriber>(
      n.subscribe("as_rx/rear_pass_door_cmd", 20, callback_rear_pass_door_set_cmd));
    rx_list.emplace(
      RearPassDoorCmdMsg::CAN_ID,
      std::shared_ptr<LockedData>(new LockedData(RearPassDoorCmdMsg::DATA_LENGTH)));
  }

  if (commands_present.SPRAY)
  {
    sprayer_set_cmd_sub = std::make_shared<ros::Subscriber>(
      n.subscribe("as_rx/sprayer_cmd", 20, callback_sprayer_set_cmd));
    rx_list.emplace(
      SprayerCmdMsg::CAN_ID,
      std::shared_ptr<LockedData>(new LockedData(SprayerCmdMsg::DATA_LENGTH)));
  }

  if (commands_present.TURN)
  {
    turn_cmd_sub = std::make_shared<ros::Subscriber>(
      n.subscribe("as_rx/turn_cmd", 20, callback_turn_signal_set_cmd));
    rx_list.emplace(
      TurnSignalCmdMsg::CAN_ID,
      std::shared_ptr<LockedData>(new LockedData(TurnSignalCmdMsg::DATA_LENGTH)));
    
    // Initialize Turn Signal with non-0 value
    TurnSignalCmdMsg turn_encoder;
    turn_encoder.encode(false, false, false, pacmod_msgs::SystemCmdInt::TURN_NONE);
    rx_list[TurnSignalCmdMsg::CAN_ID]->setData(std::move(turn_encoder.data));
  }

  if (commands_present.WIPER)
  {
    wiper_set_cmd_sub = std::make_shared<ros::Subscriber>(
      n.subscribe("as_rx/wiper_cmd", 20, callback_wiper_set_cmd));
    rx_list.emplace(
      WiperCmdMsg::CAN_ID,
      std::shared_ptr<LockedData>(new LockedData(WiperCmdMsg::DATA_LENGTH)));
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
      else if (system->first == BrakeDeccelRptMsg::CAN_ID)
        kvp.key = "Brake Deccel";
      else if (system->first == CabinClimateRptMsg::CAN_ID)
        kvp.key = "Cabin Climate";
      else if (system->first == CruiseControlButtonsRptMsg::CAN_ID)
        kvp.key = "Cruise Control Buttons";
      else if (system->first == DashControlsLeftRptMsg::CAN_ID)
        kvp.key = "Dash Controls Left";
      else if (system->first == DashControlsRightRptMsg::CAN_ID)
        kvp.key = "Dash Controls Right";
      else if (system->first == EngineBrakeRptMsg::CAN_ID)
        kvp.key = "Engine Brake";
      else if (system->first == HazardLightRptMsg::CAN_ID)
        kvp.key = "Hazard Lights";
      else if (system->first == HeadlightRptMsg::CAN_ID)
        kvp.key = "Headlights";
      else if (system->first == HornRptMsg::CAN_ID)
        kvp.key = "Horn";
      else if (system->first == MarkerLampRptMsg::CAN_ID)
        kvp.key = "Marker Lamp";
      else if (system->first == MediaControlsRptMsg::CAN_ID)
        kvp.key = "Media Controls";
      else if (system->first == ParkingBrakeRptMsg::CAN_ID)
        kvp.key = "Parking Brake";
      else if (system->first == RearPassDoorRptMsg::CAN_ID)
        kvp.key = "Rear Passenger Door";
      else if (system->first == SafetyBrakeRptMsg::CAN_ID)
        kvp.key = "Safety Brake";
      else if (system->first == SafetyFuncRptMsg::CAN_ID)
        kvp.key = "Safety Function";
      else if (system->first == ShiftRptMsg::CAN_ID)
        kvp.key = "Shifter";
      else if (system->first == SprayerRptMsg::CAN_ID)
        kvp.key = "Sprayer";
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

  can_write_thread.join();

  return 0;
}
