/*
* Unpublished Copyright (c) 2009-2017 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod v3 ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include <pacmod3_core.h>

using namespace AS::Drivers::PACMod3;

const int64_t AS::Drivers::PACMod3::TurnSignalCmdMsg::CAN_ID = 0x63;
const int64_t AS::Drivers::PACMod3::TurnSignalRptMsg::CAN_ID = 0x64;
const int64_t AS::Drivers::PACMod3::ShiftCmdMsg::CAN_ID = 0x65;
const int64_t AS::Drivers::PACMod3::ShiftRptMsg::CAN_ID = 0x66;
const int64_t AS::Drivers::PACMod3::AccelCmdMsg::CAN_ID = 0x67;
const int64_t AS::Drivers::PACMod3::AccelRptMsg::CAN_ID = 0x68;
const int64_t AS::Drivers::PACMod3::GlobalRptMsg::CAN_ID = 0x6A;
const int64_t AS::Drivers::PACMod3::BrakeCmdMsg::CAN_ID = 0x6B;
const int64_t AS::Drivers::PACMod3::SteerCmdMsg::CAN_ID = 0x6D;
const int64_t AS::Drivers::PACMod3::BrakeRptMsg::CAN_ID = 0x6C;
const int64_t AS::Drivers::PACMod3::SteerRptMsg::CAN_ID = 0x6E;
const int64_t AS::Drivers::PACMod3::VehicleSpeedRptMsg::CAN_ID = 0x6F;
const int64_t AS::Drivers::PACMod3::BrakeMotorRpt1Msg::CAN_ID = 0x70;
const int64_t AS::Drivers::PACMod3::BrakeMotorRpt2Msg::CAN_ID = 0x71;
const int64_t AS::Drivers::PACMod3::BrakeMotorRpt3Msg::CAN_ID = 0x72;
const int64_t AS::Drivers::PACMod3::SteerMotorRpt1Msg::CAN_ID = 0x73;
const int64_t AS::Drivers::PACMod3::SteerMotorRpt2Msg::CAN_ID = 0x74;
const int64_t AS::Drivers::PACMod3::SteerMotorRpt3Msg::CAN_ID = 0x75;
const int64_t AS::Drivers::PACMod3::HeadlightCmdMsg::CAN_ID = 0x76;
const int64_t AS::Drivers::PACMod3::HeadlightRptMsg::CAN_ID = 0x77;
const int64_t AS::Drivers::PACMod3::HornCmdMsg::CAN_ID = 0x78;
const int64_t AS::Drivers::PACMod3::HornRptMsg::CAN_ID = 0x79;
const int64_t AS::Drivers::PACMod3::WheelSpeedRptMsg::CAN_ID = 0x7A;
const int64_t AS::Drivers::PACMod3::SteeringPIDRpt1Msg::CAN_ID = 0x7B;
const int64_t AS::Drivers::PACMod3::SteeringPIDRpt2Msg::CAN_ID = 0x7C;
const int64_t AS::Drivers::PACMod3::SteeringPIDRpt3Msg::CAN_ID = 0x7D;
const int64_t AS::Drivers::PACMod3::SteerRpt2Msg::CAN_ID = 0x7E;
const int64_t AS::Drivers::PACMod3::SteerRpt3Msg::CAN_ID = 0x7F;
const int64_t AS::Drivers::PACMod3::ParkingBrakeRptMsg::CAN_ID = 0x80;
const int64_t AS::Drivers::PACMod3::YawRateRptMsg::CAN_ID = 0x81;
const int64_t AS::Drivers::PACMod3::LatLonHeadingRptMsg::CAN_ID = 0x82;
const int64_t AS::Drivers::PACMod3::DateTimeRptMsg::CAN_ID = 0x83;
const int64_t AS::Drivers::PACMod3::SteeringPIDRpt4Msg::CAN_ID = 0x84;
const int64_t AS::Drivers::PACMod3::WiperCmdMsg::CAN_ID = 0x90;
const int64_t AS::Drivers::PACMod3::WiperRptMsg::CAN_ID = 0x91;
const int64_t AS::Drivers::PACMod3::ParkingBrakeCmdMsg::CAN_ID = 0x92;
const int64_t AS::Drivers::PACMod3::VinRptMsg::CAN_ID = 0xFF;

std::shared_ptr<Pacmod3TxMsg> Pacmod3TxMsg::make_message(const int64_t& can_id)
{
  switch (can_id)
  {
    case AccelRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new AccelRptMsg);
      break;
    case BrakeMotorRpt1Msg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new BrakeMotorRpt1Msg);
      break;
    case BrakeMotorRpt2Msg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new BrakeMotorRpt2Msg);
      break;
    case BrakeMotorRpt3Msg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new BrakeMotorRpt3Msg);
      break;
    case BrakeRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new BrakeRptMsg);
      break;
    case DateTimeRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new DateTimeRptMsg);
      break;
    case GlobalRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new GlobalRptMsg);
      break;
    case HeadlightRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new HeadlightRptMsg);
      break;
    case HornRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new HornRptMsg);
      break;
    case LatLonHeadingRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new LatLonHeadingRptMsg);
      break;
    case ParkingBrakeRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new ParkingBrakeRptMsg);
      break;
    case ShiftRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new ShiftRptMsg);
      break;
    case SteeringPIDRpt1Msg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new SteeringPIDRpt1Msg);
      break;
    case SteeringPIDRpt2Msg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new SteeringPIDRpt2Msg);
      break;
    case SteeringPIDRpt3Msg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new SteeringPIDRpt3Msg);
      break;
    case SteeringPIDRpt4Msg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new SteeringPIDRpt4Msg);
      break;
    case SteerMotorRpt1Msg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new SteerMotorRpt1Msg);
      break;
    case SteerMotorRpt2Msg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new SteerMotorRpt2Msg);
      break;
    case SteerMotorRpt3Msg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new SteerMotorRpt3Msg);
      break;
    case SteerRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new SteerRptMsg);
      break;
    case SteerRpt2Msg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new SteerRpt2Msg);
      break;
    case SteerRpt3Msg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new SteerRpt3Msg);
      break;
    case TurnSignalRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new TurnSignalRptMsg);
      break;
    case VehicleSpeedRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new VehicleSpeedRptMsg);
      break;
    case VinRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new VinRptMsg);
      break;
    case WheelSpeedRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new WheelSpeedRptMsg);
      break;
    case WiperRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new WiperRptMsg);
      break;
    case YawRateRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod3TxMsg>(new YawRateRptMsg);
      break;
    default:
      return NULL;
  }
}

// TX Messages
void GlobalRptMsg::parse(uint8_t *in)
{
  enabled = in[0] & 0x01;
  override_active = ((in[0] & 0x02) >> 1) != 0;
  user_can_timeout = ((in[0] & 0x20) >> 5) != 0;
  brake_can_timeout = ((in[0] & 0x10) >> 4) != 0;
  steering_can_timeout = ((in[0] & 0x08) >> 3) != 0;
  vehicle_can_timeout = ((in[0] & 0x04) >> 2) != 0;
  user_can_read_errors = ((in[6] << 8) | in[7]);
}

void SystemRptBoolMsg::parse(uint8_t *in)
{
  enabled = ((in[0] & 0x01) > 0);
  override_active = ((in[0] & 0x02) > 0);
  system_fault = ((in[0] & 0x04) > 0);

  manual_input = ((in[1] & 0x01) > 0);
  command = ((in[1] & 0x02) > 0);
  output = ((in[1] & 0x04) > 0);
}

void SystemRptIntMsg::parse(uint8_t *in)
{
  enabled = ((in[0] & 0x01) > 0);
  override_active = ((in[0] & 0x02) > 0);
  system_fault = ((in[0] & 0x04) > 0);

  manual_input = in[1];
  command = in[2];
  output = in[3];
}

void SystemRptFloatMsg::parse(uint8_t *in)
{
  enabled = ((in[0] & 0x01) > 0);
  override_active = ((in[0] & 0x02) > 0);
  system_fault = ((in[0] & 0x04) > 0);

  int16_t temp;

  temp = ((int16_t)in[1] << 8) | in[2];
  manual_input = (double)(temp / 1000.0);

  temp = ((int16_t)in[3] << 8) | in[4];
  command = (double)(temp / 1000.0);

  temp = ((int16_t)in[5] << 8) | in[6];
  output = (double)(temp / 1000.0);
}

void DateTimeRptMsg::parse(uint8_t *in)
{
  year = in[0];
  month = in[1];
  day = in[2];
  hour = in[3];
  minute = in[4];
  second = in[5];
}

void LatLonHeadingRptMsg::parse(uint8_t *in)
{
  latitude_degrees = (int8_t)in[0];
  latitude_minutes = (uint8_t)in[1];
  latitude_seconds = (uint8_t)in[2];
  longitude_degrees = (int8_t)in[3];
  longitude_minutes = (uint8_t)in[4];
  longitude_seconds = (uint8_t)in[5];
  heading = (((int16_t)in[6] << 8) | in[7]) / 100.0;
}

void MotorRpt1Msg::parse(uint8_t *in)
{
  int32_t temp;

  temp = ((int32_t)in[0] << 24) | ((int32_t)in[1] << 16) | ((int32_t)in[2] << 8) | in[3];
  current = (double)(temp / 1000.0);
  
  temp = ((int32_t)in[4] << 24) | ((int32_t)in[5] << 16) | ((int32_t)in[6] << 8) | in[7];
  position = (double)(temp / 1000.0);
}

void MotorRpt2Msg::parse(uint8_t *in)
{
  int16_t temp16;
  int32_t temp32;

  temp16 = ((int16_t)in[0] << 8) | in[1];
  encoder_temp = (double)temp16;

  temp16 = ((int16_t)in[2] << 8) | in[3];
  motor_temp = (double)temp16;

  temp32 = ((int32_t)in[7] << 24) | ((int32_t)in[6] << 16) | ((int32_t)in[5] << 8) | in[4];
  velocity = (double)(temp32 / 1000.0);
}

void MotorRpt3Msg::parse(uint8_t *in)
{
  int32_t temp;

  temp = ((int32_t)in[0] << 24) | ((int32_t)in[1] << 16) | ((int32_t)in[2] << 8) | in[3];
  torque_output = (double)(temp / 1000.0);

  temp = ((int32_t)in[4] << 24) | ((int32_t)in[5] << 16) | ((int32_t)in[6] << 8) | in[7];
  torque_input = (double)(temp / 1000.0);
}

void SteeringPIDRpt1Msg::parse(uint8_t *in)
{
  int16_t temp;

  temp = ((int16_t)in[0] << 8) | in[1];
  dt = (double)(temp / 1000.0);

  temp = ((int16_t)in[2] << 8) | in[3];
  Kp = (double)(temp / 1000.0);

  temp = ((int16_t)in[4] << 8) | in[5];
  Ki = (double)(temp / 1000.0);

  temp = ((int16_t)in[6] << 8) | in[7];
  Kd = (double)(temp / 1000.0);
}

void SteeringPIDRpt2Msg::parse(uint8_t *in)
{
  int16_t temp;

  temp = ((int16_t)in[0] << 8) | in[1];
  P_term = (double)(temp / 1000.0);

  temp = ((int16_t)in[2] << 8) | in[3];
  I_term = (double)(temp / 1000.0);

  temp = ((int16_t)in[4] << 8) | in[5];
  D_term = (double)(temp / 1000.0);

  temp = ((int16_t)in[6] << 8) | in[7];
  all_terms = (double)(temp / 1000.0);
}

void SteeringPIDRpt3Msg::parse(uint8_t *in)
{
  int16_t temp;

  temp = ((int16_t)in[0] << 8) | in[1];
  new_torque = (double)(temp / 1000.0);

  temp = ((int16_t)in[2] << 8) | in[3];
  str_angle_desired = (double)(temp / 1000.0);

  temp = ((int16_t)in[4] << 8) | in[5];
  str_angle_actual = (double)(temp / 1000.0);

  temp = ((int16_t)in[6] << 8) | in[7];
  error = (double)(temp / 1000.0);
}

void SteeringPIDRpt4Msg::parse(uint8_t *in)
{
  int16_t temp;

  temp = ((int16_t)in[0] << 8) | in[1];
  angular_velocity = (double)(temp / 1000.0);

  temp = ((int16_t)in[2] << 8) | in[3];
  angular_acceleration = (double)(temp / 1000.0);
}

void VehicleSpeedRptMsg::parse(uint8_t *in)
{
  int16_t temp;

  temp = ((int16_t)in[0] << 8) | in[1];
  vehicle_speed = (double)(temp / 100.0);

  vehicle_speed_valid = (in[2] == 1);
  vehicle_speed_raw[0] = in[3];
  vehicle_speed_raw[1] = in[4];
}

void VinRptMsg::parse(uint8_t *in)
{
  std::ostringstream oss;
  oss << in[0] << in[1] << in[2];
  mfg_code = oss.str();

  if (mfg_code == "52C")
    mfg = "POLARIS INDUSTRIES INC.";
  else if (mfg_code == "3HS")
    mfg = "NAVISTAR, INC.";
  else if (mfg_code == "2T2")
    mfg = "TOYOTA MOTOR MANUFACTURING CANADA";

  model_year_code = in[3];

  if (model_year_code >= '1' && model_year_code <= '9')
  {
    model_year = 2000 + model_year_code;
  }
  else if (model_year_code >= 'A' && model_year_code < 'Z')
  {
    switch (model_year_code)
    {
      case 'A':
        model_year = 2010;
        break;
      case 'B':
        model_year = 2011;
        break;
      case 'C':
        model_year = 2012;
        break;
      case 'D':
        model_year = 2013;
        break;
      case 'E':
        model_year = 2014;
        break;
      case 'F':
        model_year = 2015;
        break;
      case 'G':
        model_year = 2016;
        break;
      case 'H':
        model_year = 2017;
        break;
      case 'J':
        model_year = 2018;
        break;
      case 'K':
        model_year = 2019;
        break;
      case 'L':
        model_year = 2020;
        break;
      case 'M':
        model_year = 2021;
        break;
      case 'N':
        model_year = 2022;
        break;
      case 'P':
        model_year = 2023;
        break;
      case 'R':
        model_year = 2024;
        break;
      case 'S':
        model_year = 2025;
        break;
      case 'T':
        model_year = 2026;
        break;
      case 'V':
        model_year = 2027;
        break;
      case 'W':
        model_year = 2028;
        break;
      case 'X':
        model_year = 2029;
        break;
      case 'Y':
        model_year = 2030;
        break;
    }
  }

  serial = (in[4] & 0x0F);
  serial = (serial << 8) | in[5];
  serial = (serial << 8) | in[6];
}

void WheelSpeedRptMsg::parse(uint8_t *in)
{
  int16_t temp;

  temp = ((int16_t)in[0] << 8) | in[1];
  front_left_wheel_speed = (double)(temp / 100.0);

  temp = ((int16_t)in[2] << 8) | in[3];
  front_right_wheel_speed = (double)(temp / 100.0);

  temp = ((int16_t)in[4] << 8) | in[5];
  rear_left_wheel_speed = (double)(temp / 100.0);

  temp = ((int16_t)in[6] << 8) | in[7];
  rear_right_wheel_speed = (double)(temp / 100.0);
}

void YawRateRptMsg::parse(uint8_t *in)
{
  int16_t temp;
  
  temp = ((int16_t)in[0] << 8) | in[1];
  yaw_rate = (double)(temp / 100.0);
}

// RX Messages
void SystemCmdBool::encode(bool enable,
                           bool ignore_overrides,
                           bool clear_override,
                           bool cmd)
{
  data.assign(8, 0);

  data[0] = enable ? 0x01 : 0x00;
  data[0] |= ignore_overrides ? 0x02 : 0x00;
  data[0] |= clear_override ? 0x04 : 0x00;
  data[1] = cmd ? 0x00 : 0x01;
}

void SystemCmdFloat::encode(bool enable,
                            bool ignore_overrides,
                            bool clear_override,
                            float cmd)
{
  data.assign(8, 0);

  data[0] = enable ? 0x01 : 0x00;
  data[0] |= ignore_overrides ? 0x02 : 0x00;
  data[0] |= clear_override ? 0x04 : 0x00;

  uint16_t cmd_float = (uint16_t)(cmd * 1000.0);
  data[1] = (cmd_float & 0xFF00) >> 8;
  data[2] = cmd_float & 0x00FF;
}

void SystemCmdInt::encode(bool enable,
                          bool ignore_overrides,
                          bool clear_override,
                          uint8_t cmd)
{
  data.assign(8, 0);

  data[0] = enable ? 0x01 : 0x00;
  data[0] |= ignore_overrides ? 0x02 : 0x00;
  data[0] |= clear_override ? 0x04 : 0x00;
  data[1] = cmd;
}

void SteerCmdMsg::encode(bool enable,
                         bool ignore_overrides,
                         bool clear_override,
                         float steer_pos,
                         float steer_spd)
{
  data.assign(8, 0);

  data[0] = enable ? 0x01 : 0x00;
  data[0] |= ignore_overrides ? 0x02 : 0x00;
  data[0] |= clear_override ? 0x04 : 0x00;

  int16_t raw_pos = (int16_t)(1000.0 * steer_pos);
  uint16_t raw_spd = (uint16_t)(1000.0 * steer_spd);

  data[1] = (raw_pos & 0xFF00) >> 8;
  data[2] = raw_pos & 0x00FF;
  data[3] = (raw_spd & 0xFF00) >> 8;    
  data[4] = raw_spd & 0x00FF;
}
