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

#ifndef PACMOD3_PACMOD3_ROS_MSG_HANDLER_H
#define PACMOD3_PACMOD3_ROS_MSG_HANDLER_H

#include <pacmod3/pacmod3_common.h>

#include <string>
#include <vector>

namespace AS
{
namespace Drivers
{
namespace PACMod3
{
class LockedData
{
public:
  explicit LockedData(unsigned char data_length);

  std::vector<unsigned char> getData() const;
  void setData(std::vector<unsigned char> new_data);

private:
  std::vector<unsigned char> _data;
  mutable std::mutex _data_mut;
};

class Pacmod3TxRosMsgHandler
{
public:
  void fillAndPublish(const uint32_t& can_id,
                      const std::string& frame_id,
                      const ros::Publisher& pub,
                      const std::shared_ptr<Pacmod3TxMsg>& parser_class);

private:

// General Reports
  void fillSystemRptBool(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::SystemRptBool * new_msg,
      const std::string& frame_id);
  void fillSystemRptInt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::SystemRptInt * new_msg,
      const std::string& frame_id);
  void fillSystemRptFloat(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::SystemRptFloat * new_msg,
      const std::string& frame_id);
  void fillComponentRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::ComponentRpt * new_msg,
      const std::string& frame_id);
  void fillSoftwareVersionRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::SoftwareVersionRpt * new_msg,
      const std::string& frame_id);
  void fillMotorRpt1(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::MotorRpt1 * new_msg,
      const std::string& frame_id);
  void fillMotorRpt2(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::MotorRpt2 * new_msg,
      const std::string& frame_id);
  void fillMotorRpt3(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::MotorRpt3 * new_msg,
      const std::string& frame_id);

  void fillCabinClimateRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::CabinClimateRpt * new_msg,
      const std::string& frame_id);
  void fillSafetyBrakeRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::SafetyBrakeRpt * new_msg,
      const std::string& frame_id);
  void fillSafetyFuncCriticalStopRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::SafetyFuncCriticalStopRpt * new_msg,
      const std::string& frame_id);
  void fillSafetyFuncRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::SafetyFuncRpt * new_msg,
      const std::string& frame_id);
  void fillEStopRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::EStopRpt * new_msg,
      const std::string& frame_id);
  void fillWatchdogRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::WatchdogRpt * new_msg,
      const std::string& frame_id);
  void fillWatchdogRpt2(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::WatchdogRpt2 * new_msg,
      const std::string& frame_id);

// Global
  void fillGlobalRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::GlobalRpt * new_msg,
      const std::string& frame_id);
  void fillGlobalRpt2(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::GlobalRpt2 * new_msg,
      const std::string& frame_id);

// System Aux Reports
  void fillAccelAuxRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::AccelAuxRpt * new_msg,
      const std::string& frame_id);
  void fillBrakeAuxRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::BrakeAuxRpt * new_msg,
      const std::string& frame_id);
  void fillBrakeDeccelAuxRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::BrakeDeccelAuxRpt * new_msg,
      const std::string& frame_id);
  void fillHeadlightAuxRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::HeadlightAuxRpt * new_msg,
      const std::string& frame_id);
  void fillParkingBrakeAuxRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::ParkingBrakeAuxRpt * new_msg,
      const std::string& frame_id);
  void fillShiftAuxRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::ShiftAuxRpt * new_msg,
      const std::string& frame_id);
  void fillSteerAuxRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::SteerAuxRpt * new_msg,
      const std::string& frame_id);
  void fillTurnAuxRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::TurnAuxRpt * new_msg,
      const std::string& frame_id);
  void fillWiperAuxRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::WiperAuxRpt * new_msg,
      const std::string& frame_id);

// Misc Reports
  void fillAngVelRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::AngVelRpt * new_msg,
      const std::string& frame_id);
  void fillDateTimeRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::DateTimeRpt * new_msg,
      const std::string& frame_id);
  void fillDetectedObjectRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::DetectedObjectRpt * new_msg,
      const std::string& frame_id);
  void fillDoorRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::DoorRpt * new_msg,
      const std::string& frame_id);
  void fillDriveTrainRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::DriveTrainRpt * new_msg,
      const std::string& frame_id);
  void fillEngineRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::EngineRpt * new_msg,
      const std::string& frame_id);
  void fillInteriorLightsRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::InteriorLightsRpt * new_msg,
      const std::string& frame_id);
  void fillLatLonHeadingRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::LatLonHeadingRpt * new_msg,
      const std::string& frame_id);
  void fillLinearAccelRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::LinearAccelRpt * new_msg,
      const std::string& frame_id);
  void fillOccupancyRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::OccupancyRpt * new_msg,
      const std::string& frame_id);
  void fillRearLightsRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::RearLightsRpt * new_msg,
      const std::string& frame_id);
  void fillTirePressureRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::TirePressureRpt * new_msg,
      const std::string& frame_id);
  void fillVehDynamicsRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::VehDynamicsRpt * new_msg,
      const std::string& frame_id);
  void fillVehicleSpeedRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::VehicleSpeedRpt * new_msg,
      const std::string& frame_id);
  void fillVinRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::VinRpt * new_msg,
      const std::string& frame_id);
  void fillWheelSpeedRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::WheelSpeedRpt * new_msg,
      const std::string& frame_id);
  void fillYawRateRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::YawRateRpt * new_msg,
      const std::string& frame_id);

  void fillSystemCmdLimitRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::SystemCmdLimitRpt * new_msg,
      const std::string& frame_id);
  void fillSteerCmdLimitRpt(
      const std::shared_ptr<Pacmod3TxMsg>& parser_class,
      pacmod_msgs::SteerCmdLimitRpt * new_msg,
      const std::string& frame_id);

};

class Pacmod3RxRosMsgHandler
{
public:
  static std::vector<uint8_t> unpackAndEncode(const uint32_t& can_id, const pacmod_msgs::SystemCmdBool::ConstPtr& msg);
  static std::vector<uint8_t> unpackAndEncode(const uint32_t& can_id, const pacmod_msgs::SystemCmdFloat::ConstPtr& msg);
  static std::vector<uint8_t> unpackAndEncode(const uint32_t& can_id, const pacmod_msgs::SystemCmdInt::ConstPtr& msg);
  static std::vector<uint8_t> unpackAndEncode(const uint32_t& can_id, const pacmod_msgs::SteerSystemCmd::ConstPtr& msg);
  static std::vector<uint8_t> unpackAndEncode(const uint32_t& can_id, const pacmod_msgs::GlobalCmd::ConstPtr& msg);
  static std::vector<uint8_t> unpackAndEncode(const uint32_t& can_id, const pacmod_msgs::BrakeDeccelCmd::ConstPtr& msg);
  static std::vector<uint8_t> unpackAndEncode(const uint32_t& can_id, const pacmod_msgs::CabinClimateCmd::ConstPtr& msg);
  static std::vector<uint8_t> unpackAndEncode(const uint32_t& can_id, const pacmod_msgs::SafetyBrakeCmd::ConstPtr& msg);
  static std::vector<uint8_t> unpackAndEncode(const uint32_t& can_id, const pacmod_msgs::SafetyFuncCmd::ConstPtr& msg);
  static std::vector<uint8_t> unpackAndEncode(const uint32_t& can_id, const pacmod_msgs::SupervisoryCtrl::ConstPtr& msg);
  static std::vector<uint8_t> unpackAndEncode(const uint32_t& can_id, const pacmod_msgs::NotificationCmd::ConstPtr& msg);

};

typedef union
{
    uint8_t u8Arr[9];
    struct __attribute__((packed))
    {
        uint8_t GLOBAL            : 1;
        uint8_t GLOBAL_2          : 1;
        uint8_t ESTOP             : 1;
        uint8_t WATCHDOG          : 1;
        uint8_t WATCHDOG_2        : 1;

        uint8_t ACCEL             : 1;
        uint8_t BRAKE             : 1;
        uint8_t SHIFT             : 1;
        uint8_t STEER             : 1;

        uint8_t ACCEL_AUX         : 1;
        uint8_t BRAKE_AUX         : 1;
        uint8_t SHIFT_AUX         : 1;
        uint8_t STEER_AUX         : 1;

        uint8_t COMP_0            : 1;
        uint8_t COMP_1            : 1;
        uint8_t COMP_2            : 1;
        uint8_t COMP_3            : 1;
        uint8_t COMP_4            : 1;
        uint8_t SOFTW_0           : 1;
        uint8_t SOFTW_1           : 1;
        uint8_t SOFTW_2           : 1;
        uint8_t SOFTW_3           : 1;
        uint8_t SOFTW_4           : 1;

        uint8_t ACCEL_CMD_LIMIT   : 1;
        uint8_t BRAKE_CMD_LIMIT   : 1;
        uint8_t STEER_CMD_LIMIT   : 1;

        uint8_t BRAKE_MOTOR_1     : 1;
        uint8_t BRAKE_MOTOR_2     : 1;
        uint8_t BRAKE_MOTOR_3     : 1;
        uint8_t STEER_MOTOR_1     : 1;
        uint8_t STEER_MOTOR_2     : 1;
        uint8_t STEER_MOTOR_3     : 1;

        uint8_t BRAKE_DECCEL      : 1;
        uint8_t BRAKE_DECCEL_AUX  : 1;
        uint8_t CABIN_CLIMATE     : 1;
        uint8_t CABIN_FAN_SPEED   : 1;
        uint8_t CABIN_TEMP        : 1;
        uint8_t CRUISE_CONTROL    : 1;
        uint8_t DASH_LEFT         : 1;
        uint8_t DASH_RIGHT        : 1;
        uint8_t ENGINE_BRAKE      : 1;
        uint8_t HAZARDS           : 1;
        uint8_t HEADLIGHTS        : 1;
        uint8_t HEADLIGHTS_AUX    : 1;
        uint8_t HORN              : 1;
        uint8_t MARKER_LAMP       : 1;
        uint8_t MEDIA_CONTROLS    : 1;
        uint8_t PARKING_BRAKE     : 1;
        uint8_t PARKING_BRAKE_AUX : 1;
        uint8_t REAR_PASS_DOOR    : 1;
        uint8_t SPRAY             : 1;
        uint8_t TURN              : 1;
        uint8_t TURN_AUX          : 1;
        uint8_t WIPER             : 1;
        uint8_t WIPER_AUX         : 1;

        uint8_t ANG_VEL           : 1;
        uint8_t DATE_TIME         : 1;
        uint8_t DETECTED_OBJECT   : 1;
        uint8_t DOOR              : 1;
        uint8_t DRIVETRAIN_FT     : 1;
        uint8_t ENGINE            : 1;
        uint8_t INTERIOR_LIGHTS   : 1;
        uint8_t LAT_LON_HEADING   : 1;
        uint8_t LINEAR_ACCEL      : 1;
        uint8_t OCCUPANCY         : 1;
        uint8_t REAR_LIGHTS       : 1;
        uint8_t TIRE_PRESSURE     : 1;
        uint8_t VEHICLE_DYNAMICS  : 1;
        uint8_t WHEEL_SPEED       : 1;
        uint8_t YAW_RATE          : 1;

        uint8_t VEHICLE_SPEED     : 1;
        uint8_t VIN               : 1;
    };
} ReportPresent;

typedef union
{
    uint8_t u8Arr[3];
    struct __attribute__((packed))
    {
        uint8_t GLOBAL            : 1;
        uint8_t USER_NOTIFICATION : 1;

        uint8_t ACCEL             : 1;
        uint8_t BRAKE             : 1;
        uint8_t SHIFT             : 1;
        uint8_t STEER             : 1;

        uint8_t BRAKE_DECCEL      : 1;
        uint8_t CABIN_CLIMATE     : 1;
        uint8_t CABIN_FAN_SPEED   : 1;
        uint8_t CABIN_TEMP        : 1;
        uint8_t CRUISE_CONTROL    : 1;
        uint8_t DASH_LEFT         : 1;
        uint8_t DASH_RIGHT        : 1;
        uint8_t ENGINE_BRAKE      : 1;
        uint8_t HAZARDS           : 1;
        uint8_t HEADLIGHTS        : 1;
        uint8_t HORN              : 1;
        uint8_t MARKER_LAMP       : 1;
        uint8_t MEDIA_CONTROLS    : 1;
        uint8_t PARKING_BRAKE     : 1;
        uint8_t REAR_PASS_DOOR    : 1;
        uint8_t SPRAY             : 1;
        uint8_t TURN              : 1;
        uint8_t WIPER             : 1;
    };
} CommandPresent;

}  // namespace PACMod3
}  // namespace Drivers
}  // namespace AS

#endif  // PACMOD3_PACMOD3_ROS_MSG_HANDLER_H
