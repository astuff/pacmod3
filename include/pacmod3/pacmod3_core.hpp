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

#ifndef PACMOD3__PACMOD3_CORE_HPP_
#define PACMOD3__PACMOD3_CORE_HPP_

#include <cstring>
#include <sstream>
#include <cstdint>
#include <memory>
#include <vector>
#include <string>

namespace pacmod3
{

/*** Enum Classes ***/

  enum CalibrationStatus
  {
    INACTIVE = 0,
    ACTIVE = 1,
    COMPLETE = 2,
    ERROR = 3
  };

  enum class ComponentType
  {
    PACMOD = 0,
    PACMINI = 1,
    PACMICRO = 2,
    NONE = 15
  };

  enum class DimLevel
  {
    DIM_LEVEL_MIN = 0,
    DIM_LEVEL_1 = 1,
    DIM_LEVEL_2 = 2,
    DIM_LEVEL_3 = 3,
    DIM_LEVEL_4 = 4,
    DIM_LEVEL_5 = 5,
    DIM_LEVEL_6 = 6,
    DIM_LEVEL_7 = 7,
    DIM_LEVEL_8 = 8,
    DIM_LEVEL_9 = 9,
    DIM_LEVEL_10 = 10,
    DIM_LEVEL_11 = 11,
    DIM_LEVEL_MAX = 12,
    DIM_LEVEL_12 = 13,
    DIM_LEVEL_13 = 14,
    DIM_LEVEL_14 = 15,
    DIM_LEVEL_15 = 16,
    DIM_LEVEL_16 = 17,
    DIM_LEVEL_17 = 18,
    DIM_LEVEL_18 = 19,
    DIM_LEVEL_19 = 20,
    DIM_LEVEL_20 = 21,
    DIM_LEVEL_21 = 22,
    DIM_LEVEL_22 = 23,
    DIM_LEVEL_23 = 24
  };

  enum DriveMode
  {
    NORMAL = 0,
    ECO = 1,
    SPORT = 2
  };

  enum class Gears : int8_t
  {
    R_TENTH = -10,
    R_NINTH = -9,
    R_EIGHTH = -8,
    R_SEVENTH = -7,
    R_SIXTH = -6,
    R_FIFTH = -5,
    R_FOURTH = -4,
    R_THIRD = -3,
    R_SECOND = -2,
    R_FIRST = -1,
    NEUTRAL = 0,
    FIRST = 1,
    SECOND = 2,
    THIRD = 3,
    FOURTH = 4,
    FIFTH = 5,
    SIXTH = 6,
    SEVENTH = 7,
    EIGHTH = 8,
    NINTH = 9,
    TENTH = 10,
    ELEVENTH = 11,
    TWELFTH = 12,
    THIRTEENTH = 13,
    FOURTEENTH = 14,
    FIFTEENTH = 15,
    SIXTEENTH = 16,
    SEVENTEENTH = 17,
    EIGHTEENTH = 18
  };

  enum class HeadlightSystemState
  {
    HEADLIGHTS_SYSTEM_OFF = 0,
    HEADLIGHTS_SYSTEM_PARKING = 1,
    HEADLIGHTS_SYSTEM_MANUAL = 2,
    HEADLIGHTS_SYSTEM_AUTO = 3
  };

  enum class VehicleType
  {
    FREIGHTLINER_CASCADIA,
    INTERNATIONAL_PROSTAR_122,
    JUPITER_SPIRIT,
    LEXUS_RX_450H,
    POLARIS_GEM,
    POLARIS_RANGER,
    HEXAGON_TRACTOR,
    FORD_RANGER,
    VEHICLE_FTT,
    VEHICLE_HCV,
    VEHICLE_4,
    VEHICLE_5,
    VEHICLE_6
  };

  enum class XBR_EBI_Mode
  {
    NO_ENDURANCE_BRAKE_INTEGRATION_ALLOWED = 0,
    ONLY_ENDURANCE_BRAKES_ALLOWED = 1,
    ENDURANCE_BRAKE_INTEGRATION_ALLOWED = 2
  };

  enum class XBRPriority
  {
    HIGHEST_PRIORITY = 0,
    HIGH_PRIORITY = 1,
    MEDIUM_PRIORITY = 2,
    LOW_PRIORITY = 3
  };

  enum class XBRControlMode
  {
    OVERRIDE_DISABLE = 0,
    ACCELERATION_CONTROL_WITH_ADDITION_MODE = 1,
    ACCELERATION_CONTROL_WITH_MAXIMUM_MODE = 2
  };

  enum class XBRActiveControlMode
  {
    NO_BRAKE_DEMAND = 0,
    DRIVERS_BRAKE_DEMAND = 1,
    ADDITION_MODE_OF_XBR_ACCELERATION_CONTROL = 2,
    MAXIMUM_MODE_OF_XBR_ACCELERATION_CONTROL = 3
  };

  enum class XBRSystemState
  {
    ANY_EXTERNAL_BRAKE_DEMAND_WILL_BE_ACCEPTED = 0,
    NO_EXTERNAL_BRAKE_DEMAND_WILL_BE_ACCEPTED = 2
  };

  enum class FoundationBrakeState
  {
    FOUNDATION_BRAKES_NOT_IN_USE = 0,
    FOUNDATION_BRAKES_IN_USE = 1
  };

  enum class HillHolderMode
  {
    HH_INACTIVE = 0,
    HH_ACTIVE = 1,
    HH_ACTIVE_BUT_INACTIVE_SOON = 2,
    HH_ERROR = 6
  };

  enum class SafetyFunctionCommand
  {
    CMD_NONE = 0,
    CMD_AUTO_READY = 1,
    CMD_AUTO = 2,
    CMD_MANUAL_READY = 3,
    CMD_CRITICAL_STOP1 = 4,
    CMD_CRITICAL_STOP2 = 5
  };

  enum class SafetyFunctionState
  {
    MANUAL_BRAKED = 0,
    MANUAL_UNBRAKED = 1,
    AUTOMS_READY = 2,
    AUTOMS_INACTIVE = 3,
    AUTOMS_ACTIVE_BRAKED = 4,
    AUTOMS_ACTIVE_UNBRAKED = 5,
    MANUAL_READY = 6,
    CRITICAL_STOP1 = 7,
    CRITICAL_STOP2 = 8,
    STARTUP = 9
  };

  enum class AutomsManOpCtrl
  {
    AUTOMS_MAN_INVALID = 0,
    AUTOMS_MAN_MANUAL = 1,
    AUTOMS_MAN_AUTOMS = 2
  };
  
  enum class CabinSafetyBrakeState
  {
    CABIN_BRAKE_INVALID = 0,
    CABIN_BRAKE_APPLIED = 1,
    CABIN_BRAKE_UNAPPLIED = 2
  };

  enum class RemoteStopState
  {
    REMOTE_STOP_STATE_INVALID = 0,
    REMOTE_STOP_STATE_GO = 1,
    REMOTE_STOP_STATE_STOP = 2
  };

  enum class SafetyFuncFaults
  {
    OKAY = 0,
    FAULT = 1,
    TIMEOUT = 2
  };

/*** Message Classes ***/
class Pacmod3RxMsg
{
public:
  std::vector<uint8_t> data;
};

class Pacmod3TxMsg
{
public:
  static std::shared_ptr<Pacmod3TxMsg> make_message(const uint32_t & can_id);
  virtual void parse(const std::vector<uint8_t> & in) = 0;
  virtual bool isSystem();
};

// General Classes
class SystemCmdBool : public Pacmod3RxMsg
{
public:
  static constexpr uint8_t DATA_LENGTH = 2;

  void encode(
    bool enable,
    bool ignore_overrides,
    bool clear_override,
    bool cmd);
};

class SystemCmdFloat : public Pacmod3RxMsg
{
public:
  static constexpr uint8_t DATA_LENGTH = 3;

  void encode(
    bool enable,
    bool ignore_overrides,
    bool clear_override,
    float cmd);
};

class SystemCmdInt : public Pacmod3RxMsg
{
public:
  static constexpr uint8_t DATA_LENGTH = 2;

  void encode(
    bool enable,
    bool ignore_overrides,
    bool clear_override,
    uint8_t cmd);
};

class SystemCmdLimitRptMsg : public  Pacmod3TxMsg
{
public:
  SystemCmdLimitRptMsg();

  double sys_cmd_limit;
  double limited_sys_cmd;

  void parse(const std::vector<uint8_t> & in);
};

class SystemRptMsg : public Pacmod3TxMsg
{
public:
  SystemRptMsg();

  bool isSystem();

  bool enabled;
  bool override_active;
  bool command_output_fault;
  bool input_output_fault;
  bool output_reported_fault;
  bool pacmod_fault;
  bool vehicle_fault;
  bool command_timeout;
};

class SystemRptBoolMsg : public SystemRptMsg
{
public:
  SystemRptBoolMsg();

  bool manual_input;
  bool command;
  bool output;

  void parse(const std::vector<uint8_t> & in);
};

class SystemRptIntMsg : public SystemRptMsg
{
public:
  SystemRptIntMsg();

  uint8_t manual_input;
  uint8_t command;
  uint8_t output;

  void parse(const std::vector<uint8_t> & in);
};

class SystemRptFloatMsg : public SystemRptMsg
{
public:
  SystemRptFloatMsg();

  double manual_input;
  double command;
  double output;

  void parse(const std::vector<uint8_t> & in);
};

class ComponentRptMsg : public Pacmod3TxMsg
{
  public:
    ComponentType component_type;

    bool accel;
    bool brake;
    bool cruise_control_buttons;
    bool dash_controls_left;
    bool dash_controls_right;
    bool hazard_lights;
    bool headlight;
    bool horn;
    bool media_controls;
    bool parking_brake;
    bool shift;
    bool sprayer;
    bool steering;
    bool turn;
    bool wiper;
    bool watchdog;
    bool brake_decel;
    bool rear_pass_door;
    bool engine_brake;
    bool marker_lamp;
    bool cabin_climate;
    bool cabin_fan_speed;
    bool cabin_temp;
    bool exhaust_brake;

    uint8_t counter;
    uint8_t complement;
    bool config_fault;
    bool can_timeout_fault;
    bool internal_supply_voltage_fault;
    bool supervisory_timeout;           
    bool supervisory_sanity_fault;
    bool watchdog_sanity_fault;
    bool watchdog_system_present_fault;
    bool component_ready;

  void parse(const std::vector<uint8_t> & in);
};

class SoftwareVersionRptMsg : public Pacmod3TxMsg
{
  public:
    uint8_t mjr;
    uint8_t mnr;
    uint8_t patch;
    char build0;
    char build1;
    char build2;
    char build3;

  void parse(const std::vector<uint8_t> & in);
};

class MotorRpt1Msg : public Pacmod3TxMsg
{
public:
  double current;
  double position;

  void parse(const std::vector<uint8_t> & in);
};

class MotorRpt2Msg : public Pacmod3TxMsg
{
public:
  double encoder_temp;
  double motor_temp;
  double velocity;

  void parse(const std::vector<uint8_t> & in);
};

class MotorRpt3Msg : public Pacmod3TxMsg
{
public:
  double torque_output;
  double torque_input;

  void parse(const std::vector<uint8_t> & in);
};

// Global
class GlobalCmdMsg : public Pacmod3RxMsg
{
  public:
    static constexpr uint8_t DATA_LENGTH = 1;
    static constexpr uint32_t CAN_ID = 0x80;

    void encode(bool clear_faults,
                bool sanity_check_required,
                uint8_t counter,
                uint8_t complement);
};

class GlobalRptMsg : public Pacmod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x10;

  bool enabled;
  bool override_active;
  bool user_can_timeout;
  bool steering_can_timeout;
  bool brake_can_timeout;
  bool subsystem_can_timeout;
  bool vehicle_can_timeout;
  bool pacmod_sys_fault_active;
  bool supervisory_enable_required;
  bool config_fault_active;
  uint16_t user_can_read_errors;

  void parse(const std::vector<uint8_t> & in);
};

class GlobalRpt2Msg : public Pacmod3TxMsg
{
  public:
    static constexpr uint32_t CAN_ID = 0x11;

    bool system_enabled;
    bool system_override_active;
    bool system_fault_active;
    bool supervisory_enable_required;
    bool disable_all_systems;
    bool system_ready;

  void parse(const std::vector<uint8_t> & in);
};

class SupervisoryCtrlMsg : public Pacmod3RxMsg
{
  public:

    static constexpr uint8_t DATA_LENGTH = 1;
    static constexpr uint32_t CAN_ID = 0x81;

    void encode(bool enabled,
                uint8_t counter,
                uint8_t complement);

};

// System Commands
class AccelCmdMsg : public SystemCmdFloat
{
public:
  static constexpr uint32_t CAN_ID = 0x100;
};

class BrakeCmdMsg : public SystemCmdFloat
{
public:
  static constexpr uint32_t CAN_ID = 0x104;
};

class BrakeDecelCmdMsg : public Pacmod3RxMsg
{
public:
  static constexpr uint8_t DATA_LENGTH = 4;

  static constexpr uint32_t CAN_ID = 0x13C;

  void encode(bool enable,
              bool ignore_overrides,
              bool clear_override,
              float brake_decel_command,
              uint8_t xbr_ebi_mode,
              uint8_t xbr_priority,
              uint8_t xbr_control_mode);
};

class CabinClimateCmdMsg : public Pacmod3RxMsg
{
public:
  static constexpr uint8_t DATA_LENGTH = 3;

  static constexpr uint32_t CAN_ID = 0x154;

  void encode(bool enable,
              bool ignore_overrides,
              bool clear_override,
              uint8_t cmd_ac_off_on,
              uint8_t cmd_max_ac_off_on,
              uint8_t cmd_defrost_off_on,
              uint8_t cmd_max_defrost_off_on,
              uint8_t cmd_dir_up_off_on,
              uint8_t cmd_dir_down_off_on);
};

class CabinFanSpeedCmdMsg : public SystemCmdInt
{
public:
  static constexpr uint32_t CAN_ID = 0x150;
};

class CabinTempCmdMsg : public SystemCmdFloat
{
public:
  static constexpr uint32_t CAN_ID = 0x14C;
};

class CruiseControlButtonsCmdMsg : public SystemCmdInt
{
public:
  static constexpr uint32_t CAN_ID = 0x108;
};

class DashControlsLeftCmdMsg : public SystemCmdInt
{
public:
  static constexpr uint32_t CAN_ID = 0x10C;
};

class DashControlsRightCmdMsg : public SystemCmdInt
{
public:
  static constexpr uint32_t CAN_ID = 0x110;
};

class EngineBrakeCmdMsg : public SystemCmdInt
{
public:
  static constexpr uint32_t CAN_ID = 0x144;
};

class ExhaustBrakeCmdMsg : public SystemCmdInt
{
public:
  static constexpr uint32_t CAN_ID = 0x145;
};

class HazardLightCmdMsg : public SystemCmdBool
{
public:
  static constexpr uint32_t CAN_ID = 0x114;
};

class HeadlightCmdMsg : public SystemCmdInt
{
public:
  static constexpr uint32_t CAN_ID = 0x118;
};

class HornCmdMsg : public SystemCmdBool
{
public:
  static constexpr uint32_t CAN_ID = 0x11C;
};

class MarkerLampCmdMsg : public SystemCmdBool
{
public:
  static constexpr uint32_t CAN_ID = 0x148;
};

class MediaControlsCmdMsg : public SystemCmdInt
{
public:
  static constexpr uint32_t CAN_ID = 0x120;
};

class ParkingBrakeCmdMsg : public SystemCmdBool
{
public:
  static constexpr uint32_t CAN_ID = 0x124;
};

class RearPassDoorCmdMsg : public SystemCmdInt
{
public:
  static constexpr uint32_t CAN_ID = 0x140;
};

class SafetyBrakeCmdMsg : public Pacmod3RxMsg
{
public:
  static constexpr uint8_t DATA_LENGTH = 1;

  static constexpr uint32_t CAN_ID = 0xC1;

  void encode(bool safety_brake_cmd);
};

class SafetyFuncCmdMsg : public Pacmod3RxMsg
{
  public:
    static constexpr uint8_t DATA_LENGTH = 1;

    static constexpr uint32_t CAN_ID = 0xC0;

    void encode(uint8_t command);
};

class ShiftCmdMsg : public SystemCmdInt
{
public:
  static constexpr uint32_t CAN_ID = 0x128;
};

class SprayerCmdMsg : public SystemCmdBool
{
public:
  static constexpr uint32_t CAN_ID = 0x138;
};

class SteerCmdMsg : public SystemCmdFloat
{
public:
  static constexpr uint32_t CAN_ID = 0x12C;
  static constexpr uint8_t DATA_LENGTH = 5;

  void encode(
    bool enabled,
    bool ignore_overrides,
    bool clear_override,
    float steer_pos,
    float steer_spd);
};

class TurnSignalCmdMsg : public SystemCmdInt
{
public:
  static constexpr uint32_t CAN_ID = 0x130;
};

class UserNotificationCmdMsg : public Pacmod3RxMsg
{
public:
  static constexpr uint8_t DATA_LENGTH = 1;

  static constexpr uint32_t CAN_ID = 0x41D;

  void encode(bool buzzer_mute,
              bool underdash_lights_white);
};

class WiperCmdMsg : public SystemCmdInt
{
public:
  static constexpr uint32_t CAN_ID = 0x134;
};

// System Reports
class AccelRptMsg : public SystemRptFloatMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x200;
};

class BrakeRptMsg : public SystemRptFloatMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x204;
};

class BrakeDecelRptMsg : public SystemRptFloatMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x23C;
};

class CabinClimateRptMsg : public SystemRptMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x254;

  uint8_t man_ac_off_on;
  uint8_t man_max_ac_off_on;
  uint8_t man_defrost_off_on;
  uint8_t man_max_defrost_off_on;
  uint8_t man_dir_up_off_on;
  uint8_t man_dir_down_off_on;
  uint8_t cmd_ac_off_on;
  uint8_t cmd_max_ac_off_on;
  uint8_t cmd_defrost_off_on;
  uint8_t cmd_max_defrost_off_on;
  uint8_t cmd_dir_up_off_on;
  uint8_t cmd_dir_down_off_on;
  uint8_t out_ac_off_on;
  uint8_t out_max_ac_off_on;
  uint8_t out_defrost_off_on;
  uint8_t out_max_defrost_off_on;
  uint8_t out_dir_up_off_on;
  uint8_t out_dir_down_off_on;

  void parse(const std::vector<uint8_t> & in);
};

class CabinFanSpeedRptMsg : public SystemRptIntMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x250;
};

class CabinTempRptMsg : public SystemRptFloatMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x24C;
};

class CruiseControlButtonsRptMsg : public SystemRptIntMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x208;
};

class DashControlsLeftRptMsg : public SystemRptIntMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x20C;
};

class DashControlsRightRptMsg : public SystemRptIntMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x210;
};

class EngineBrakeRptMsg : public SystemRptIntMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x244;
};

class ExhaustBrakeRptMsg : public SystemRptIntMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x245;
};

class HazardLightRptMsg : public SystemRptBoolMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x214;
};

class HeadlightRptMsg : public SystemRptIntMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x218;
};

class HornRptMsg : public SystemRptBoolMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x21C;
};

class MarkerLampRptMsg : public SystemRptBoolMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x248;
};

class MediaControlsRptMsg : public SystemRptIntMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x220;
};

class ParkingBrakeRptMsg : public SystemRptBoolMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x224;
};

class RearPassDoorRptMsg : public SystemRptIntMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x240;
};

class SafetyBrakeRptMsg : public Pacmod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x41;

  bool commanded_val;
  bool output_val;
  bool reported_fault;
  bool cmd_reported_fault;
  bool cmd_timeout;
  bool cmd_permitted;

  void parse(const std::vector<uint8_t> & in);
};

class SafetyFuncCriticalStopRptMsg : public Pacmod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x420;

  bool automsman_opctrl_fault;
  bool remote_stop_fault;
  bool safety_brake_opctrl_off;
  bool safety_brake_cmd_timeout;
  bool safety_func_cmd_timeout;
  bool safety_func_critical_stop_1_cmd;
  bool safety_func_critical_stop_2_cmd;
  bool safety_func_none_cmd;
  bool pacmod_system_timeout;
  bool pacmod_system_fault;
  bool pacmod_system_not_active;
  bool vehicle_report_timeout;
  bool vehicle_report_fault;
  bool low_engine_rpm;
  bool pri_safety_brake_signal_1_fault;
  bool pri_safety_brake_signal_2_fault;
  bool sec_safety_brake_signal_1_fault;
  bool sec_safety_brake_signal_2_fault;
  bool primary_processor_fault;
  bool secondary_processor_fault;
  bool remote_stop_cmd;
  bool pri_safety_brake_pressure_fault;

  void parse(const std::vector<uint8_t> & in);
};

class SafetyFuncRptMsg : public Pacmod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x40;

  SafetyFunctionCommand commanded_val;
  SafetyFunctionState state;
  AutoManualOpCtrl automanual_opctrl;
  CabinSafetyBrakeState cabin_safety_brake_opctrl;
  RemoteStopState remote_stop_status;
  bool engine_status;
  bool pacmod_system_status;
  SafetyFuncFaults user_pc_fault;
  SafetyFuncFaults pacmod_system_fault;
  SafetyFuncFaults vehicle_fault;
  bool manual_state_obtainable;
  bool auto_ready_state_obtainable;
  bool auto_state_obtainable;
  bool manual_ready_state_obtainable;
  bool critical_stop1_state_obtainable;
  bool critical_stop2_state_obtainable;

  void parse(const std::vector<uint8_t> & in);
};

class ShiftRptMsg : public SystemRptIntMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x228;
};

class SprayerRptMsg : public SystemRptBoolMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x238;
};

class SteerRptMsg : public SystemRptFloatMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x22C;
};

class TurnSignalRptMsg : public SystemRptIntMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x230;
};

class WiperRptMsg : public SystemRptIntMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x234;
};

// System Aux Reports
class AccelAuxRptMsg : public Pacmod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x300;

  bool operator_interaction;
  bool accel_limiting_active;
  bool park_brake_interlock_active;
  bool brake_interlock_active;
  uint8_t calibration_status;
  bool operator_interaction_avail;
  bool accel_limiting_active_avail;
  bool park_brake_interlock_active_avail;
  bool brake_interlock_active_avail;

  void parse(const std::vector<uint8_t> & in);
};

class BrakeAuxRptMsg : public Pacmod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x304;

  float brake_pressure;
  bool operator_interaction;
  bool brake_on_off;
  bool brake_limiting_active;
  bool brake_reduced_assist;
  uint8_t calibration_status;
  bool brake_pressure_avail;
  bool operator_interaction_avail;
  bool brake_on_off_avail;
  bool brake_limiting_active_avail;
  bool brake_reduced_assist_avail;

  void parse(const std::vector<uint8_t> & in);
};

class BrakeDecelAuxRptMsg : public Pacmod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x338;

  XBRActiveControlMode xbr_active_control_mode;
  XBRSystemState xbr_system_state;
  FoundationBrakeState foundation_brake_use;
  HillHolderMode hill_holder_mode;
  bool xbr_active_control_mode_avail;
  bool xbr_system_state_avail;
  bool foundation_brake_use_avail;    
  bool hill_holder_mode_avail;

  void parse(const std::vector<uint8_t> & in);
};

class HeadlightAuxRptMsg : public Pacmod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x318;

  bool headlights_on;
  bool headlights_on_bright;
  bool fog_lights_on;
  HeadlightSystemState headlights_mode;
  bool headlights_on_avail;
  bool headlights_on_bright_avail;
  bool fog_lights_on_avail;
  bool headlights_mode_avail;

  void parse(const std::vector<uint8_t> & in);
};

class ParkingBrakeAuxRptMsg : public Pacmod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x324;

  uint8_t parking_brake_status;
  bool parking_brake_status_avail;

  void parse(const std::vector<uint8_t> & in);
};

class ShiftAuxRptMsg : public Pacmod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x328;

  bool between_gears;
  bool stay_in_neutral_mode;
  bool brake_interlock_active;
  bool speed_interlock_active;
  bool write_to_config;
  uint8_t calibration_status;
  bool between_gears_avail;
  bool stay_in_neutral_mode_avail;
  bool brake_interlock_active_avail;
  bool speed_interlock_active_avail;
  bool write_to_config_is_valid;
  bool gear_number_avail;
  Gears gear_number;

  void parse(const std::vector<uint8_t> & in);
};

class SteerAuxRptMsg : public Pacmod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x32C;

  float steering_torque;
  float rotation_rate;
  bool operator_interaction;
  bool rotation_rate_sign;
  bool vehicle_angle_calib_status;
  bool steering_limiting_active;
  uint8_t calibration_status;
  bool steering_torque_avail;
  bool rotation_rate_avail;
  bool operator_interaction_avail;
  bool rotation_rate_sign_avail;
  bool vehicle_angle_calib_status_avail;
  bool steering_limiting_active_avail;

  void parse(const std::vector<uint8_t> & in);
};

class TurnAuxRptMsg : public Pacmod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x330;

  bool driver_blinker_bulb_on;
  bool passenger_blinker_bulb_on;
  bool driver_blinker_bulb_on_avail;
  bool passenger_blinker_bulb_on_avail;

  void parse(const std::vector<uint8_t> & in);
};

class WiperAuxRptMsg : public Pacmod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x334;

  bool front_wiping;
  bool front_spraying;
  bool rear_wiping;
  bool rear_spraying;
  bool spray_near_empty;
  bool spray_empty;
  bool front_wiping_avail;
  bool front_spraying_avail;
  bool rear_wiping_avail;
  bool rear_spraying_avail;
  bool spray_near_empty_avail;
  bool spray_empty_avail;

  void parse(const std::vector<uint8_t> & in);
};

// Module Reports
class ComponentRptMsg00 : public ComponentRptMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x20;
};

class ComponentRptMsg01 : public ComponentRptMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x21;
};

class ComponentRptMsg02 : public ComponentRptMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x22;
};

class ComponentRptMsg03 : public ComponentRptMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x23;
};

class ComponentRptMsg04 : public ComponentRptMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x24;
};

class SoftwareVerRptMsg00 : public SoftwareVersionRptMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x408;
};

class SoftwareVerRptMsg01 : public SoftwareVersionRptMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x409;
};

class SoftwareVerRptMsg02 : public SoftwareVersionRptMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x40A;
};

class SoftwareVerRptMsg03 : public SoftwareVersionRptMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x40B;
};

class SoftwareVerRptMsg04 : public SoftwareVersionRptMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x40C;
};

class EStopRptMsg : public Pacmod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x41C;

  bool estop_status;
  bool estop_fault;

  void parse(const std::vector<uint8_t> & in);
};

class WatchdogRptMsg : public Pacmod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x600;

  bool global_enabled_flag;
  bool global_override_active;
  bool global_command_timeout_error;
  bool global_pacmod_subsystem_timeout;
  bool global_vehicle_can_timeout;
  bool global_pacmod_system_fault_active;
  bool global_config_fault_active;
  bool global_timeout;
  bool accel_enabled;
  bool accel_override_active;
  bool accel_command_output_fault;
  bool accel_input_output_fault;
  bool accel_output_reported_fault;
  bool accel_pacmod_fault;
  bool accel_vehicle_fault;
  bool accel_timeout;
  bool brake_enabled;
  bool brake_override_active;
  bool brake_command_output_fault;
  bool brake_input_output_fault;
  bool brake_output_reported_fault;
  bool brake_pacmod_fault;
  bool brake_vehicle_fault;
  bool brake_timeout;
  bool shift_enabled;
  bool shift_override_active;
  bool shift_command_output_fault;
  bool shift_input_output_fault;
  bool shift_output_reported_fault;
  bool shift_pacmod_fault;
  bool shift_vehicle_fault;
  bool shift_timeout;
  bool steer_enabled;
  bool steer_override_active;
  bool steer_command_output_fault;
  bool steer_input_output_fault;
  bool steer_output_reported_fault;
  bool steer_pacmod_fault;
  bool steer_vehicle_fault;
  bool steer_timeout;
  bool mod1_config_fault;
  bool mod1_can_timeout;
  bool mod1_counter_fault;
  bool mod2_config_fault;
  bool mod2_can_timeout;
  bool mod2_counter_fault;
  bool mod3_config_fault;
  bool mod3_can_timeout;
  bool mod3_counter_fault;
  bool mini1_rpt_timeout;
  bool mini1_config_fault;
  bool mini1_can_timeout;
  bool mini1_counter_fault;
  bool mini2_rpt_timeout;
  bool mini2_config_fault;
  bool mini2_can_timeout;
  bool mini2_counter_fault;
  bool mini3_rpt_timeout;
  bool mini3_config_fault;
  bool mini3_can_timeout;
  bool mini3_counter_fault;
  bool mod_system_present_fault;
  bool mini_system_present_fault;
  bool global_internal_power_supply_fault;

  void parse(const std::vector<uint8_t> & in);
};

class WatchdogRpt2Msg : public Pacmod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x421;

  bool accel_rpt_timeout;
  bool brake_rpt_timeout;
  bool brake_decel_rpt_timeout;
  bool cabin_climate_rpt_timeout;
  bool cabin_fan_speed_rpt_timeout;
  bool cabin_temp_rpt_timeout;
  bool cruise_control_rpt_timeout;
  bool dash_left_rpt_timeout;
  bool dash_right_rpt_timeout;
  bool engine_brake_rpt_timeout;
  bool hazard_lights_rpt_timeout;
  bool headlight_rpt_timeout;
  bool horn_rpt_timeout;
  bool marker_lamp_rpt_timeout;
  bool media_controls_rpt_timeout;
  bool parking_brake_rpt_timeout;
  bool rear_pass_door_rpt_timeout;
  bool shift_rpt_timeout;
  bool sprayer_rpt_timeout;
  bool steering_rpt_timeout;
  bool turn_rpt_timeout;
  bool wiper_rpt_timeout;
  bool pacmod1_sanity_fault;
  bool pacmod2_sanity_fault;
  bool pacmod3_sanity_fault;
  bool pacmini1_sanity_fault;
  bool pacmini2_sanity_fault;
  bool pacmini3_sanity_fault;
  bool pacmod1_component_rpt_timeout;
  bool pacmod2_component_rpt_timeout;
  bool pacmod3_component_rpt_timeout;
  bool pacmini1_component_rpt_timeout;
  bool pacmini2_component_rpt_timeout;
  bool pacmini3_component_rpt_timeout;
  bool pacmod1_system_present_fault;
  bool pacmod2_system_present_fault;
  bool pacmod3_system_present_fault;
  bool pacmini1_system_present_fault;
  bool pacmini2_system_present_fault;
  bool pacmini3_system_present_fault;
  bool drive_mode_invalid;
  bool global_cmd_sanity_fault;
  bool global_cmd_timeout;
  bool exhaust_brake_rpt_timeout;

  void parse(const std::vector<uint8_t> & in);
};

// Misc Reports
class AngVelRptMsg : public Pacmod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x41A;

  bool pitch_new_data_rx;
  bool roll_new_data_rx;
  bool yaw_new_data_rx;
  bool pitch_valid;
  bool roll_valid;
  bool yaw_valid;
  float pitch_vel;
  float roll_vel;
  float yaw_vel;

  void parse(const std::vector<uint8_t> & in);
};

class BrakeMotorRpt1Msg : public MotorRpt1Msg
{
public:
  static constexpr uint32_t CAN_ID = 0x401;
};

class BrakeMotorRpt2Msg : public MotorRpt2Msg
{
public:
  static constexpr uint32_t CAN_ID = 0x402;
};

class BrakeMotorRpt3Msg : public MotorRpt3Msg
{
public:
  static constexpr uint32_t CAN_ID = 0x403;
};

class DateTimeRptMsg : public Pacmod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x40F;

  uint32_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;

  void parse(const std::vector<uint8_t> & in);
};

class DetectedObjectRptMsg : public Pacmod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x411;

  double front_object_distance_low_res;
  double front_object_distance_high_res;

  void parse(const std::vector<uint8_t> & in);
};

class DoorRptMsg : public Pacmod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x417;

  bool driver_door_open;
  bool passenger_door_open;
  bool rear_driver_door_open;
  bool rear_passenger_door_open;
  bool hood_open;
  bool trunk_open;
  bool fuel_door_open;
  bool driver_door_open_avail;
  bool passenger_door_open_avail;
  bool rear_driver_door_open_avail;
  bool rear_passenger_door_open_avail;
  bool hood_open_avail;
  bool trunk_open_avail;
  bool fuel_door_open_avail;

  void parse(const std::vector<uint8_t> & in);
};

class DriveTrainRptMsg : public Pacmod3TxMsg
{
  public:
    static constexpr uint32_t CAN_ID = 0x41F;

    bool antilock_brake_active;
    bool traction_control_active;
    bool four_wheel_drive_active;
    bool antilock_brake_active_avail;
    bool traction_control_active_avail;
    bool four_wheel_drive_active_avail;
    DriveMode drive_mode;
    bool drive_mode_avail;

  void parse(const std::vector<uint8_t> & in);
};

class EngineRptMsg : public Pacmod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x410;

  float engine_speed;
  float engine_torque;
  int16_t engine_coolant_temp;
  bool engine_speed_avail;
  bool engine_torque_avail;
  bool engine_coolant_temp_avail;
  bool fuel_level_avail;
  float fuel_level;

  void parse(const std::vector<uint8_t> & in);
};

class InteriorLightsRptMsg : public Pacmod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x416;

  bool front_dome_lights_on;
  bool rear_dome_lights_on;
  bool mood_lights_on;
  bool ambient_light_sensor;
  DimLevel dim_level;

  bool front_dome_lights_on_avail;
  bool rear_dome_lights_on_avail;
  bool mood_lights_on_avail;
  bool dim_level_avail;
  bool ambient_light_sensor_avail;

  void parse(const std::vector<uint8_t> & in);
};

class LatLonHeadingRptMsg : public Pacmod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x40E;

  int latitude_degrees;
  uint32_t latitude_minutes;
  uint32_t latitude_seconds;
  int longitude_degrees;
  uint32_t longitude_minutes;
  uint32_t longitude_seconds;
  double heading;

  void parse(const std::vector<uint8_t> & in);
};

class LinearAccelRptMsg : public Pacmod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x419;

  bool lateral_new_data_rx;
  bool longitudinal_new_data_rx;
  bool vertical_new_data_rx;
  bool lateral_valid;
  bool longitudinal_valid;
  bool vertical_valid;
  double lateral_accel;
  double longitudinal_accel;
  double vertical_accel;

  void parse(const std::vector<uint8_t> & in);
};

class OccupancyRptMsg : public Pacmod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x415;

  bool driver_seat_occupied;
  bool passenger_seat_occupied;
  bool rear_seat_occupied;
  bool driver_seatbelt_buckled;
  bool passenger_seatbelt_buckled;
  bool driver_rear_seatbelt_buckled;
  bool pass_rear_seatbelt_buckled;
  bool center_rear_seatbelt_buckled;
  bool driver_seat_occupied_avail;
  bool passenger_seat_occupied_avail;
  bool rear_seat_occupied_avail;
  bool driver_seatbelt_buckled_avail;
  bool passenger_seatbelt_buckled_avail;
  bool driver_rear_seatbelt_buckled_avail;
  bool pass_rear_seatbelt_buckled_avail;
  bool center_rear_seatbelt_buckled_avail;

  void parse(const std::vector<uint8_t> & in);
};

class RearLightsRptMsg : public Pacmod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x418;

  bool brake_lights_on;
  bool brake_lights_on_avail;
  bool reverse_lights_on;
  bool reverse_lights_on_avail;

  void parse(const std::vector<uint8_t> & in);
};

class SteerMotorRpt1Msg : public MotorRpt1Msg
{
public:
  static constexpr uint32_t CAN_ID = 0x404;
};

class SteerMotorRpt2Msg : public MotorRpt2Msg
{
public:
  static constexpr uint32_t CAN_ID = 0x405;
};

class SteerMotorRpt3Msg : public MotorRpt3Msg
{
public:
  static constexpr uint32_t CAN_ID = 0x406;
};

class TirePressureRptMsg : public Pacmod3TxMsg
{
  public:
    static constexpr uint32_t CAN_ID = 0x41E;

    uint8_t front_left_tire_pressure;
    uint8_t front_right_tire_pressure;
    uint8_t rear_left_tire_pressure;
    uint8_t rear_right_tire_pressure;

  void parse(const std::vector<uint8_t> & in);
};

class VehDynamicsRptMsg : public Pacmod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x413;

  double veh_g_forces;

  void parse(const std::vector<uint8_t> & in);
};

class VehicleFaultRptMsg : public Pacmod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x42;

  bool engine_check_light;
  bool engine_check_light_avail;
  bool trc_fault_light;
  bool trc_fault_light_avail;
  bool trc_off_fault_light;
  bool trc_off_fault_light_avail;
  bool antilock_brake_fault_light;
  bool antilock_brake_fault_light_avail;
  bool tire_fault_light;
  bool tire_fault_light_avail;
  bool air_bags_fault_light;
  bool air_bags_fault_light_avail;
  bool low_engine_oil_pressure;
  bool low_engine_oil_pressure_avail;
  bool brake_fault;
  bool brake_fault_avail;
  bool brake_applied_power_reduced;
  bool brake_applied_power_reduced_avail;
  bool steering_loss_stop_safely;
  bool steering_loss_stop_safely_avail;
  bool steering_fault_service_now;
  bool steering_fault_service_now_avail;
  bool xmsn_fault_service_now;
  bool xmsn_fault_service_now_avail;
  bool xmsn_over_temp_stop_safely;
  bool xmsn_over_temp_stop_safely_avail;
  bool low_battery_features_off;
  bool low_battery_features_off_avail;
  bool charging_system_fault;
  bool charging_system_fault_avail;

  void parse(const std::vector<uint8_t> & in);
};

class VehicleSpeedRptMsg : public Pacmod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x400;

  double vehicle_speed;
  bool vehicle_speed_valid;

  void parse(const std::vector<uint8_t> & in);
};

class VinRptMsg : public Pacmod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x414;

  std::string mfg_code;
  std::string mfg;
  char model_year_code;
  uint32_t model_year;
  uint32_t serial;

  void parse(const std::vector<uint8_t> & in);
};

class VinRpt2Msg : public Pacmod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x422;

  char vis_10;
  char vis_11;
  char vis_12;
  char vis_13;
  char vis_14;
  char vis_15;
  char vis_16;
  char vis_17;

  void parse(const std::vector<uint8_t> & in);
};

class WheelSpeedRptMsg : public Pacmod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x407;

  double front_left_wheel_speed;
  double front_right_wheel_speed;
  double rear_left_wheel_speed;
  double rear_right_wheel_speed;

  void parse(const std::vector<uint8_t> & in);
};

class YawRateRptMsg : public Pacmod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x40D;

  double yaw_rate;

  void parse(const std::vector<uint8_t> & in);
};

class AccelCmdLimitRptMsg : public SystemCmdLimitRptMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x201;
};

class BrakeCmdLimitRptMsg : public SystemCmdLimitRptMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x205;
};

class SteerCmdLimitRptMsg : public  Pacmod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x22D;

  double pos_cmd_limit;
  double limited_pos_cmd;
  double rotation_rate_cmd_limit;
  double limited_rotation_rate_cmd;
  
  void parse(const std::vector<uint8_t> & in);
};

}  // namespace pacmod3

#endif  // PACMOD3__PACMOD3_CORE_HPP_
