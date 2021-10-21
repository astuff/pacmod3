#include "pacmod3/autogen/pacmod12.h"


#ifdef PACMOD12_USE_DIAG_MONITORS
// Function prototypes to be called each time CAN frame is unpacked
// FMon function may detect RC, CRC or DLC violation
#include "pacmod12-fmon.h"

#endif // PACMOD12_USE_DIAG_MONITORS


uint32_t Unpack_GLOBAL_RPT_pacmod12(GLOBAL_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->PACMOD_SYSTEM_ENABLED = (_d[0] & (0x01U));
  _m->PACMOD_SYSTEM_OVERRIDE_ACTIVE = ((_d[0] >> 1) & (0x01U));
  _m->USR_CAN_TIMEOUT = ((_d[0] >> 2) & (0x01U));
  _m->STR_CAN_TIMEOUT = ((_d[0] >> 3) & (0x01U));
  _m->BRK_CAN_TIMEOUT = ((_d[0] >> 4) & (0x01U));
  _m->PACMOD_SUBSYSTEM_TIMEOUT = ((_d[0] >> 5) & (0x01U));
  _m->VEH_CAN_TIMEOUT = ((_d[0] >> 6) & (0x01U));
  _m->PACMOD_SYSTEM_FAULT_ACTIVE = ((_d[0] >> 7) & (0x01U));
  _m->SUPERVISORY_ENABLE_REQUIRED = ((_d[1] >> 6) & (0x01U));
  _m->CONFIG_FAULT_ACTIVE = ((_d[1] >> 7) & (0x01U));
  _m->USR_CAN_READ_ERRORS = ((_d[6] & (0xFFU)) << 8) | (_d[7] & (0xFFU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < GLOBAL_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_GLOBAL_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return GLOBAL_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_GLOBAL_RPT_pacmod12(GLOBAL_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < GLOBAL_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->PACMOD_SYSTEM_ENABLED & (0x01U)) | ((_m->PACMOD_SYSTEM_OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->USR_CAN_TIMEOUT & (0x01U)) << 2) | ((_m->STR_CAN_TIMEOUT & (0x01U)) << 3) | ((_m->BRK_CAN_TIMEOUT & (0x01U)) << 4) | ((_m->PACMOD_SUBSYSTEM_TIMEOUT & (0x01U)) << 5) | ((_m->VEH_CAN_TIMEOUT & (0x01U)) << 6) | ((_m->PACMOD_SYSTEM_FAULT_ACTIVE & (0x01U)) << 7);
  cframe->Data[1] |= ((_m->SUPERVISORY_ENABLE_REQUIRED & (0x01U)) << 6) | ((_m->CONFIG_FAULT_ACTIVE & (0x01U)) << 7);
  cframe->Data[6] |= ((_m->USR_CAN_READ_ERRORS >> 8) & (0xFFU));
  cframe->Data[7] |= (_m->USR_CAN_READ_ERRORS & (0xFFU));

  cframe->MsgId = GLOBAL_RPT_CANID;
  cframe->DLC = GLOBAL_RPT_DLC;
  cframe->IDE = GLOBAL_RPT_IDE;
  return GLOBAL_RPT_CANID;
}

#else

uint32_t Pack_GLOBAL_RPT_pacmod12(GLOBAL_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < GLOBAL_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->PACMOD_SYSTEM_ENABLED & (0x01U)) | ((_m->PACMOD_SYSTEM_OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->USR_CAN_TIMEOUT & (0x01U)) << 2) | ((_m->STR_CAN_TIMEOUT & (0x01U)) << 3) | ((_m->BRK_CAN_TIMEOUT & (0x01U)) << 4) | ((_m->PACMOD_SUBSYSTEM_TIMEOUT & (0x01U)) << 5) | ((_m->VEH_CAN_TIMEOUT & (0x01U)) << 6) | ((_m->PACMOD_SYSTEM_FAULT_ACTIVE & (0x01U)) << 7);
  _d[1] |= ((_m->SUPERVISORY_ENABLE_REQUIRED & (0x01U)) << 6) | ((_m->CONFIG_FAULT_ACTIVE & (0x01U)) << 7);
  _d[6] |= ((_m->USR_CAN_READ_ERRORS >> 8) & (0xFFU));
  _d[7] |= (_m->USR_CAN_READ_ERRORS & (0xFFU));

  *_len = GLOBAL_RPT_DLC;
  *_ide = GLOBAL_RPT_IDE;
  return GLOBAL_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_GLOBAL_RPT_2_pacmod12(GLOBAL_RPT_2_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->SYSTEM_ENABLED = (_d[0] & (0x01U));
  _m->SYSTEM_OVERRIDE_ACTIVE = ((_d[0] >> 1) & (0x01U));
  _m->SYSTEM_FAULT_ACTIVE = ((_d[0] >> 2) & (0x01U));
  _m->SUPERVISORY_ENABLE_REQUIRED = ((_d[0] >> 3) & (0x01U));
  _m->DISABLE_ALL_SYSTEMS = ((_d[0] >> 4) & (0x01U));
  _m->SYSTEM_READY = ((_d[0] >> 5) & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < GLOBAL_RPT_2_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_GLOBAL_RPT_2_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return GLOBAL_RPT_2_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_GLOBAL_RPT_2_pacmod12(GLOBAL_RPT_2_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < GLOBAL_RPT_2_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->SYSTEM_ENABLED & (0x01U)) | ((_m->SYSTEM_OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->SYSTEM_FAULT_ACTIVE & (0x01U)) << 2) | ((_m->SUPERVISORY_ENABLE_REQUIRED & (0x01U)) << 3) | ((_m->DISABLE_ALL_SYSTEMS & (0x01U)) << 4) | ((_m->SYSTEM_READY & (0x01U)) << 5);

  cframe->MsgId = GLOBAL_RPT_2_CANID;
  cframe->DLC = GLOBAL_RPT_2_DLC;
  cframe->IDE = GLOBAL_RPT_2_IDE;
  return GLOBAL_RPT_2_CANID;
}

#else

uint32_t Pack_GLOBAL_RPT_2_pacmod12(GLOBAL_RPT_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < GLOBAL_RPT_2_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->SYSTEM_ENABLED & (0x01U)) | ((_m->SYSTEM_OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->SYSTEM_FAULT_ACTIVE & (0x01U)) << 2) | ((_m->SUPERVISORY_ENABLE_REQUIRED & (0x01U)) << 3) | ((_m->DISABLE_ALL_SYSTEMS & (0x01U)) << 4) | ((_m->SYSTEM_READY & (0x01U)) << 5);

  *_len = GLOBAL_RPT_2_DLC;
  *_ide = GLOBAL_RPT_2_IDE;
  return GLOBAL_RPT_2_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_COMPONENT_RPT_00_pacmod12(COMPONENT_RPT_00_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->COMPONENT_TYPE = (_d[0] & (0x0FU));
  _m->ACCEL = ((_d[0] >> 4) & (0x01U));
  _m->BRAKE = ((_d[0] >> 5) & (0x01U));
  _m->CRUISE_CONTROL_BUTTONS = ((_d[0] >> 6) & (0x01U));
  _m->DASH_CONTROLS_LEFT = ((_d[0] >> 7) & (0x01U));
  _m->DASH_CONTROLS_RIGHT = (_d[1] & (0x01U));
  _m->HAZARD_LIGHTS = ((_d[1] >> 1) & (0x01U));
  _m->HEADLIGHT = ((_d[1] >> 2) & (0x01U));
  _m->HORN = ((_d[1] >> 3) & (0x01U));
  _m->MEDIA_CONTROLS = ((_d[1] >> 4) & (0x01U));
  _m->PARKING_BRAKE = ((_d[1] >> 5) & (0x01U));
  _m->SHIFT = ((_d[1] >> 6) & (0x01U));
  _m->SPRAYER = ((_d[1] >> 7) & (0x01U));
  _m->STEERING = (_d[2] & (0x01U));
  _m->TURN = ((_d[2] >> 1) & (0x01U));
  _m->WIPER = ((_d[2] >> 2) & (0x01U));
  _m->WATCHDOG = ((_d[2] >> 3) & (0x01U));
  _m->BRAKE_DECEL = ((_d[2] >> 4) & (0x01U));
  _m->REAR_PASS_DOOR = ((_d[2] >> 5) & (0x01U));
  _m->ENGINE_BRAKE = ((_d[2] >> 6) & (0x01U));
  _m->MARKER_LAMP = ((_d[2] >> 7) & (0x01U));
  _m->CABIN_CLIMATE = (_d[3] & (0x01U));
  _m->CABIN_FAN_SPEED = ((_d[3] >> 1) & (0x01U));
  _m->CABIN_TEMP = ((_d[3] >> 2) & (0x01U));
  _m->EXHAUST_BRAKE = ((_d[3] >> 3) & (0x01U));
  _m->COUNTER = (_d[4] & (0x0FU));
  _m->COMPLEMENT = ((_d[4] >> 4) & (0x0FU));
  _m->CONFIG_FAULT = (_d[5] & (0x01U));
  _m->CAN_TIMEOUT_FAULT = ((_d[5] >> 1) & (0x01U));
  _m->INTERNAL_SUPPLY_VOLTAGE_FAULT = ((_d[5] >> 2) & (0x01U));
  _m->SUPERVISORY_TIMEOUT = ((_d[5] >> 3) & (0x01U));
  _m->SUPERVISORY_SANITY_FAULT = ((_d[5] >> 4) & (0x01U));
  _m->WATCHDOG_SANITY_FAULT = ((_d[5] >> 5) & (0x01U));
  _m->WATCHDOG_SYSTEM_PRESENT_FAULT = ((_d[5] >> 6) & (0x01U));
  _m->COMPONENT_READY = ((_d[5] >> 7) & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < COMPONENT_RPT_00_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_COMPONENT_RPT_00_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return COMPONENT_RPT_00_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_COMPONENT_RPT_00_pacmod12(COMPONENT_RPT_00_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < COMPONENT_RPT_00_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->COMPONENT_TYPE & (0x0FU)) | ((_m->ACCEL & (0x01U)) << 4) | ((_m->BRAKE & (0x01U)) << 5) | ((_m->CRUISE_CONTROL_BUTTONS & (0x01U)) << 6) | ((_m->DASH_CONTROLS_LEFT & (0x01U)) << 7);
  cframe->Data[1] |= (_m->DASH_CONTROLS_RIGHT & (0x01U)) | ((_m->HAZARD_LIGHTS & (0x01U)) << 1) | ((_m->HEADLIGHT & (0x01U)) << 2) | ((_m->HORN & (0x01U)) << 3) | ((_m->MEDIA_CONTROLS & (0x01U)) << 4) | ((_m->PARKING_BRAKE & (0x01U)) << 5) | ((_m->SHIFT & (0x01U)) << 6) | ((_m->SPRAYER & (0x01U)) << 7);
  cframe->Data[2] |= (_m->STEERING & (0x01U)) | ((_m->TURN & (0x01U)) << 1) | ((_m->WIPER & (0x01U)) << 2) | ((_m->WATCHDOG & (0x01U)) << 3) | ((_m->BRAKE_DECEL & (0x01U)) << 4) | ((_m->REAR_PASS_DOOR & (0x01U)) << 5) | ((_m->ENGINE_BRAKE & (0x01U)) << 6) | ((_m->MARKER_LAMP & (0x01U)) << 7);
  cframe->Data[3] |= (_m->CABIN_CLIMATE & (0x01U)) | ((_m->CABIN_FAN_SPEED & (0x01U)) << 1) | ((_m->CABIN_TEMP & (0x01U)) << 2) | ((_m->EXHAUST_BRAKE & (0x01U)) << 3);
  cframe->Data[4] |= (_m->COUNTER & (0x0FU)) | ((_m->COMPLEMENT & (0x0FU)) << 4);
  cframe->Data[5] |= (_m->CONFIG_FAULT & (0x01U)) | ((_m->CAN_TIMEOUT_FAULT & (0x01U)) << 1) | ((_m->INTERNAL_SUPPLY_VOLTAGE_FAULT & (0x01U)) << 2) | ((_m->SUPERVISORY_TIMEOUT & (0x01U)) << 3) | ((_m->SUPERVISORY_SANITY_FAULT & (0x01U)) << 4) | ((_m->WATCHDOG_SANITY_FAULT & (0x01U)) << 5) | ((_m->WATCHDOG_SYSTEM_PRESENT_FAULT & (0x01U)) << 6) | ((_m->COMPONENT_READY & (0x01U)) << 7);

  cframe->MsgId = COMPONENT_RPT_00_CANID;
  cframe->DLC = COMPONENT_RPT_00_DLC;
  cframe->IDE = COMPONENT_RPT_00_IDE;
  return COMPONENT_RPT_00_CANID;
}

#else

uint32_t Pack_COMPONENT_RPT_00_pacmod12(COMPONENT_RPT_00_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < COMPONENT_RPT_00_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->COMPONENT_TYPE & (0x0FU)) | ((_m->ACCEL & (0x01U)) << 4) | ((_m->BRAKE & (0x01U)) << 5) | ((_m->CRUISE_CONTROL_BUTTONS & (0x01U)) << 6) | ((_m->DASH_CONTROLS_LEFT & (0x01U)) << 7);
  _d[1] |= (_m->DASH_CONTROLS_RIGHT & (0x01U)) | ((_m->HAZARD_LIGHTS & (0x01U)) << 1) | ((_m->HEADLIGHT & (0x01U)) << 2) | ((_m->HORN & (0x01U)) << 3) | ((_m->MEDIA_CONTROLS & (0x01U)) << 4) | ((_m->PARKING_BRAKE & (0x01U)) << 5) | ((_m->SHIFT & (0x01U)) << 6) | ((_m->SPRAYER & (0x01U)) << 7);
  _d[2] |= (_m->STEERING & (0x01U)) | ((_m->TURN & (0x01U)) << 1) | ((_m->WIPER & (0x01U)) << 2) | ((_m->WATCHDOG & (0x01U)) << 3) | ((_m->BRAKE_DECEL & (0x01U)) << 4) | ((_m->REAR_PASS_DOOR & (0x01U)) << 5) | ((_m->ENGINE_BRAKE & (0x01U)) << 6) | ((_m->MARKER_LAMP & (0x01U)) << 7);
  _d[3] |= (_m->CABIN_CLIMATE & (0x01U)) | ((_m->CABIN_FAN_SPEED & (0x01U)) << 1) | ((_m->CABIN_TEMP & (0x01U)) << 2) | ((_m->EXHAUST_BRAKE & (0x01U)) << 3);
  _d[4] |= (_m->COUNTER & (0x0FU)) | ((_m->COMPLEMENT & (0x0FU)) << 4);
  _d[5] |= (_m->CONFIG_FAULT & (0x01U)) | ((_m->CAN_TIMEOUT_FAULT & (0x01U)) << 1) | ((_m->INTERNAL_SUPPLY_VOLTAGE_FAULT & (0x01U)) << 2) | ((_m->SUPERVISORY_TIMEOUT & (0x01U)) << 3) | ((_m->SUPERVISORY_SANITY_FAULT & (0x01U)) << 4) | ((_m->WATCHDOG_SANITY_FAULT & (0x01U)) << 5) | ((_m->WATCHDOG_SYSTEM_PRESENT_FAULT & (0x01U)) << 6) | ((_m->COMPONENT_READY & (0x01U)) << 7);

  *_len = COMPONENT_RPT_00_DLC;
  *_ide = COMPONENT_RPT_00_IDE;
  return COMPONENT_RPT_00_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_COMPONENT_RPT_01_pacmod12(COMPONENT_RPT_01_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->COMPONENT_TYPE = (_d[0] & (0x0FU));
  _m->ACCEL = ((_d[0] >> 4) & (0x01U));
  _m->BRAKE = ((_d[0] >> 5) & (0x01U));
  _m->CRUISE_CONTROL_BUTTONS = ((_d[0] >> 6) & (0x01U));
  _m->DASH_CONTROLS_LEFT = ((_d[0] >> 7) & (0x01U));
  _m->DASH_CONTROLS_RIGHT = (_d[1] & (0x01U));
  _m->HAZARD_LIGHTS = ((_d[1] >> 1) & (0x01U));
  _m->HEADLIGHT = ((_d[1] >> 2) & (0x01U));
  _m->HORN = ((_d[1] >> 3) & (0x01U));
  _m->MEDIA_CONTROLS = ((_d[1] >> 4) & (0x01U));
  _m->PARKING_BRAKE = ((_d[1] >> 5) & (0x01U));
  _m->SHIFT = ((_d[1] >> 6) & (0x01U));
  _m->SPRAYER = ((_d[1] >> 7) & (0x01U));
  _m->STEERING = (_d[2] & (0x01U));
  _m->TURN = ((_d[2] >> 1) & (0x01U));
  _m->WIPER = ((_d[2] >> 2) & (0x01U));
  _m->WATCHDOG = ((_d[2] >> 3) & (0x01U));
  _m->BRAKE_DECEL = ((_d[2] >> 4) & (0x01U));
  _m->REAR_PASS_DOOR = ((_d[2] >> 5) & (0x01U));
  _m->ENGINE_BRAKE = ((_d[2] >> 6) & (0x01U));
  _m->MARKER_LAMP = ((_d[2] >> 7) & (0x01U));
  _m->CABIN_CLIMATE = (_d[3] & (0x01U));
  _m->CABIN_FAN_SPEED = ((_d[3] >> 1) & (0x01U));
  _m->CABIN_TEMP = ((_d[3] >> 2) & (0x01U));
  _m->EXHAUST_BRAKE = ((_d[3] >> 3) & (0x01U));
  _m->COUNTER = (_d[4] & (0x0FU));
  _m->COMPLEMENT = ((_d[4] >> 4) & (0x0FU));
  _m->CONFIG_FAULT = (_d[5] & (0x01U));
  _m->CAN_TIMEOUT_FAULT = ((_d[5] >> 1) & (0x01U));
  _m->INTERNAL_SUPPLY_VOLTAGE_FAULT = ((_d[5] >> 2) & (0x01U));
  _m->SUPERVISORY_TIMEOUT = ((_d[5] >> 3) & (0x01U));
  _m->SUPERVISORY_SANITY_FAULT = ((_d[5] >> 4) & (0x01U));
  _m->WATCHDOG_SANITY_FAULT = ((_d[5] >> 5) & (0x01U));
  _m->WATCHDOG_SYSTEM_PRESENT_FAULT = ((_d[5] >> 6) & (0x01U));
  _m->COMPONENT_READY = ((_d[5] >> 7) & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < COMPONENT_RPT_01_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_COMPONENT_RPT_01_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return COMPONENT_RPT_01_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_COMPONENT_RPT_01_pacmod12(COMPONENT_RPT_01_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < COMPONENT_RPT_01_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->COMPONENT_TYPE & (0x0FU)) | ((_m->ACCEL & (0x01U)) << 4) | ((_m->BRAKE & (0x01U)) << 5) | ((_m->CRUISE_CONTROL_BUTTONS & (0x01U)) << 6) | ((_m->DASH_CONTROLS_LEFT & (0x01U)) << 7);
  cframe->Data[1] |= (_m->DASH_CONTROLS_RIGHT & (0x01U)) | ((_m->HAZARD_LIGHTS & (0x01U)) << 1) | ((_m->HEADLIGHT & (0x01U)) << 2) | ((_m->HORN & (0x01U)) << 3) | ((_m->MEDIA_CONTROLS & (0x01U)) << 4) | ((_m->PARKING_BRAKE & (0x01U)) << 5) | ((_m->SHIFT & (0x01U)) << 6) | ((_m->SPRAYER & (0x01U)) << 7);
  cframe->Data[2] |= (_m->STEERING & (0x01U)) | ((_m->TURN & (0x01U)) << 1) | ((_m->WIPER & (0x01U)) << 2) | ((_m->WATCHDOG & (0x01U)) << 3) | ((_m->BRAKE_DECEL & (0x01U)) << 4) | ((_m->REAR_PASS_DOOR & (0x01U)) << 5) | ((_m->ENGINE_BRAKE & (0x01U)) << 6) | ((_m->MARKER_LAMP & (0x01U)) << 7);
  cframe->Data[3] |= (_m->CABIN_CLIMATE & (0x01U)) | ((_m->CABIN_FAN_SPEED & (0x01U)) << 1) | ((_m->CABIN_TEMP & (0x01U)) << 2) | ((_m->EXHAUST_BRAKE & (0x01U)) << 3);
  cframe->Data[4] |= (_m->COUNTER & (0x0FU)) | ((_m->COMPLEMENT & (0x0FU)) << 4);
  cframe->Data[5] |= (_m->CONFIG_FAULT & (0x01U)) | ((_m->CAN_TIMEOUT_FAULT & (0x01U)) << 1) | ((_m->INTERNAL_SUPPLY_VOLTAGE_FAULT & (0x01U)) << 2) | ((_m->SUPERVISORY_TIMEOUT & (0x01U)) << 3) | ((_m->SUPERVISORY_SANITY_FAULT & (0x01U)) << 4) | ((_m->WATCHDOG_SANITY_FAULT & (0x01U)) << 5) | ((_m->WATCHDOG_SYSTEM_PRESENT_FAULT & (0x01U)) << 6) | ((_m->COMPONENT_READY & (0x01U)) << 7);

  cframe->MsgId = COMPONENT_RPT_01_CANID;
  cframe->DLC = COMPONENT_RPT_01_DLC;
  cframe->IDE = COMPONENT_RPT_01_IDE;
  return COMPONENT_RPT_01_CANID;
}

#else

uint32_t Pack_COMPONENT_RPT_01_pacmod12(COMPONENT_RPT_01_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < COMPONENT_RPT_01_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->COMPONENT_TYPE & (0x0FU)) | ((_m->ACCEL & (0x01U)) << 4) | ((_m->BRAKE & (0x01U)) << 5) | ((_m->CRUISE_CONTROL_BUTTONS & (0x01U)) << 6) | ((_m->DASH_CONTROLS_LEFT & (0x01U)) << 7);
  _d[1] |= (_m->DASH_CONTROLS_RIGHT & (0x01U)) | ((_m->HAZARD_LIGHTS & (0x01U)) << 1) | ((_m->HEADLIGHT & (0x01U)) << 2) | ((_m->HORN & (0x01U)) << 3) | ((_m->MEDIA_CONTROLS & (0x01U)) << 4) | ((_m->PARKING_BRAKE & (0x01U)) << 5) | ((_m->SHIFT & (0x01U)) << 6) | ((_m->SPRAYER & (0x01U)) << 7);
  _d[2] |= (_m->STEERING & (0x01U)) | ((_m->TURN & (0x01U)) << 1) | ((_m->WIPER & (0x01U)) << 2) | ((_m->WATCHDOG & (0x01U)) << 3) | ((_m->BRAKE_DECEL & (0x01U)) << 4) | ((_m->REAR_PASS_DOOR & (0x01U)) << 5) | ((_m->ENGINE_BRAKE & (0x01U)) << 6) | ((_m->MARKER_LAMP & (0x01U)) << 7);
  _d[3] |= (_m->CABIN_CLIMATE & (0x01U)) | ((_m->CABIN_FAN_SPEED & (0x01U)) << 1) | ((_m->CABIN_TEMP & (0x01U)) << 2) | ((_m->EXHAUST_BRAKE & (0x01U)) << 3);
  _d[4] |= (_m->COUNTER & (0x0FU)) | ((_m->COMPLEMENT & (0x0FU)) << 4);
  _d[5] |= (_m->CONFIG_FAULT & (0x01U)) | ((_m->CAN_TIMEOUT_FAULT & (0x01U)) << 1) | ((_m->INTERNAL_SUPPLY_VOLTAGE_FAULT & (0x01U)) << 2) | ((_m->SUPERVISORY_TIMEOUT & (0x01U)) << 3) | ((_m->SUPERVISORY_SANITY_FAULT & (0x01U)) << 4) | ((_m->WATCHDOG_SANITY_FAULT & (0x01U)) << 5) | ((_m->WATCHDOG_SYSTEM_PRESENT_FAULT & (0x01U)) << 6) | ((_m->COMPONENT_READY & (0x01U)) << 7);

  *_len = COMPONENT_RPT_01_DLC;
  *_ide = COMPONENT_RPT_01_IDE;
  return COMPONENT_RPT_01_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_COMPONENT_RPT_02_pacmod12(COMPONENT_RPT_02_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->COMPONENT_TYPE = (_d[0] & (0x0FU));
  _m->ACCEL = ((_d[0] >> 4) & (0x01U));
  _m->BRAKE = ((_d[0] >> 5) & (0x01U));
  _m->CRUISE_CONTROL_BUTTONS = ((_d[0] >> 6) & (0x01U));
  _m->DASH_CONTROLS_LEFT = ((_d[0] >> 7) & (0x01U));
  _m->DASH_CONTROLS_RIGHT = (_d[1] & (0x01U));
  _m->HAZARD_LIGHTS = ((_d[1] >> 1) & (0x01U));
  _m->HEADLIGHT = ((_d[1] >> 2) & (0x01U));
  _m->HORN = ((_d[1] >> 3) & (0x01U));
  _m->MEDIA_CONTROLS = ((_d[1] >> 4) & (0x01U));
  _m->PARKING_BRAKE = ((_d[1] >> 5) & (0x01U));
  _m->SHIFT = ((_d[1] >> 6) & (0x01U));
  _m->SPRAYER = ((_d[1] >> 7) & (0x01U));
  _m->STEERING = (_d[2] & (0x01U));
  _m->TURN = ((_d[2] >> 1) & (0x01U));
  _m->WIPER = ((_d[2] >> 2) & (0x01U));
  _m->WATCHDOG = ((_d[2] >> 3) & (0x01U));
  _m->BRAKE_DECEL = ((_d[2] >> 4) & (0x01U));
  _m->REAR_PASS_DOOR = ((_d[2] >> 5) & (0x01U));
  _m->ENGINE_BRAKE = ((_d[2] >> 6) & (0x01U));
  _m->MARKER_LAMP = ((_d[2] >> 7) & (0x01U));
  _m->CABIN_CLIMATE = (_d[3] & (0x01U));
  _m->CABIN_FAN_SPEED = ((_d[3] >> 1) & (0x01U));
  _m->CABIN_TEMP = ((_d[3] >> 2) & (0x01U));
  _m->EXHAUST_BRAKE = ((_d[3] >> 3) & (0x01U));
  _m->COUNTER = (_d[4] & (0x0FU));
  _m->COMPLEMENT = ((_d[4] >> 4) & (0x0FU));
  _m->CONFIG_FAULT = (_d[5] & (0x01U));
  _m->CAN_TIMEOUT_FAULT = ((_d[5] >> 1) & (0x01U));
  _m->INTERNAL_SUPPLY_VOLTAGE_FAULT = ((_d[5] >> 2) & (0x01U));
  _m->SUPERVISORY_TIMEOUT = ((_d[5] >> 3) & (0x01U));
  _m->SUPERVISORY_SANITY_FAULT = ((_d[5] >> 4) & (0x01U));
  _m->WATCHDOG_SANITY_FAULT = ((_d[5] >> 5) & (0x01U));
  _m->WATCHDOG_SYSTEM_PRESENT_FAULT = ((_d[5] >> 6) & (0x01U));
  _m->COMPONENT_READY = ((_d[5] >> 7) & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < COMPONENT_RPT_02_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_COMPONENT_RPT_02_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return COMPONENT_RPT_02_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_COMPONENT_RPT_02_pacmod12(COMPONENT_RPT_02_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < COMPONENT_RPT_02_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->COMPONENT_TYPE & (0x0FU)) | ((_m->ACCEL & (0x01U)) << 4) | ((_m->BRAKE & (0x01U)) << 5) | ((_m->CRUISE_CONTROL_BUTTONS & (0x01U)) << 6) | ((_m->DASH_CONTROLS_LEFT & (0x01U)) << 7);
  cframe->Data[1] |= (_m->DASH_CONTROLS_RIGHT & (0x01U)) | ((_m->HAZARD_LIGHTS & (0x01U)) << 1) | ((_m->HEADLIGHT & (0x01U)) << 2) | ((_m->HORN & (0x01U)) << 3) | ((_m->MEDIA_CONTROLS & (0x01U)) << 4) | ((_m->PARKING_BRAKE & (0x01U)) << 5) | ((_m->SHIFT & (0x01U)) << 6) | ((_m->SPRAYER & (0x01U)) << 7);
  cframe->Data[2] |= (_m->STEERING & (0x01U)) | ((_m->TURN & (0x01U)) << 1) | ((_m->WIPER & (0x01U)) << 2) | ((_m->WATCHDOG & (0x01U)) << 3) | ((_m->BRAKE_DECEL & (0x01U)) << 4) | ((_m->REAR_PASS_DOOR & (0x01U)) << 5) | ((_m->ENGINE_BRAKE & (0x01U)) << 6) | ((_m->MARKER_LAMP & (0x01U)) << 7);
  cframe->Data[3] |= (_m->CABIN_CLIMATE & (0x01U)) | ((_m->CABIN_FAN_SPEED & (0x01U)) << 1) | ((_m->CABIN_TEMP & (0x01U)) << 2) | ((_m->EXHAUST_BRAKE & (0x01U)) << 3);
  cframe->Data[4] |= (_m->COUNTER & (0x0FU)) | ((_m->COMPLEMENT & (0x0FU)) << 4);
  cframe->Data[5] |= (_m->CONFIG_FAULT & (0x01U)) | ((_m->CAN_TIMEOUT_FAULT & (0x01U)) << 1) | ((_m->INTERNAL_SUPPLY_VOLTAGE_FAULT & (0x01U)) << 2) | ((_m->SUPERVISORY_TIMEOUT & (0x01U)) << 3) | ((_m->SUPERVISORY_SANITY_FAULT & (0x01U)) << 4) | ((_m->WATCHDOG_SANITY_FAULT & (0x01U)) << 5) | ((_m->WATCHDOG_SYSTEM_PRESENT_FAULT & (0x01U)) << 6) | ((_m->COMPONENT_READY & (0x01U)) << 7);

  cframe->MsgId = COMPONENT_RPT_02_CANID;
  cframe->DLC = COMPONENT_RPT_02_DLC;
  cframe->IDE = COMPONENT_RPT_02_IDE;
  return COMPONENT_RPT_02_CANID;
}

#else

uint32_t Pack_COMPONENT_RPT_02_pacmod12(COMPONENT_RPT_02_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < COMPONENT_RPT_02_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->COMPONENT_TYPE & (0x0FU)) | ((_m->ACCEL & (0x01U)) << 4) | ((_m->BRAKE & (0x01U)) << 5) | ((_m->CRUISE_CONTROL_BUTTONS & (0x01U)) << 6) | ((_m->DASH_CONTROLS_LEFT & (0x01U)) << 7);
  _d[1] |= (_m->DASH_CONTROLS_RIGHT & (0x01U)) | ((_m->HAZARD_LIGHTS & (0x01U)) << 1) | ((_m->HEADLIGHT & (0x01U)) << 2) | ((_m->HORN & (0x01U)) << 3) | ((_m->MEDIA_CONTROLS & (0x01U)) << 4) | ((_m->PARKING_BRAKE & (0x01U)) << 5) | ((_m->SHIFT & (0x01U)) << 6) | ((_m->SPRAYER & (0x01U)) << 7);
  _d[2] |= (_m->STEERING & (0x01U)) | ((_m->TURN & (0x01U)) << 1) | ((_m->WIPER & (0x01U)) << 2) | ((_m->WATCHDOG & (0x01U)) << 3) | ((_m->BRAKE_DECEL & (0x01U)) << 4) | ((_m->REAR_PASS_DOOR & (0x01U)) << 5) | ((_m->ENGINE_BRAKE & (0x01U)) << 6) | ((_m->MARKER_LAMP & (0x01U)) << 7);
  _d[3] |= (_m->CABIN_CLIMATE & (0x01U)) | ((_m->CABIN_FAN_SPEED & (0x01U)) << 1) | ((_m->CABIN_TEMP & (0x01U)) << 2) | ((_m->EXHAUST_BRAKE & (0x01U)) << 3);
  _d[4] |= (_m->COUNTER & (0x0FU)) | ((_m->COMPLEMENT & (0x0FU)) << 4);
  _d[5] |= (_m->CONFIG_FAULT & (0x01U)) | ((_m->CAN_TIMEOUT_FAULT & (0x01U)) << 1) | ((_m->INTERNAL_SUPPLY_VOLTAGE_FAULT & (0x01U)) << 2) | ((_m->SUPERVISORY_TIMEOUT & (0x01U)) << 3) | ((_m->SUPERVISORY_SANITY_FAULT & (0x01U)) << 4) | ((_m->WATCHDOG_SANITY_FAULT & (0x01U)) << 5) | ((_m->WATCHDOG_SYSTEM_PRESENT_FAULT & (0x01U)) << 6) | ((_m->COMPONENT_READY & (0x01U)) << 7);

  *_len = COMPONENT_RPT_02_DLC;
  *_ide = COMPONENT_RPT_02_IDE;
  return COMPONENT_RPT_02_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_COMPONENT_RPT_03_pacmod12(COMPONENT_RPT_03_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->COMPONENT_TYPE = (_d[0] & (0x0FU));
  _m->ACCEL = ((_d[0] >> 4) & (0x01U));
  _m->BRAKE = ((_d[0] >> 5) & (0x01U));
  _m->CRUISE_CONTROL_BUTTONS = ((_d[0] >> 6) & (0x01U));
  _m->DASH_CONTROLS_LEFT = ((_d[0] >> 7) & (0x01U));
  _m->DASH_CONTROLS_RIGHT = (_d[1] & (0x01U));
  _m->HAZARD_LIGHTS = ((_d[1] >> 1) & (0x01U));
  _m->HEADLIGHT = ((_d[1] >> 2) & (0x01U));
  _m->HORN = ((_d[1] >> 3) & (0x01U));
  _m->MEDIA_CONTROLS = ((_d[1] >> 4) & (0x01U));
  _m->PARKING_BRAKE = ((_d[1] >> 5) & (0x01U));
  _m->SHIFT = ((_d[1] >> 6) & (0x01U));
  _m->SPRAYER = ((_d[1] >> 7) & (0x01U));
  _m->STEERING = (_d[2] & (0x01U));
  _m->TURN = ((_d[2] >> 1) & (0x01U));
  _m->WIPER = ((_d[2] >> 2) & (0x01U));
  _m->WATCHDOG = ((_d[2] >> 3) & (0x01U));
  _m->BRAKE_DECEL = ((_d[2] >> 4) & (0x01U));
  _m->REAR_PASS_DOOR = ((_d[2] >> 5) & (0x01U));
  _m->ENGINE_BRAKE = ((_d[2] >> 6) & (0x01U));
  _m->MARKER_LAMP = ((_d[2] >> 7) & (0x01U));
  _m->CABIN_CLIMATE = (_d[3] & (0x01U));
  _m->CABIN_FAN_SPEED = ((_d[3] >> 1) & (0x01U));
  _m->CABIN_TEMP = ((_d[3] >> 2) & (0x01U));
  _m->EXHAUST_BRAKE = ((_d[3] >> 3) & (0x01U));
  _m->COUNTER = (_d[4] & (0x0FU));
  _m->COMPLEMENT = ((_d[4] >> 4) & (0x0FU));
  _m->CONFIG_FAULT = (_d[5] & (0x01U));
  _m->CAN_TIMEOUT_FAULT = ((_d[5] >> 1) & (0x01U));
  _m->INTERNAL_SUPPLY_VOLTAGE_FAULT = ((_d[5] >> 2) & (0x01U));
  _m->SUPERVISORY_TIMEOUT = ((_d[5] >> 3) & (0x01U));
  _m->SUPERVISORY_SANITY_FAULT = ((_d[5] >> 4) & (0x01U));
  _m->WATCHDOG_SANITY_FAULT = ((_d[5] >> 5) & (0x01U));
  _m->WATCHDOG_SYSTEM_PRESENT_FAULT = ((_d[5] >> 6) & (0x01U));
  _m->COMPONENT_READY = ((_d[5] >> 7) & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < COMPONENT_RPT_03_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_COMPONENT_RPT_03_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return COMPONENT_RPT_03_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_COMPONENT_RPT_03_pacmod12(COMPONENT_RPT_03_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < COMPONENT_RPT_03_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->COMPONENT_TYPE & (0x0FU)) | ((_m->ACCEL & (0x01U)) << 4) | ((_m->BRAKE & (0x01U)) << 5) | ((_m->CRUISE_CONTROL_BUTTONS & (0x01U)) << 6) | ((_m->DASH_CONTROLS_LEFT & (0x01U)) << 7);
  cframe->Data[1] |= (_m->DASH_CONTROLS_RIGHT & (0x01U)) | ((_m->HAZARD_LIGHTS & (0x01U)) << 1) | ((_m->HEADLIGHT & (0x01U)) << 2) | ((_m->HORN & (0x01U)) << 3) | ((_m->MEDIA_CONTROLS & (0x01U)) << 4) | ((_m->PARKING_BRAKE & (0x01U)) << 5) | ((_m->SHIFT & (0x01U)) << 6) | ((_m->SPRAYER & (0x01U)) << 7);
  cframe->Data[2] |= (_m->STEERING & (0x01U)) | ((_m->TURN & (0x01U)) << 1) | ((_m->WIPER & (0x01U)) << 2) | ((_m->WATCHDOG & (0x01U)) << 3) | ((_m->BRAKE_DECEL & (0x01U)) << 4) | ((_m->REAR_PASS_DOOR & (0x01U)) << 5) | ((_m->ENGINE_BRAKE & (0x01U)) << 6) | ((_m->MARKER_LAMP & (0x01U)) << 7);
  cframe->Data[3] |= (_m->CABIN_CLIMATE & (0x01U)) | ((_m->CABIN_FAN_SPEED & (0x01U)) << 1) | ((_m->CABIN_TEMP & (0x01U)) << 2) | ((_m->EXHAUST_BRAKE & (0x01U)) << 3);
  cframe->Data[4] |= (_m->COUNTER & (0x0FU)) | ((_m->COMPLEMENT & (0x0FU)) << 4);
  cframe->Data[5] |= (_m->CONFIG_FAULT & (0x01U)) | ((_m->CAN_TIMEOUT_FAULT & (0x01U)) << 1) | ((_m->INTERNAL_SUPPLY_VOLTAGE_FAULT & (0x01U)) << 2) | ((_m->SUPERVISORY_TIMEOUT & (0x01U)) << 3) | ((_m->SUPERVISORY_SANITY_FAULT & (0x01U)) << 4) | ((_m->WATCHDOG_SANITY_FAULT & (0x01U)) << 5) | ((_m->WATCHDOG_SYSTEM_PRESENT_FAULT & (0x01U)) << 6) | ((_m->COMPONENT_READY & (0x01U)) << 7);

  cframe->MsgId = COMPONENT_RPT_03_CANID;
  cframe->DLC = COMPONENT_RPT_03_DLC;
  cframe->IDE = COMPONENT_RPT_03_IDE;
  return COMPONENT_RPT_03_CANID;
}

#else

uint32_t Pack_COMPONENT_RPT_03_pacmod12(COMPONENT_RPT_03_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < COMPONENT_RPT_03_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->COMPONENT_TYPE & (0x0FU)) | ((_m->ACCEL & (0x01U)) << 4) | ((_m->BRAKE & (0x01U)) << 5) | ((_m->CRUISE_CONTROL_BUTTONS & (0x01U)) << 6) | ((_m->DASH_CONTROLS_LEFT & (0x01U)) << 7);
  _d[1] |= (_m->DASH_CONTROLS_RIGHT & (0x01U)) | ((_m->HAZARD_LIGHTS & (0x01U)) << 1) | ((_m->HEADLIGHT & (0x01U)) << 2) | ((_m->HORN & (0x01U)) << 3) | ((_m->MEDIA_CONTROLS & (0x01U)) << 4) | ((_m->PARKING_BRAKE & (0x01U)) << 5) | ((_m->SHIFT & (0x01U)) << 6) | ((_m->SPRAYER & (0x01U)) << 7);
  _d[2] |= (_m->STEERING & (0x01U)) | ((_m->TURN & (0x01U)) << 1) | ((_m->WIPER & (0x01U)) << 2) | ((_m->WATCHDOG & (0x01U)) << 3) | ((_m->BRAKE_DECEL & (0x01U)) << 4) | ((_m->REAR_PASS_DOOR & (0x01U)) << 5) | ((_m->ENGINE_BRAKE & (0x01U)) << 6) | ((_m->MARKER_LAMP & (0x01U)) << 7);
  _d[3] |= (_m->CABIN_CLIMATE & (0x01U)) | ((_m->CABIN_FAN_SPEED & (0x01U)) << 1) | ((_m->CABIN_TEMP & (0x01U)) << 2) | ((_m->EXHAUST_BRAKE & (0x01U)) << 3);
  _d[4] |= (_m->COUNTER & (0x0FU)) | ((_m->COMPLEMENT & (0x0FU)) << 4);
  _d[5] |= (_m->CONFIG_FAULT & (0x01U)) | ((_m->CAN_TIMEOUT_FAULT & (0x01U)) << 1) | ((_m->INTERNAL_SUPPLY_VOLTAGE_FAULT & (0x01U)) << 2) | ((_m->SUPERVISORY_TIMEOUT & (0x01U)) << 3) | ((_m->SUPERVISORY_SANITY_FAULT & (0x01U)) << 4) | ((_m->WATCHDOG_SANITY_FAULT & (0x01U)) << 5) | ((_m->WATCHDOG_SYSTEM_PRESENT_FAULT & (0x01U)) << 6) | ((_m->COMPONENT_READY & (0x01U)) << 7);

  *_len = COMPONENT_RPT_03_DLC;
  *_ide = COMPONENT_RPT_03_IDE;
  return COMPONENT_RPT_03_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_COMPONENT_RPT_04_pacmod12(COMPONENT_RPT_04_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->COMPONENT_TYPE = (_d[0] & (0x0FU));
  _m->ACCEL = ((_d[0] >> 4) & (0x01U));
  _m->BRAKE = ((_d[0] >> 5) & (0x01U));
  _m->CRUISE_CONTROL_BUTTONS = ((_d[0] >> 6) & (0x01U));
  _m->DASH_CONTROLS_LEFT = ((_d[0] >> 7) & (0x01U));
  _m->DASH_CONTROLS_RIGHT = (_d[1] & (0x01U));
  _m->HAZARD_LIGHTS = ((_d[1] >> 1) & (0x01U));
  _m->HEADLIGHT = ((_d[1] >> 2) & (0x01U));
  _m->HORN = ((_d[1] >> 3) & (0x01U));
  _m->MEDIA_CONTROLS = ((_d[1] >> 4) & (0x01U));
  _m->PARKING_BRAKE = ((_d[1] >> 5) & (0x01U));
  _m->SHIFT = ((_d[1] >> 6) & (0x01U));
  _m->SPRAYER = ((_d[1] >> 7) & (0x01U));
  _m->STEERING = (_d[2] & (0x01U));
  _m->TURN = ((_d[2] >> 1) & (0x01U));
  _m->WIPER = ((_d[2] >> 2) & (0x01U));
  _m->WATCHDOG = ((_d[2] >> 3) & (0x01U));
  _m->BRAKE_DECEL = ((_d[2] >> 4) & (0x01U));
  _m->REAR_PASS_DOOR = ((_d[2] >> 5) & (0x01U));
  _m->ENGINE_BRAKE = ((_d[2] >> 6) & (0x01U));
  _m->MARKER_LAMP = ((_d[2] >> 7) & (0x01U));
  _m->CABIN_CLIMATE = (_d[3] & (0x01U));
  _m->CABIN_FAN_SPEED = ((_d[3] >> 1) & (0x01U));
  _m->CABIN_TEMP = ((_d[3] >> 2) & (0x01U));
  _m->EXHAUST_BRAKE = ((_d[3] >> 3) & (0x01U));
  _m->COUNTER = (_d[4] & (0x0FU));
  _m->COMPLEMENT = ((_d[4] >> 4) & (0x0FU));
  _m->CONFIG_FAULT = (_d[5] & (0x01U));
  _m->CAN_TIMEOUT_FAULT = ((_d[5] >> 1) & (0x01U));
  _m->INTERNAL_SUPPLY_VOLTAGE_FAULT = ((_d[5] >> 2) & (0x01U));
  _m->SUPERVISORY_TIMEOUT = ((_d[5] >> 3) & (0x01U));
  _m->SUPERVISORY_SANITY_FAULT = ((_d[5] >> 4) & (0x01U));
  _m->WATCHDOG_SANITY_FAULT = ((_d[5] >> 5) & (0x01U));
  _m->WATCHDOG_SYSTEM_PRESENT_FAULT = ((_d[5] >> 6) & (0x01U));
  _m->COMPONENT_READY = ((_d[5] >> 7) & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < COMPONENT_RPT_04_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_COMPONENT_RPT_04_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return COMPONENT_RPT_04_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_COMPONENT_RPT_04_pacmod12(COMPONENT_RPT_04_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < COMPONENT_RPT_04_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->COMPONENT_TYPE & (0x0FU)) | ((_m->ACCEL & (0x01U)) << 4) | ((_m->BRAKE & (0x01U)) << 5) | ((_m->CRUISE_CONTROL_BUTTONS & (0x01U)) << 6) | ((_m->DASH_CONTROLS_LEFT & (0x01U)) << 7);
  cframe->Data[1] |= (_m->DASH_CONTROLS_RIGHT & (0x01U)) | ((_m->HAZARD_LIGHTS & (0x01U)) << 1) | ((_m->HEADLIGHT & (0x01U)) << 2) | ((_m->HORN & (0x01U)) << 3) | ((_m->MEDIA_CONTROLS & (0x01U)) << 4) | ((_m->PARKING_BRAKE & (0x01U)) << 5) | ((_m->SHIFT & (0x01U)) << 6) | ((_m->SPRAYER & (0x01U)) << 7);
  cframe->Data[2] |= (_m->STEERING & (0x01U)) | ((_m->TURN & (0x01U)) << 1) | ((_m->WIPER & (0x01U)) << 2) | ((_m->WATCHDOG & (0x01U)) << 3) | ((_m->BRAKE_DECEL & (0x01U)) << 4) | ((_m->REAR_PASS_DOOR & (0x01U)) << 5) | ((_m->ENGINE_BRAKE & (0x01U)) << 6) | ((_m->MARKER_LAMP & (0x01U)) << 7);
  cframe->Data[3] |= (_m->CABIN_CLIMATE & (0x01U)) | ((_m->CABIN_FAN_SPEED & (0x01U)) << 1) | ((_m->CABIN_TEMP & (0x01U)) << 2) | ((_m->EXHAUST_BRAKE & (0x01U)) << 3);
  cframe->Data[4] |= (_m->COUNTER & (0x0FU)) | ((_m->COMPLEMENT & (0x0FU)) << 4);
  cframe->Data[5] |= (_m->CONFIG_FAULT & (0x01U)) | ((_m->CAN_TIMEOUT_FAULT & (0x01U)) << 1) | ((_m->INTERNAL_SUPPLY_VOLTAGE_FAULT & (0x01U)) << 2) | ((_m->SUPERVISORY_TIMEOUT & (0x01U)) << 3) | ((_m->SUPERVISORY_SANITY_FAULT & (0x01U)) << 4) | ((_m->WATCHDOG_SANITY_FAULT & (0x01U)) << 5) | ((_m->WATCHDOG_SYSTEM_PRESENT_FAULT & (0x01U)) << 6) | ((_m->COMPONENT_READY & (0x01U)) << 7);

  cframe->MsgId = COMPONENT_RPT_04_CANID;
  cframe->DLC = COMPONENT_RPT_04_DLC;
  cframe->IDE = COMPONENT_RPT_04_IDE;
  return COMPONENT_RPT_04_CANID;
}

#else

uint32_t Pack_COMPONENT_RPT_04_pacmod12(COMPONENT_RPT_04_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < COMPONENT_RPT_04_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->COMPONENT_TYPE & (0x0FU)) | ((_m->ACCEL & (0x01U)) << 4) | ((_m->BRAKE & (0x01U)) << 5) | ((_m->CRUISE_CONTROL_BUTTONS & (0x01U)) << 6) | ((_m->DASH_CONTROLS_LEFT & (0x01U)) << 7);
  _d[1] |= (_m->DASH_CONTROLS_RIGHT & (0x01U)) | ((_m->HAZARD_LIGHTS & (0x01U)) << 1) | ((_m->HEADLIGHT & (0x01U)) << 2) | ((_m->HORN & (0x01U)) << 3) | ((_m->MEDIA_CONTROLS & (0x01U)) << 4) | ((_m->PARKING_BRAKE & (0x01U)) << 5) | ((_m->SHIFT & (0x01U)) << 6) | ((_m->SPRAYER & (0x01U)) << 7);
  _d[2] |= (_m->STEERING & (0x01U)) | ((_m->TURN & (0x01U)) << 1) | ((_m->WIPER & (0x01U)) << 2) | ((_m->WATCHDOG & (0x01U)) << 3) | ((_m->BRAKE_DECEL & (0x01U)) << 4) | ((_m->REAR_PASS_DOOR & (0x01U)) << 5) | ((_m->ENGINE_BRAKE & (0x01U)) << 6) | ((_m->MARKER_LAMP & (0x01U)) << 7);
  _d[3] |= (_m->CABIN_CLIMATE & (0x01U)) | ((_m->CABIN_FAN_SPEED & (0x01U)) << 1) | ((_m->CABIN_TEMP & (0x01U)) << 2) | ((_m->EXHAUST_BRAKE & (0x01U)) << 3);
  _d[4] |= (_m->COUNTER & (0x0FU)) | ((_m->COMPLEMENT & (0x0FU)) << 4);
  _d[5] |= (_m->CONFIG_FAULT & (0x01U)) | ((_m->CAN_TIMEOUT_FAULT & (0x01U)) << 1) | ((_m->INTERNAL_SUPPLY_VOLTAGE_FAULT & (0x01U)) << 2) | ((_m->SUPERVISORY_TIMEOUT & (0x01U)) << 3) | ((_m->SUPERVISORY_SANITY_FAULT & (0x01U)) << 4) | ((_m->WATCHDOG_SANITY_FAULT & (0x01U)) << 5) | ((_m->WATCHDOG_SYSTEM_PRESENT_FAULT & (0x01U)) << 6) | ((_m->COMPONENT_READY & (0x01U)) << 7);

  *_len = COMPONENT_RPT_04_DLC;
  *_ide = COMPONENT_RPT_04_IDE;
  return COMPONENT_RPT_04_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_SAFETY_FUNC_RPT_pacmod12(SAFETY_FUNC_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->COMMANDED_VALUE = (_d[0] & (0x0FU));
  _m->STATE = ((_d[0] >> 4) & (0x0FU));
  _m->AUTOMSMAN_OPCTRL = (_d[1] & (0x03U));
  _m->CABIN_SAFETY_BRAKE_OPCTRL = ((_d[1] >> 2) & (0x03U));
  _m->REMOTE_STOP_STATUS = ((_d[1] >> 4) & (0x03U));
  _m->ENGINE_STATUS = ((_d[1] >> 6) & (0x01U));
  _m->PACMOD_SYSTEM_STATUS = ((_d[1] >> 7) & (0x01U));
  _m->USER_PC_FAULT = (_d[2] & (0x03U));
  _m->PACMOD_SYSTEM_FAULT = ((_d[2] >> 2) & (0x03U));
  _m->MANUAL_STATE_OBTAINABLE = (_d[3] & (0x01U));
  _m->AUTOMS_READY_STATE_OBTAINABLE = ((_d[3] >> 1) & (0x01U));
  _m->AUTOMS_STATE_OBTAINABLE = ((_d[3] >> 2) & (0x01U));
  _m->MANUAL_READY_STATE_OBTAINABLE = ((_d[3] >> 3) & (0x01U));
  _m->CRITICAL_STOP1_STATE_OBTAINABLE = ((_d[3] >> 4) & (0x01U));
  _m->CRITICAL_STOP2_STATE_OBTAINABLE = ((_d[3] >> 5) & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < SAFETY_FUNC_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_SAFETY_FUNC_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return SAFETY_FUNC_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_SAFETY_FUNC_RPT_pacmod12(SAFETY_FUNC_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < SAFETY_FUNC_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->COMMANDED_VALUE & (0x0FU)) | ((_m->STATE & (0x0FU)) << 4);
  cframe->Data[1] |= (_m->AUTOMSMAN_OPCTRL & (0x03U)) | ((_m->CABIN_SAFETY_BRAKE_OPCTRL & (0x03U)) << 2) | ((_m->REMOTE_STOP_STATUS & (0x03U)) << 4) | ((_m->ENGINE_STATUS & (0x01U)) << 6) | ((_m->PACMOD_SYSTEM_STATUS & (0x01U)) << 7);
  cframe->Data[2] |= (_m->USER_PC_FAULT & (0x03U)) | ((_m->PACMOD_SYSTEM_FAULT & (0x03U)) << 2);
  cframe->Data[3] |= (_m->MANUAL_STATE_OBTAINABLE & (0x01U)) | ((_m->AUTOMS_READY_STATE_OBTAINABLE & (0x01U)) << 1) | ((_m->AUTOMS_STATE_OBTAINABLE & (0x01U)) << 2) | ((_m->MANUAL_READY_STATE_OBTAINABLE & (0x01U)) << 3) | ((_m->CRITICAL_STOP1_STATE_OBTAINABLE & (0x01U)) << 4) | ((_m->CRITICAL_STOP2_STATE_OBTAINABLE & (0x01U)) << 5);

  cframe->MsgId = SAFETY_FUNC_RPT_CANID;
  cframe->DLC = SAFETY_FUNC_RPT_DLC;
  cframe->IDE = SAFETY_FUNC_RPT_IDE;
  return SAFETY_FUNC_RPT_CANID;
}

#else

uint32_t Pack_SAFETY_FUNC_RPT_pacmod12(SAFETY_FUNC_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < SAFETY_FUNC_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->COMMANDED_VALUE & (0x0FU)) | ((_m->STATE & (0x0FU)) << 4);
  _d[1] |= (_m->AUTOMSMAN_OPCTRL & (0x03U)) | ((_m->CABIN_SAFETY_BRAKE_OPCTRL & (0x03U)) << 2) | ((_m->REMOTE_STOP_STATUS & (0x03U)) << 4) | ((_m->ENGINE_STATUS & (0x01U)) << 6) | ((_m->PACMOD_SYSTEM_STATUS & (0x01U)) << 7);
  _d[2] |= (_m->USER_PC_FAULT & (0x03U)) | ((_m->PACMOD_SYSTEM_FAULT & (0x03U)) << 2);
  _d[3] |= (_m->MANUAL_STATE_OBTAINABLE & (0x01U)) | ((_m->AUTOMS_READY_STATE_OBTAINABLE & (0x01U)) << 1) | ((_m->AUTOMS_STATE_OBTAINABLE & (0x01U)) << 2) | ((_m->MANUAL_READY_STATE_OBTAINABLE & (0x01U)) << 3) | ((_m->CRITICAL_STOP1_STATE_OBTAINABLE & (0x01U)) << 4) | ((_m->CRITICAL_STOP2_STATE_OBTAINABLE & (0x01U)) << 5);

  *_len = SAFETY_FUNC_RPT_DLC;
  *_ide = SAFETY_FUNC_RPT_IDE;
  return SAFETY_FUNC_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_SAFETY_BRAKE_RPT_pacmod12(SAFETY_BRAKE_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->COMMANDED_VALUE = (_d[0] & (0x01U));
  _m->OUTPUT_VALUE = ((_d[0] >> 1) & (0x03U));
  _m->COMMAND_REPORTED_FAULT = ((_d[0] >> 3) & (0x01U));
  _m->COMMAND_TIMEOUT = ((_d[0] >> 4) & (0x01U));
  _m->COMMAND_PERMITTED = ((_d[0] >> 5) & (0x01U));
  _m->REPORTED_FAULT = ((_d[0] >> 6) & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < SAFETY_BRAKE_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_SAFETY_BRAKE_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return SAFETY_BRAKE_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_SAFETY_BRAKE_RPT_pacmod12(SAFETY_BRAKE_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < SAFETY_BRAKE_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->COMMANDED_VALUE & (0x01U)) | ((_m->OUTPUT_VALUE & (0x03U)) << 1) | ((_m->COMMAND_REPORTED_FAULT & (0x01U)) << 3) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 4) | ((_m->COMMAND_PERMITTED & (0x01U)) << 5) | ((_m->REPORTED_FAULT & (0x01U)) << 6);

  cframe->MsgId = SAFETY_BRAKE_RPT_CANID;
  cframe->DLC = SAFETY_BRAKE_RPT_DLC;
  cframe->IDE = SAFETY_BRAKE_RPT_IDE;
  return SAFETY_BRAKE_RPT_CANID;
}

#else

uint32_t Pack_SAFETY_BRAKE_RPT_pacmod12(SAFETY_BRAKE_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < SAFETY_BRAKE_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->COMMANDED_VALUE & (0x01U)) | ((_m->OUTPUT_VALUE & (0x03U)) << 1) | ((_m->COMMAND_REPORTED_FAULT & (0x01U)) << 3) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 4) | ((_m->COMMAND_PERMITTED & (0x01U)) << 5) | ((_m->REPORTED_FAULT & (0x01U)) << 6);

  *_len = SAFETY_BRAKE_RPT_DLC;
  *_ide = SAFETY_BRAKE_RPT_IDE;
  return SAFETY_BRAKE_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_VEHICLE_FAULT_RPT_pacmod12(VEHICLE_FAULT_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENGINE_CHECK_LIGHT = (_d[0] & (0x01U));
  _m->ENGINE_CHECK_LIGHT_AVAIL = ((_d[0] >> 1) & (0x01U));
  _m->TRC_FAULT_LIGHT = ((_d[0] >> 2) & (0x01U));
  _m->TRC_FAULT_LIGHT_AVAIL = ((_d[0] >> 3) & (0x01U));
  _m->TRC_OFF_FAULT_LIGHT = ((_d[0] >> 4) & (0x01U));
  _m->TRC_OFF_FAULT_LIGHT_AVAIL = ((_d[0] >> 5) & (0x01U));
  _m->ANTILOCK_BRAKE_FAULT_LIGHT = ((_d[0] >> 6) & (0x01U));
  _m->ANTILOCK_BRAKE_FAULT_LIGHT_AVAIL = ((_d[0] >> 7) & (0x01U));
  _m->TIRE_FAULT_LIGHT = (_d[1] & (0x01U));
  _m->TIRE_FAULT_LIGHT_AVAIL = ((_d[1] >> 1) & (0x01U));
  _m->AIR_BAGS_FAULT_LIGHT = ((_d[1] >> 2) & (0x01U));
  _m->AIR_BAGS_FAULT_LIGHT_AVAIL = ((_d[1] >> 3) & (0x01U));
  _m->LOW_ENGINE_OIL_PRESSURE = ((_d[1] >> 4) & (0x01U));
  _m->LOW_ENGINE_OIL_PRESSURE_AVAIL = ((_d[1] >> 5) & (0x01U));
  _m->BRAKE_FAULT = ((_d[1] >> 6) & (0x01U));
  _m->BRAKE_FAULT_AVAIL = ((_d[1] >> 7) & (0x01U));
  _m->BRK_APPLIED_POWER_REDUCED = (_d[2] & (0x01U));
  _m->BRK_APPLIED_POWER_REDUCED_AVAIL = ((_d[2] >> 1) & (0x01U));
  _m->STEERING_LOSS_STOP_SAFELY = ((_d[2] >> 2) & (0x01U));
  _m->STEERING_LOSS_STOP_SAFELY_AVAIL = ((_d[2] >> 3) & (0x01U));
  _m->STEERING_FAULT_SERVICE_NOW = ((_d[2] >> 4) & (0x01U));
  _m->STEERING_FAULT_SERVICE_NOW_AVAIL = ((_d[2] >> 5) & (0x01U));
  _m->XMSN_FAULT_SERVICE_NOW = ((_d[2] >> 6) & (0x01U));
  _m->XMSN_FAULT_SERVICE_NOW_AVAIL = ((_d[2] >> 7) & (0x01U));
  _m->XMSN_OVER_TEMP_STOP_SAFELY = (_d[3] & (0x01U));
  _m->XMSN_OVER_TEMP_STOP_SAFELY_AVAIL = ((_d[3] >> 1) & (0x01U));
  _m->LOW_BATTERY_FEATURES_OFF = ((_d[3] >> 2) & (0x01U));
  _m->LOW_BATTERY_FEATURES_OFF_AVAIL = ((_d[3] >> 3) & (0x01U));
  _m->CHARGING_SYSTEM_FAULT = ((_d[3] >> 4) & (0x01U));
  _m->CHARGING_SYSTEM_FAULT_AVAIL = ((_d[3] >> 5) & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < VEHICLE_FAULT_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_VEHICLE_FAULT_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return VEHICLE_FAULT_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_VEHICLE_FAULT_RPT_pacmod12(VEHICLE_FAULT_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < VEHICLE_FAULT_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENGINE_CHECK_LIGHT & (0x01U)) | ((_m->ENGINE_CHECK_LIGHT_AVAIL & (0x01U)) << 1) | ((_m->TRC_FAULT_LIGHT & (0x01U)) << 2) | ((_m->TRC_FAULT_LIGHT_AVAIL & (0x01U)) << 3) | ((_m->TRC_OFF_FAULT_LIGHT & (0x01U)) << 4) | ((_m->TRC_OFF_FAULT_LIGHT_AVAIL & (0x01U)) << 5) | ((_m->ANTILOCK_BRAKE_FAULT_LIGHT & (0x01U)) << 6) | ((_m->ANTILOCK_BRAKE_FAULT_LIGHT_AVAIL & (0x01U)) << 7);
  cframe->Data[1] |= (_m->TIRE_FAULT_LIGHT & (0x01U)) | ((_m->TIRE_FAULT_LIGHT_AVAIL & (0x01U)) << 1) | ((_m->AIR_BAGS_FAULT_LIGHT & (0x01U)) << 2) | ((_m->AIR_BAGS_FAULT_LIGHT_AVAIL & (0x01U)) << 3) | ((_m->LOW_ENGINE_OIL_PRESSURE & (0x01U)) << 4) | ((_m->LOW_ENGINE_OIL_PRESSURE_AVAIL & (0x01U)) << 5) | ((_m->BRAKE_FAULT & (0x01U)) << 6) | ((_m->BRAKE_FAULT_AVAIL & (0x01U)) << 7);
  cframe->Data[2] |= (_m->BRK_APPLIED_POWER_REDUCED & (0x01U)) | ((_m->BRK_APPLIED_POWER_REDUCED_AVAIL & (0x01U)) << 1) | ((_m->STEERING_LOSS_STOP_SAFELY & (0x01U)) << 2) | ((_m->STEERING_LOSS_STOP_SAFELY_AVAIL & (0x01U)) << 3) | ((_m->STEERING_FAULT_SERVICE_NOW & (0x01U)) << 4) | ((_m->STEERING_FAULT_SERVICE_NOW_AVAIL & (0x01U)) << 5) | ((_m->XMSN_FAULT_SERVICE_NOW & (0x01U)) << 6) | ((_m->XMSN_FAULT_SERVICE_NOW_AVAIL & (0x01U)) << 7);
  cframe->Data[3] |= (_m->XMSN_OVER_TEMP_STOP_SAFELY & (0x01U)) | ((_m->XMSN_OVER_TEMP_STOP_SAFELY_AVAIL & (0x01U)) << 1) | ((_m->LOW_BATTERY_FEATURES_OFF & (0x01U)) << 2) | ((_m->LOW_BATTERY_FEATURES_OFF_AVAIL & (0x01U)) << 3) | ((_m->CHARGING_SYSTEM_FAULT & (0x01U)) << 4) | ((_m->CHARGING_SYSTEM_FAULT_AVAIL & (0x01U)) << 5);

  cframe->MsgId = VEHICLE_FAULT_RPT_CANID;
  cframe->DLC = VEHICLE_FAULT_RPT_DLC;
  cframe->IDE = VEHICLE_FAULT_RPT_IDE;
  return VEHICLE_FAULT_RPT_CANID;
}

#else

uint32_t Pack_VEHICLE_FAULT_RPT_pacmod12(VEHICLE_FAULT_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < VEHICLE_FAULT_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENGINE_CHECK_LIGHT & (0x01U)) | ((_m->ENGINE_CHECK_LIGHT_AVAIL & (0x01U)) << 1) | ((_m->TRC_FAULT_LIGHT & (0x01U)) << 2) | ((_m->TRC_FAULT_LIGHT_AVAIL & (0x01U)) << 3) | ((_m->TRC_OFF_FAULT_LIGHT & (0x01U)) << 4) | ((_m->TRC_OFF_FAULT_LIGHT_AVAIL & (0x01U)) << 5) | ((_m->ANTILOCK_BRAKE_FAULT_LIGHT & (0x01U)) << 6) | ((_m->ANTILOCK_BRAKE_FAULT_LIGHT_AVAIL & (0x01U)) << 7);
  _d[1] |= (_m->TIRE_FAULT_LIGHT & (0x01U)) | ((_m->TIRE_FAULT_LIGHT_AVAIL & (0x01U)) << 1) | ((_m->AIR_BAGS_FAULT_LIGHT & (0x01U)) << 2) | ((_m->AIR_BAGS_FAULT_LIGHT_AVAIL & (0x01U)) << 3) | ((_m->LOW_ENGINE_OIL_PRESSURE & (0x01U)) << 4) | ((_m->LOW_ENGINE_OIL_PRESSURE_AVAIL & (0x01U)) << 5) | ((_m->BRAKE_FAULT & (0x01U)) << 6) | ((_m->BRAKE_FAULT_AVAIL & (0x01U)) << 7);
  _d[2] |= (_m->BRK_APPLIED_POWER_REDUCED & (0x01U)) | ((_m->BRK_APPLIED_POWER_REDUCED_AVAIL & (0x01U)) << 1) | ((_m->STEERING_LOSS_STOP_SAFELY & (0x01U)) << 2) | ((_m->STEERING_LOSS_STOP_SAFELY_AVAIL & (0x01U)) << 3) | ((_m->STEERING_FAULT_SERVICE_NOW & (0x01U)) << 4) | ((_m->STEERING_FAULT_SERVICE_NOW_AVAIL & (0x01U)) << 5) | ((_m->XMSN_FAULT_SERVICE_NOW & (0x01U)) << 6) | ((_m->XMSN_FAULT_SERVICE_NOW_AVAIL & (0x01U)) << 7);
  _d[3] |= (_m->XMSN_OVER_TEMP_STOP_SAFELY & (0x01U)) | ((_m->XMSN_OVER_TEMP_STOP_SAFELY_AVAIL & (0x01U)) << 1) | ((_m->LOW_BATTERY_FEATURES_OFF & (0x01U)) << 2) | ((_m->LOW_BATTERY_FEATURES_OFF_AVAIL & (0x01U)) << 3) | ((_m->CHARGING_SYSTEM_FAULT & (0x01U)) << 4) | ((_m->CHARGING_SYSTEM_FAULT_AVAIL & (0x01U)) << 5);

  *_len = VEHICLE_FAULT_RPT_DLC;
  *_ide = VEHICLE_FAULT_RPT_IDE;
  return VEHICLE_FAULT_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_GLOBAL_CMD_pacmod12(GLOBAL_CMD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->CLEAR_FAULTS = (_d[0] & (0x01U));
  _m->SANITY_CHECK_REQUIRED = ((_d[0] >> 1) & (0x01U));
  _m->COUNTER = (_d[1] & (0x0FU));
  _m->COMPLEMENT = ((_d[1] >> 4) & (0x0FU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < GLOBAL_CMD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_GLOBAL_CMD_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return GLOBAL_CMD_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_GLOBAL_CMD_pacmod12(GLOBAL_CMD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < GLOBAL_CMD_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->CLEAR_FAULTS & (0x01U)) | ((_m->SANITY_CHECK_REQUIRED & (0x01U)) << 1);
  cframe->Data[1] |= (_m->COUNTER & (0x0FU)) | ((_m->COMPLEMENT & (0x0FU)) << 4);

  cframe->MsgId = GLOBAL_CMD_CANID;
  cframe->DLC = GLOBAL_CMD_DLC;
  cframe->IDE = GLOBAL_CMD_IDE;
  return GLOBAL_CMD_CANID;
}

#else

uint32_t Pack_GLOBAL_CMD_pacmod12(GLOBAL_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < GLOBAL_CMD_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->CLEAR_FAULTS & (0x01U)) | ((_m->SANITY_CHECK_REQUIRED & (0x01U)) << 1);
  _d[1] |= (_m->COUNTER & (0x0FU)) | ((_m->COMPLEMENT & (0x0FU)) << 4);

  *_len = GLOBAL_CMD_DLC;
  *_ide = GLOBAL_CMD_IDE;
  return GLOBAL_CMD_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_SUPERVISORY_CTRL_pacmod12(SUPERVISORY_CTRL_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLE = (_d[0] & (0x01U));
  _m->COUNTER = (_d[1] & (0x0FU));
  _m->COMPLEMENT = ((_d[1] >> 4) & (0x0FU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < SUPERVISORY_CTRL_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_SUPERVISORY_CTRL_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return SUPERVISORY_CTRL_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_SUPERVISORY_CTRL_pacmod12(SUPERVISORY_CTRL_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < SUPERVISORY_CTRL_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLE & (0x01U));
  cframe->Data[1] |= (_m->COUNTER & (0x0FU)) | ((_m->COMPLEMENT & (0x0FU)) << 4);

  cframe->MsgId = SUPERVISORY_CTRL_CANID;
  cframe->DLC = SUPERVISORY_CTRL_DLC;
  cframe->IDE = SUPERVISORY_CTRL_IDE;
  return SUPERVISORY_CTRL_CANID;
}

#else

uint32_t Pack_SUPERVISORY_CTRL_pacmod12(SUPERVISORY_CTRL_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < SUPERVISORY_CTRL_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLE & (0x01U));
  _d[1] |= (_m->COUNTER & (0x0FU)) | ((_m->COMPLEMENT & (0x0FU)) << 4);

  *_len = SUPERVISORY_CTRL_DLC;
  *_ide = SUPERVISORY_CTRL_IDE;
  return SUPERVISORY_CTRL_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_SAFETY_FUNC_CMD_pacmod12(SAFETY_FUNC_CMD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->COMMAND = (_d[0] & (0x0FU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < SAFETY_FUNC_CMD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_SAFETY_FUNC_CMD_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return SAFETY_FUNC_CMD_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_SAFETY_FUNC_CMD_pacmod12(SAFETY_FUNC_CMD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < SAFETY_FUNC_CMD_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->COMMAND & (0x0FU));

  cframe->MsgId = SAFETY_FUNC_CMD_CANID;
  cframe->DLC = SAFETY_FUNC_CMD_DLC;
  cframe->IDE = SAFETY_FUNC_CMD_IDE;
  return SAFETY_FUNC_CMD_CANID;
}

#else

uint32_t Pack_SAFETY_FUNC_CMD_pacmod12(SAFETY_FUNC_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < SAFETY_FUNC_CMD_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->COMMAND & (0x0FU));

  *_len = SAFETY_FUNC_CMD_DLC;
  *_ide = SAFETY_FUNC_CMD_IDE;
  return SAFETY_FUNC_CMD_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_SAFETY_BRAKE_CMD_pacmod12(SAFETY_BRAKE_CMD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->SAFETY_BRAKE_CMD = (_d[0] & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < SAFETY_BRAKE_CMD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_SAFETY_BRAKE_CMD_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return SAFETY_BRAKE_CMD_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_SAFETY_BRAKE_CMD_pacmod12(SAFETY_BRAKE_CMD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < SAFETY_BRAKE_CMD_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->SAFETY_BRAKE_CMD & (0x01U));

  cframe->MsgId = SAFETY_BRAKE_CMD_CANID;
  cframe->DLC = SAFETY_BRAKE_CMD_DLC;
  cframe->IDE = SAFETY_BRAKE_CMD_IDE;
  return SAFETY_BRAKE_CMD_CANID;
}

#else

uint32_t Pack_SAFETY_BRAKE_CMD_pacmod12(SAFETY_BRAKE_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < SAFETY_BRAKE_CMD_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->SAFETY_BRAKE_CMD & (0x01U));

  *_len = SAFETY_BRAKE_CMD_DLC;
  *_ide = SAFETY_BRAKE_CMD_IDE;
  return SAFETY_BRAKE_CMD_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_ACCEL_CMD_pacmod12(ACCEL_CMD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLE = (_d[0] & (0x01U));
  _m->IGNORE_OVERRIDES = ((_d[0] >> 1) & (0x01U));
  _m->CLEAR_OVERRIDE = ((_d[0] >> 2) & (0x01U));
  _m->ACCEL_CMD_ro = ((_d[1] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->ACCEL_CMD_phys = (sigfloat_t)(PACMOD12_ACCEL_CMD_ro_fromS(_m->ACCEL_CMD_ro));
#endif // PACMOD12_USE_SIGFLOAT

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < ACCEL_CMD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_ACCEL_CMD_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return ACCEL_CMD_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_ACCEL_CMD_pacmod12(ACCEL_CMD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < ACCEL_CMD_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->ACCEL_CMD_ro = PACMOD12_ACCEL_CMD_ro_toS(_m->ACCEL_CMD_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  cframe->Data[1] |= ((_m->ACCEL_CMD_ro >> 8) & (0xFFU));
  cframe->Data[2] |= (_m->ACCEL_CMD_ro & (0xFFU));

  cframe->MsgId = ACCEL_CMD_CANID;
  cframe->DLC = ACCEL_CMD_DLC;
  cframe->IDE = ACCEL_CMD_IDE;
  return ACCEL_CMD_CANID;
}

#else

uint32_t Pack_ACCEL_CMD_pacmod12(ACCEL_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < ACCEL_CMD_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->ACCEL_CMD_ro = PACMOD12_ACCEL_CMD_ro_toS(_m->ACCEL_CMD_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  _d[1] |= ((_m->ACCEL_CMD_ro >> 8) & (0xFFU));
  _d[2] |= (_m->ACCEL_CMD_ro & (0xFFU));

  *_len = ACCEL_CMD_DLC;
  *_ide = ACCEL_CMD_IDE;
  return ACCEL_CMD_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_BRAKE_CMD_pacmod12(BRAKE_CMD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLE = (_d[0] & (0x01U));
  _m->IGNORE_OVERRIDES = ((_d[0] >> 1) & (0x01U));
  _m->CLEAR_OVERRIDE = ((_d[0] >> 2) & (0x01U));
  _m->BRAKE_CMD_ro = ((_d[1] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->BRAKE_CMD_phys = (sigfloat_t)(PACMOD12_BRAKE_CMD_ro_fromS(_m->BRAKE_CMD_ro));
#endif // PACMOD12_USE_SIGFLOAT

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < BRAKE_CMD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_BRAKE_CMD_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return BRAKE_CMD_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_BRAKE_CMD_pacmod12(BRAKE_CMD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < BRAKE_CMD_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->BRAKE_CMD_ro = PACMOD12_BRAKE_CMD_ro_toS(_m->BRAKE_CMD_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  cframe->Data[1] |= ((_m->BRAKE_CMD_ro >> 8) & (0xFFU));
  cframe->Data[2] |= (_m->BRAKE_CMD_ro & (0xFFU));

  cframe->MsgId = BRAKE_CMD_CANID;
  cframe->DLC = BRAKE_CMD_DLC;
  cframe->IDE = BRAKE_CMD_IDE;
  return BRAKE_CMD_CANID;
}

#else

uint32_t Pack_BRAKE_CMD_pacmod12(BRAKE_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < BRAKE_CMD_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->BRAKE_CMD_ro = PACMOD12_BRAKE_CMD_ro_toS(_m->BRAKE_CMD_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  _d[1] |= ((_m->BRAKE_CMD_ro >> 8) & (0xFFU));
  _d[2] |= (_m->BRAKE_CMD_ro & (0xFFU));

  *_len = BRAKE_CMD_DLC;
  *_ide = BRAKE_CMD_IDE;
  return BRAKE_CMD_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_CRUISE_CONTROL_BUTTONS_CMD_pacmod12(CRUISE_CONTROL_BUTTONS_CMD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLE = (_d[0] & (0x01U));
  _m->IGNORE_OVERRIDES = ((_d[0] >> 1) & (0x01U));
  _m->CLEAR_OVERRIDE = ((_d[0] >> 2) & (0x01U));
  _m->CRUISE_CONTROL_BUTTON = (_d[1] & (0xFFU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CRUISE_CONTROL_BUTTONS_CMD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CRUISE_CONTROL_BUTTONS_CMD_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return CRUISE_CONTROL_BUTTONS_CMD_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_CRUISE_CONTROL_BUTTONS_CMD_pacmod12(CRUISE_CONTROL_BUTTONS_CMD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < CRUISE_CONTROL_BUTTONS_CMD_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  cframe->Data[1] |= (_m->CRUISE_CONTROL_BUTTON & (0xFFU));

  cframe->MsgId = CRUISE_CONTROL_BUTTONS_CMD_CANID;
  cframe->DLC = CRUISE_CONTROL_BUTTONS_CMD_DLC;
  cframe->IDE = CRUISE_CONTROL_BUTTONS_CMD_IDE;
  return CRUISE_CONTROL_BUTTONS_CMD_CANID;
}

#else

uint32_t Pack_CRUISE_CONTROL_BUTTONS_CMD_pacmod12(CRUISE_CONTROL_BUTTONS_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < CRUISE_CONTROL_BUTTONS_CMD_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  _d[1] |= (_m->CRUISE_CONTROL_BUTTON & (0xFFU));

  *_len = CRUISE_CONTROL_BUTTONS_CMD_DLC;
  *_ide = CRUISE_CONTROL_BUTTONS_CMD_IDE;
  return CRUISE_CONTROL_BUTTONS_CMD_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_DASH_CONTROLS_LEFT_CMD_pacmod12(DASH_CONTROLS_LEFT_CMD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLE = (_d[0] & (0x01U));
  _m->IGNORE_OVERRIDES = ((_d[0] >> 1) & (0x01U));
  _m->CLEAR_OVERRIDE = ((_d[0] >> 2) & (0x01U));
  _m->DASH_CONTROLS_BUTTON = (_d[1] & (0xFFU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < DASH_CONTROLS_LEFT_CMD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_DASH_CONTROLS_LEFT_CMD_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return DASH_CONTROLS_LEFT_CMD_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_DASH_CONTROLS_LEFT_CMD_pacmod12(DASH_CONTROLS_LEFT_CMD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < DASH_CONTROLS_LEFT_CMD_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  cframe->Data[1] |= (_m->DASH_CONTROLS_BUTTON & (0xFFU));

  cframe->MsgId = DASH_CONTROLS_LEFT_CMD_CANID;
  cframe->DLC = DASH_CONTROLS_LEFT_CMD_DLC;
  cframe->IDE = DASH_CONTROLS_LEFT_CMD_IDE;
  return DASH_CONTROLS_LEFT_CMD_CANID;
}

#else

uint32_t Pack_DASH_CONTROLS_LEFT_CMD_pacmod12(DASH_CONTROLS_LEFT_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < DASH_CONTROLS_LEFT_CMD_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  _d[1] |= (_m->DASH_CONTROLS_BUTTON & (0xFFU));

  *_len = DASH_CONTROLS_LEFT_CMD_DLC;
  *_ide = DASH_CONTROLS_LEFT_CMD_IDE;
  return DASH_CONTROLS_LEFT_CMD_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_DASH_CONTROLS_RIGHT_CMD_pacmod12(DASH_CONTROLS_RIGHT_CMD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLE = (_d[0] & (0x01U));
  _m->IGNORE_OVERRIDES = ((_d[0] >> 1) & (0x01U));
  _m->CLEAR_OVERRIDE = ((_d[0] >> 2) & (0x01U));
  _m->DASH_CONTROLS_BUTTON = (_d[1] & (0xFFU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < DASH_CONTROLS_RIGHT_CMD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_DASH_CONTROLS_RIGHT_CMD_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return DASH_CONTROLS_RIGHT_CMD_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_DASH_CONTROLS_RIGHT_CMD_pacmod12(DASH_CONTROLS_RIGHT_CMD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < DASH_CONTROLS_RIGHT_CMD_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  cframe->Data[1] |= (_m->DASH_CONTROLS_BUTTON & (0xFFU));

  cframe->MsgId = DASH_CONTROLS_RIGHT_CMD_CANID;
  cframe->DLC = DASH_CONTROLS_RIGHT_CMD_DLC;
  cframe->IDE = DASH_CONTROLS_RIGHT_CMD_IDE;
  return DASH_CONTROLS_RIGHT_CMD_CANID;
}

#else

uint32_t Pack_DASH_CONTROLS_RIGHT_CMD_pacmod12(DASH_CONTROLS_RIGHT_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < DASH_CONTROLS_RIGHT_CMD_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  _d[1] |= (_m->DASH_CONTROLS_BUTTON & (0xFFU));

  *_len = DASH_CONTROLS_RIGHT_CMD_DLC;
  *_ide = DASH_CONTROLS_RIGHT_CMD_IDE;
  return DASH_CONTROLS_RIGHT_CMD_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_HAZARD_LIGHTS_CMD_pacmod12(HAZARD_LIGHTS_CMD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLE = (_d[0] & (0x01U));
  _m->IGNORE_OVERRIDES = ((_d[0] >> 1) & (0x01U));
  _m->CLEAR_OVERRIDE = ((_d[0] >> 2) & (0x01U));
  _m->HAZARD_LIGHTS_CMD = (_d[1] & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < HAZARD_LIGHTS_CMD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_HAZARD_LIGHTS_CMD_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return HAZARD_LIGHTS_CMD_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_HAZARD_LIGHTS_CMD_pacmod12(HAZARD_LIGHTS_CMD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < HAZARD_LIGHTS_CMD_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  cframe->Data[1] |= (_m->HAZARD_LIGHTS_CMD & (0x01U));

  cframe->MsgId = HAZARD_LIGHTS_CMD_CANID;
  cframe->DLC = HAZARD_LIGHTS_CMD_DLC;
  cframe->IDE = HAZARD_LIGHTS_CMD_IDE;
  return HAZARD_LIGHTS_CMD_CANID;
}

#else

uint32_t Pack_HAZARD_LIGHTS_CMD_pacmod12(HAZARD_LIGHTS_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < HAZARD_LIGHTS_CMD_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  _d[1] |= (_m->HAZARD_LIGHTS_CMD & (0x01U));

  *_len = HAZARD_LIGHTS_CMD_DLC;
  *_ide = HAZARD_LIGHTS_CMD_IDE;
  return HAZARD_LIGHTS_CMD_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_HEADLIGHT_CMD_pacmod12(HEADLIGHT_CMD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLE = (_d[0] & (0x01U));
  _m->IGNORE_OVERRIDES = ((_d[0] >> 1) & (0x01U));
  _m->CLEAR_OVERRIDE = ((_d[0] >> 2) & (0x01U));
  _m->HEADLIGHT_CMD = (_d[1] & (0xFFU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < HEADLIGHT_CMD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_HEADLIGHT_CMD_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return HEADLIGHT_CMD_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_HEADLIGHT_CMD_pacmod12(HEADLIGHT_CMD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < HEADLIGHT_CMD_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  cframe->Data[1] |= (_m->HEADLIGHT_CMD & (0xFFU));

  cframe->MsgId = HEADLIGHT_CMD_CANID;
  cframe->DLC = HEADLIGHT_CMD_DLC;
  cframe->IDE = HEADLIGHT_CMD_IDE;
  return HEADLIGHT_CMD_CANID;
}

#else

uint32_t Pack_HEADLIGHT_CMD_pacmod12(HEADLIGHT_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < HEADLIGHT_CMD_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  _d[1] |= (_m->HEADLIGHT_CMD & (0xFFU));

  *_len = HEADLIGHT_CMD_DLC;
  *_ide = HEADLIGHT_CMD_IDE;
  return HEADLIGHT_CMD_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_HORN_CMD_pacmod12(HORN_CMD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLE = (_d[0] & (0x01U));
  _m->IGNORE_OVERRIDES = ((_d[0] >> 1) & (0x01U));
  _m->CLEAR_OVERRIDE = ((_d[0] >> 2) & (0x01U));
  _m->HORN_CMD = (_d[1] & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < HORN_CMD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_HORN_CMD_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return HORN_CMD_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_HORN_CMD_pacmod12(HORN_CMD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < HORN_CMD_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  cframe->Data[1] |= (_m->HORN_CMD & (0x01U));

  cframe->MsgId = HORN_CMD_CANID;
  cframe->DLC = HORN_CMD_DLC;
  cframe->IDE = HORN_CMD_IDE;
  return HORN_CMD_CANID;
}

#else

uint32_t Pack_HORN_CMD_pacmod12(HORN_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < HORN_CMD_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  _d[1] |= (_m->HORN_CMD & (0x01U));

  *_len = HORN_CMD_DLC;
  *_ide = HORN_CMD_IDE;
  return HORN_CMD_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_MEDIA_CONTROLS_CMD_pacmod12(MEDIA_CONTROLS_CMD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLE = (_d[0] & (0x01U));
  _m->IGNORE_OVERRIDES = ((_d[0] >> 1) & (0x01U));
  _m->CLEAR_OVERRIDE = ((_d[0] >> 2) & (0x01U));
  _m->MEDIA_CONTROLS_CMD = (_d[1] & (0xFFU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < MEDIA_CONTROLS_CMD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_MEDIA_CONTROLS_CMD_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return MEDIA_CONTROLS_CMD_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_MEDIA_CONTROLS_CMD_pacmod12(MEDIA_CONTROLS_CMD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < MEDIA_CONTROLS_CMD_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  cframe->Data[1] |= (_m->MEDIA_CONTROLS_CMD & (0xFFU));

  cframe->MsgId = MEDIA_CONTROLS_CMD_CANID;
  cframe->DLC = MEDIA_CONTROLS_CMD_DLC;
  cframe->IDE = MEDIA_CONTROLS_CMD_IDE;
  return MEDIA_CONTROLS_CMD_CANID;
}

#else

uint32_t Pack_MEDIA_CONTROLS_CMD_pacmod12(MEDIA_CONTROLS_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < MEDIA_CONTROLS_CMD_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  _d[1] |= (_m->MEDIA_CONTROLS_CMD & (0xFFU));

  *_len = MEDIA_CONTROLS_CMD_DLC;
  *_ide = MEDIA_CONTROLS_CMD_IDE;
  return MEDIA_CONTROLS_CMD_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_PARKING_BRAKE_CMD_pacmod12(PARKING_BRAKE_CMD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLE = (_d[0] & (0x01U));
  _m->IGNORE_OVERRIDES = ((_d[0] >> 1) & (0x01U));
  _m->CLEAR_OVERRIDE = ((_d[0] >> 2) & (0x01U));
  _m->PARKING_BRAKE_CMD = (_d[1] & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < PARKING_BRAKE_CMD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_PARKING_BRAKE_CMD_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return PARKING_BRAKE_CMD_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_PARKING_BRAKE_CMD_pacmod12(PARKING_BRAKE_CMD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < PARKING_BRAKE_CMD_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  cframe->Data[1] |= (_m->PARKING_BRAKE_CMD & (0x01U));

  cframe->MsgId = PARKING_BRAKE_CMD_CANID;
  cframe->DLC = PARKING_BRAKE_CMD_DLC;
  cframe->IDE = PARKING_BRAKE_CMD_IDE;
  return PARKING_BRAKE_CMD_CANID;
}

#else

uint32_t Pack_PARKING_BRAKE_CMD_pacmod12(PARKING_BRAKE_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < PARKING_BRAKE_CMD_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  _d[1] |= (_m->PARKING_BRAKE_CMD & (0x01U));

  *_len = PARKING_BRAKE_CMD_DLC;
  *_ide = PARKING_BRAKE_CMD_IDE;
  return PARKING_BRAKE_CMD_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_SHIFT_CMD_pacmod12(SHIFT_CMD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLE = (_d[0] & (0x01U));
  _m->IGNORE_OVERRIDES = ((_d[0] >> 1) & (0x01U));
  _m->CLEAR_OVERRIDE = ((_d[0] >> 2) & (0x01U));
  _m->SHIFT_CMD = (_d[1] & (0xFFU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < SHIFT_CMD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_SHIFT_CMD_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return SHIFT_CMD_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_SHIFT_CMD_pacmod12(SHIFT_CMD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < SHIFT_CMD_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  cframe->Data[1] |= (_m->SHIFT_CMD & (0xFFU));

  cframe->MsgId = SHIFT_CMD_CANID;
  cframe->DLC = SHIFT_CMD_DLC;
  cframe->IDE = SHIFT_CMD_IDE;
  return SHIFT_CMD_CANID;
}

#else

uint32_t Pack_SHIFT_CMD_pacmod12(SHIFT_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < SHIFT_CMD_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  _d[1] |= (_m->SHIFT_CMD & (0xFFU));

  *_len = SHIFT_CMD_DLC;
  *_ide = SHIFT_CMD_IDE;
  return SHIFT_CMD_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_STEERING_CMD_pacmod12(STEERING_CMD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLE = (_d[0] & (0x01U));
  _m->IGNORE_OVERRIDES = ((_d[0] >> 1) & (0x01U));
  _m->CLEAR_OVERRIDE = ((_d[0] >> 2) & (0x01U));
  _m->POSITION_ro = ((_d[1] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->POSITION_phys = (sigfloat_t)(PACMOD12_POSITION_ro_fromS(_m->POSITION_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->ROTATION_RATE_ro = ((_d[3] & (0xFFU)) << 8) | (_d[4] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->ROTATION_RATE_phys = (sigfloat_t)(PACMOD12_ROTATION_RATE_ro_fromS(_m->ROTATION_RATE_ro));
#endif // PACMOD12_USE_SIGFLOAT

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < STEERING_CMD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_STEERING_CMD_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return STEERING_CMD_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_STEERING_CMD_pacmod12(STEERING_CMD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < STEERING_CMD_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->POSITION_ro = PACMOD12_POSITION_ro_toS(_m->POSITION_phys);
  _m->ROTATION_RATE_ro = PACMOD12_ROTATION_RATE_ro_toS(_m->ROTATION_RATE_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  cframe->Data[1] |= ((_m->POSITION_ro >> 8) & (0xFFU));
  cframe->Data[2] |= (_m->POSITION_ro & (0xFFU));
  cframe->Data[3] |= ((_m->ROTATION_RATE_ro >> 8) & (0xFFU));
  cframe->Data[4] |= (_m->ROTATION_RATE_ro & (0xFFU));

  cframe->MsgId = STEERING_CMD_CANID;
  cframe->DLC = STEERING_CMD_DLC;
  cframe->IDE = STEERING_CMD_IDE;
  return STEERING_CMD_CANID;
}

#else

uint32_t Pack_STEERING_CMD_pacmod12(STEERING_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < STEERING_CMD_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->POSITION_ro = PACMOD12_POSITION_ro_toS(_m->POSITION_phys);
  _m->ROTATION_RATE_ro = PACMOD12_ROTATION_RATE_ro_toS(_m->ROTATION_RATE_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  _d[1] |= ((_m->POSITION_ro >> 8) & (0xFFU));
  _d[2] |= (_m->POSITION_ro & (0xFFU));
  _d[3] |= ((_m->ROTATION_RATE_ro >> 8) & (0xFFU));
  _d[4] |= (_m->ROTATION_RATE_ro & (0xFFU));

  *_len = STEERING_CMD_DLC;
  *_ide = STEERING_CMD_IDE;
  return STEERING_CMD_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_TURN_CMD_pacmod12(TURN_CMD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLE = (_d[0] & (0x01U));
  _m->IGNORE_OVERRIDES = ((_d[0] >> 1) & (0x01U));
  _m->CLEAR_OVERRIDE = ((_d[0] >> 2) & (0x01U));
  _m->TURN_SIGNAL_CMD = (_d[1] & (0xFFU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < TURN_CMD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_TURN_CMD_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return TURN_CMD_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_TURN_CMD_pacmod12(TURN_CMD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < TURN_CMD_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  cframe->Data[1] |= (_m->TURN_SIGNAL_CMD & (0xFFU));

  cframe->MsgId = TURN_CMD_CANID;
  cframe->DLC = TURN_CMD_DLC;
  cframe->IDE = TURN_CMD_IDE;
  return TURN_CMD_CANID;
}

#else

uint32_t Pack_TURN_CMD_pacmod12(TURN_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < TURN_CMD_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  _d[1] |= (_m->TURN_SIGNAL_CMD & (0xFFU));

  *_len = TURN_CMD_DLC;
  *_ide = TURN_CMD_IDE;
  return TURN_CMD_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_WIPER_CMD_pacmod12(WIPER_CMD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLE = (_d[0] & (0x01U));
  _m->IGNORE_OVERRIDES = ((_d[0] >> 1) & (0x01U));
  _m->CLEAR_OVERRIDE = ((_d[0] >> 2) & (0x01U));
  _m->WIPER_CMD = (_d[1] & (0xFFU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < WIPER_CMD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_WIPER_CMD_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return WIPER_CMD_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_WIPER_CMD_pacmod12(WIPER_CMD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < WIPER_CMD_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  cframe->Data[1] |= (_m->WIPER_CMD & (0xFFU));

  cframe->MsgId = WIPER_CMD_CANID;
  cframe->DLC = WIPER_CMD_DLC;
  cframe->IDE = WIPER_CMD_IDE;
  return WIPER_CMD_CANID;
}

#else

uint32_t Pack_WIPER_CMD_pacmod12(WIPER_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < WIPER_CMD_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  _d[1] |= (_m->WIPER_CMD & (0xFFU));

  *_len = WIPER_CMD_DLC;
  *_ide = WIPER_CMD_IDE;
  return WIPER_CMD_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_SPRAYER_CMD_pacmod12(SPRAYER_CMD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLE = (_d[0] & (0x01U));
  _m->IGNORE_OVERRIDES = ((_d[0] >> 1) & (0x01U));
  _m->CLEAR_OVERRIDE = ((_d[0] >> 2) & (0x01U));
  _m->SPRAYER_CMD = (_d[1] & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < SPRAYER_CMD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_SPRAYER_CMD_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return SPRAYER_CMD_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_SPRAYER_CMD_pacmod12(SPRAYER_CMD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < SPRAYER_CMD_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  cframe->Data[1] |= (_m->SPRAYER_CMD & (0x01U));

  cframe->MsgId = SPRAYER_CMD_CANID;
  cframe->DLC = SPRAYER_CMD_DLC;
  cframe->IDE = SPRAYER_CMD_IDE;
  return SPRAYER_CMD_CANID;
}

#else

uint32_t Pack_SPRAYER_CMD_pacmod12(SPRAYER_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < SPRAYER_CMD_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  _d[1] |= (_m->SPRAYER_CMD & (0x01U));

  *_len = SPRAYER_CMD_DLC;
  *_ide = SPRAYER_CMD_IDE;
  return SPRAYER_CMD_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_BRAKE_DECEL_CMD_pacmod12(BRAKE_DECEL_CMD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLE = (_d[0] & (0x01U));
  _m->IGNORE_OVERRIDES = ((_d[0] >> 1) & (0x01U));
  _m->CLEAR_OVERRIDE = ((_d[0] >> 2) & (0x01U));
  _m->BRAKE_DECEL_CMD_ro = ((_d[1] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->BRAKE_DECEL_CMD_phys = (sigfloat_t)(PACMOD12_BRAKE_DECEL_CMD_ro_fromS(_m->BRAKE_DECEL_CMD_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->XBR_EBI_MODE = (_d[3] & (0x03U));
  _m->XBR_PRIORITY = ((_d[3] >> 2) & (0x03U));
  _m->XBR_CONTROL_MODE = ((_d[3] >> 4) & (0x03U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < BRAKE_DECEL_CMD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_BRAKE_DECEL_CMD_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return BRAKE_DECEL_CMD_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_BRAKE_DECEL_CMD_pacmod12(BRAKE_DECEL_CMD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < BRAKE_DECEL_CMD_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->BRAKE_DECEL_CMD_ro = PACMOD12_BRAKE_DECEL_CMD_ro_toS(_m->BRAKE_DECEL_CMD_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  cframe->Data[1] |= ((_m->BRAKE_DECEL_CMD_ro >> 8) & (0xFFU));
  cframe->Data[2] |= (_m->BRAKE_DECEL_CMD_ro & (0xFFU));
  cframe->Data[3] |= (_m->XBR_EBI_MODE & (0x03U)) | ((_m->XBR_PRIORITY & (0x03U)) << 2) | ((_m->XBR_CONTROL_MODE & (0x03U)) << 4);

  cframe->MsgId = BRAKE_DECEL_CMD_CANID;
  cframe->DLC = BRAKE_DECEL_CMD_DLC;
  cframe->IDE = BRAKE_DECEL_CMD_IDE;
  return BRAKE_DECEL_CMD_CANID;
}

#else

uint32_t Pack_BRAKE_DECEL_CMD_pacmod12(BRAKE_DECEL_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < BRAKE_DECEL_CMD_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->BRAKE_DECEL_CMD_ro = PACMOD12_BRAKE_DECEL_CMD_ro_toS(_m->BRAKE_DECEL_CMD_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  _d[1] |= ((_m->BRAKE_DECEL_CMD_ro >> 8) & (0xFFU));
  _d[2] |= (_m->BRAKE_DECEL_CMD_ro & (0xFFU));
  _d[3] |= (_m->XBR_EBI_MODE & (0x03U)) | ((_m->XBR_PRIORITY & (0x03U)) << 2) | ((_m->XBR_CONTROL_MODE & (0x03U)) << 4);

  *_len = BRAKE_DECEL_CMD_DLC;
  *_ide = BRAKE_DECEL_CMD_IDE;
  return BRAKE_DECEL_CMD_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_REAR_PASS_DOOR_CMD_pacmod12(REAR_PASS_DOOR_CMD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLE = (_d[0] & (0x01U));
  _m->IGNORE_OVERRIDES = ((_d[0] >> 1) & (0x01U));
  _m->CLEAR_OVERRIDE = ((_d[0] >> 2) & (0x01U));
  _m->REAR_PASS_DOOR_CMD = (_d[1] & (0xFFU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < REAR_PASS_DOOR_CMD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_REAR_PASS_DOOR_CMD_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return REAR_PASS_DOOR_CMD_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_REAR_PASS_DOOR_CMD_pacmod12(REAR_PASS_DOOR_CMD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < REAR_PASS_DOOR_CMD_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  cframe->Data[1] |= (_m->REAR_PASS_DOOR_CMD & (0xFFU));

  cframe->MsgId = REAR_PASS_DOOR_CMD_CANID;
  cframe->DLC = REAR_PASS_DOOR_CMD_DLC;
  cframe->IDE = REAR_PASS_DOOR_CMD_IDE;
  return REAR_PASS_DOOR_CMD_CANID;
}

#else

uint32_t Pack_REAR_PASS_DOOR_CMD_pacmod12(REAR_PASS_DOOR_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < REAR_PASS_DOOR_CMD_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  _d[1] |= (_m->REAR_PASS_DOOR_CMD & (0xFFU));

  *_len = REAR_PASS_DOOR_CMD_DLC;
  *_ide = REAR_PASS_DOOR_CMD_IDE;
  return REAR_PASS_DOOR_CMD_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_ENGINE_BRAKE_CMD_pacmod12(ENGINE_BRAKE_CMD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLE = (_d[0] & (0x01U));
  _m->IGNORE_OVERRIDES = ((_d[0] >> 1) & (0x01U));
  _m->CLEAR_OVERRIDE = ((_d[0] >> 2) & (0x01U));
  _m->ENGINE_BRAKE_CMD = (_d[1] & (0xFFU));
  _m->AUTO_CMD = (_d[2] & (0x03U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < ENGINE_BRAKE_CMD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_ENGINE_BRAKE_CMD_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return ENGINE_BRAKE_CMD_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_ENGINE_BRAKE_CMD_pacmod12(ENGINE_BRAKE_CMD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < ENGINE_BRAKE_CMD_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  cframe->Data[1] |= (_m->ENGINE_BRAKE_CMD & (0xFFU));
  cframe->Data[2] |= (_m->AUTO_CMD & (0x03U));

  cframe->MsgId = ENGINE_BRAKE_CMD_CANID;
  cframe->DLC = ENGINE_BRAKE_CMD_DLC;
  cframe->IDE = ENGINE_BRAKE_CMD_IDE;
  return ENGINE_BRAKE_CMD_CANID;
}

#else

uint32_t Pack_ENGINE_BRAKE_CMD_pacmod12(ENGINE_BRAKE_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < ENGINE_BRAKE_CMD_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  _d[1] |= (_m->ENGINE_BRAKE_CMD & (0xFFU));
  _d[2] |= (_m->AUTO_CMD & (0x03U));

  *_len = ENGINE_BRAKE_CMD_DLC;
  *_ide = ENGINE_BRAKE_CMD_IDE;
  return ENGINE_BRAKE_CMD_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_EXHAUST_BRAKE_CMD_pacmod12(EXHAUST_BRAKE_CMD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLE = (_d[0] & (0x01U));
  _m->IGNORE_OVERRIDES = ((_d[0] >> 1) & (0x01U));
  _m->CLEAR_OVERRIDE = ((_d[0] >> 2) & (0x01U));
  _m->EXHAUST_BRAKE_CMD = (_d[1] & (0xFFU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < EXHAUST_BRAKE_CMD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_EXHAUST_BRAKE_CMD_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return EXHAUST_BRAKE_CMD_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_EXHAUST_BRAKE_CMD_pacmod12(EXHAUST_BRAKE_CMD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < EXHAUST_BRAKE_CMD_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  cframe->Data[1] |= (_m->EXHAUST_BRAKE_CMD & (0xFFU));

  cframe->MsgId = EXHAUST_BRAKE_CMD_CANID;
  cframe->DLC = EXHAUST_BRAKE_CMD_DLC;
  cframe->IDE = EXHAUST_BRAKE_CMD_IDE;
  return EXHAUST_BRAKE_CMD_CANID;
}

#else

uint32_t Pack_EXHAUST_BRAKE_CMD_pacmod12(EXHAUST_BRAKE_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < EXHAUST_BRAKE_CMD_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  _d[1] |= (_m->EXHAUST_BRAKE_CMD & (0xFFU));

  *_len = EXHAUST_BRAKE_CMD_DLC;
  *_ide = EXHAUST_BRAKE_CMD_IDE;
  return EXHAUST_BRAKE_CMD_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_MARKER_LAMP_CMD_pacmod12(MARKER_LAMP_CMD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLE = (_d[0] & (0x01U));
  _m->IGNORE_OVERRIDES = ((_d[0] >> 1) & (0x01U));
  _m->CLEAR_OVERRIDE = ((_d[0] >> 2) & (0x01U));
  _m->MARKER_LAMP_CMD = (_d[1] & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < MARKER_LAMP_CMD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_MARKER_LAMP_CMD_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return MARKER_LAMP_CMD_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_MARKER_LAMP_CMD_pacmod12(MARKER_LAMP_CMD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < MARKER_LAMP_CMD_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  cframe->Data[1] |= (_m->MARKER_LAMP_CMD & (0x01U));

  cframe->MsgId = MARKER_LAMP_CMD_CANID;
  cframe->DLC = MARKER_LAMP_CMD_DLC;
  cframe->IDE = MARKER_LAMP_CMD_IDE;
  return MARKER_LAMP_CMD_CANID;
}

#else

uint32_t Pack_MARKER_LAMP_CMD_pacmod12(MARKER_LAMP_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < MARKER_LAMP_CMD_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  _d[1] |= (_m->MARKER_LAMP_CMD & (0x01U));

  *_len = MARKER_LAMP_CMD_DLC;
  *_ide = MARKER_LAMP_CMD_IDE;
  return MARKER_LAMP_CMD_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_CABIN_TEMP_CMD_pacmod12(CABIN_TEMP_CMD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLE = (_d[0] & (0x01U));
  _m->IGNORE_OVERRIDES = ((_d[0] >> 1) & (0x01U));
  _m->CLEAR_OVERRIDE = ((_d[0] >> 2) & (0x01U));
  _m->CABIN_TEMP_CMD_ro = (_d[1] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->CABIN_TEMP_CMD_phys = (sigfloat_t)(PACMOD12_CABIN_TEMP_CMD_ro_fromS(_m->CABIN_TEMP_CMD_ro));
#endif // PACMOD12_USE_SIGFLOAT

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CABIN_TEMP_CMD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CABIN_TEMP_CMD_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return CABIN_TEMP_CMD_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_CABIN_TEMP_CMD_pacmod12(CABIN_TEMP_CMD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < CABIN_TEMP_CMD_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->CABIN_TEMP_CMD_ro = PACMOD12_CABIN_TEMP_CMD_ro_toS(_m->CABIN_TEMP_CMD_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  cframe->Data[1] |= (_m->CABIN_TEMP_CMD_ro & (0xFFU));

  cframe->MsgId = CABIN_TEMP_CMD_CANID;
  cframe->DLC = CABIN_TEMP_CMD_DLC;
  cframe->IDE = CABIN_TEMP_CMD_IDE;
  return CABIN_TEMP_CMD_CANID;
}

#else

uint32_t Pack_CABIN_TEMP_CMD_pacmod12(CABIN_TEMP_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < CABIN_TEMP_CMD_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->CABIN_TEMP_CMD_ro = PACMOD12_CABIN_TEMP_CMD_ro_toS(_m->CABIN_TEMP_CMD_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  _d[1] |= (_m->CABIN_TEMP_CMD_ro & (0xFFU));

  *_len = CABIN_TEMP_CMD_DLC;
  *_ide = CABIN_TEMP_CMD_IDE;
  return CABIN_TEMP_CMD_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_CABIN_FAN_SPEED_CMD_pacmod12(CABIN_FAN_SPEED_CMD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLE = (_d[0] & (0x01U));
  _m->IGNORE_OVERRIDES = ((_d[0] >> 1) & (0x01U));
  _m->CLEAR_OVERRIDE = ((_d[0] >> 2) & (0x01U));
  _m->CABIN_FAN_SPEED_CMD = (_d[1] & (0xFFU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CABIN_FAN_SPEED_CMD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CABIN_FAN_SPEED_CMD_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return CABIN_FAN_SPEED_CMD_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_CABIN_FAN_SPEED_CMD_pacmod12(CABIN_FAN_SPEED_CMD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < CABIN_FAN_SPEED_CMD_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  cframe->Data[1] |= (_m->CABIN_FAN_SPEED_CMD & (0xFFU));

  cframe->MsgId = CABIN_FAN_SPEED_CMD_CANID;
  cframe->DLC = CABIN_FAN_SPEED_CMD_DLC;
  cframe->IDE = CABIN_FAN_SPEED_CMD_IDE;
  return CABIN_FAN_SPEED_CMD_CANID;
}

#else

uint32_t Pack_CABIN_FAN_SPEED_CMD_pacmod12(CABIN_FAN_SPEED_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < CABIN_FAN_SPEED_CMD_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  _d[1] |= (_m->CABIN_FAN_SPEED_CMD & (0xFFU));

  *_len = CABIN_FAN_SPEED_CMD_DLC;
  *_ide = CABIN_FAN_SPEED_CMD_IDE;
  return CABIN_FAN_SPEED_CMD_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_CABIN_CLIMATE_CMD_pacmod12(CABIN_CLIMATE_CMD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLE = (_d[0] & (0x01U));
  _m->IGNORE_OVERRIDES = ((_d[0] >> 1) & (0x01U));
  _m->CLEAR_OVERRIDE = ((_d[0] >> 2) & (0x01U));
  _m->CMD_AC_OFF_ON = (_d[1] & (0x03U));
  _m->CMD_MAX_AC_OFF_ON = ((_d[1] >> 2) & (0x03U));
  _m->CMD_DEFROST_OFF_ON = ((_d[1] >> 4) & (0x03U));
  _m->CMD_MAX_DEFROST_OFF_ON = ((_d[1] >> 6) & (0x03U));
  _m->CMD_DIR_UP_OFF_ON = (_d[2] & (0x03U));
  _m->CMD_DIR_DOWN_OFF_ON = ((_d[2] >> 2) & (0x03U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CABIN_CLIMATE_CMD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CABIN_CLIMATE_CMD_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return CABIN_CLIMATE_CMD_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_CABIN_CLIMATE_CMD_pacmod12(CABIN_CLIMATE_CMD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < CABIN_CLIMATE_CMD_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  cframe->Data[1] |= (_m->CMD_AC_OFF_ON & (0x03U)) | ((_m->CMD_MAX_AC_OFF_ON & (0x03U)) << 2) | ((_m->CMD_DEFROST_OFF_ON & (0x03U)) << 4) | ((_m->CMD_MAX_DEFROST_OFF_ON & (0x03U)) << 6);
  cframe->Data[2] |= (_m->CMD_DIR_UP_OFF_ON & (0x03U)) | ((_m->CMD_DIR_DOWN_OFF_ON & (0x03U)) << 2);

  cframe->MsgId = CABIN_CLIMATE_CMD_CANID;
  cframe->DLC = CABIN_CLIMATE_CMD_DLC;
  cframe->IDE = CABIN_CLIMATE_CMD_IDE;
  return CABIN_CLIMATE_CMD_CANID;
}

#else

uint32_t Pack_CABIN_CLIMATE_CMD_pacmod12(CABIN_CLIMATE_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < CABIN_CLIMATE_CMD_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLE & (0x01U)) | ((_m->IGNORE_OVERRIDES & (0x01U)) << 1) | ((_m->CLEAR_OVERRIDE & (0x01U)) << 2);
  _d[1] |= (_m->CMD_AC_OFF_ON & (0x03U)) | ((_m->CMD_MAX_AC_OFF_ON & (0x03U)) << 2) | ((_m->CMD_DEFROST_OFF_ON & (0x03U)) << 4) | ((_m->CMD_MAX_DEFROST_OFF_ON & (0x03U)) << 6);
  _d[2] |= (_m->CMD_DIR_UP_OFF_ON & (0x03U)) | ((_m->CMD_DIR_DOWN_OFF_ON & (0x03U)) << 2);

  *_len = CABIN_CLIMATE_CMD_DLC;
  *_ide = CABIN_CLIMATE_CMD_IDE;
  return CABIN_CLIMATE_CMD_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_ACCEL_RPT_pacmod12(ACCEL_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLED = (_d[0] & (0x01U));
  _m->OVERRIDE_ACTIVE = ((_d[0] >> 1) & (0x01U));
  _m->COMMAND_OUTPUT_FAULT = ((_d[0] >> 2) & (0x01U));
  _m->INPUT_OUTPUT_FAULT = ((_d[0] >> 3) & (0x01U));
  _m->OUTPUT_REPORTED_FAULT = ((_d[0] >> 4) & (0x01U));
  _m->PACMOD_FAULT = ((_d[0] >> 5) & (0x01U));
  _m->VEHICLE_FAULT = ((_d[0] >> 6) & (0x01U));
  _m->COMMAND_TIMEOUT = ((_d[0] >> 7) & (0x01U));
  _m->MANUAL_INPUT_ro = ((_d[1] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->MANUAL_INPUT_phys = (sigfloat_t)(PACMOD12_MANUAL_INPUT_ro_fromS(_m->MANUAL_INPUT_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->COMMANDED_VALUE_ro = ((_d[3] & (0xFFU)) << 8) | (_d[4] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->COMMANDED_VALUE_phys = (sigfloat_t)(PACMOD12_COMMANDED_VALUE_ro_fromS(_m->COMMANDED_VALUE_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->OUTPUT_VALUE_ro = ((_d[5] & (0xFFU)) << 8) | (_d[6] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->OUTPUT_VALUE_phys = (sigfloat_t)(PACMOD12_OUTPUT_VALUE_ro_fromS(_m->OUTPUT_VALUE_ro));
#endif // PACMOD12_USE_SIGFLOAT

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < ACCEL_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_ACCEL_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return ACCEL_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_ACCEL_RPT_pacmod12(ACCEL_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < ACCEL_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->MANUAL_INPUT_ro = PACMOD12_MANUAL_INPUT_ro_toS(_m->MANUAL_INPUT_phys);
  _m->COMMANDED_VALUE_ro = PACMOD12_COMMANDED_VALUE_ro_toS(_m->COMMANDED_VALUE_phys);
  _m->OUTPUT_VALUE_ro = PACMOD12_OUTPUT_VALUE_ro_toS(_m->OUTPUT_VALUE_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  cframe->Data[1] |= ((_m->MANUAL_INPUT_ro >> 8) & (0xFFU));
  cframe->Data[2] |= (_m->MANUAL_INPUT_ro & (0xFFU));
  cframe->Data[3] |= ((_m->COMMANDED_VALUE_ro >> 8) & (0xFFU));
  cframe->Data[4] |= (_m->COMMANDED_VALUE_ro & (0xFFU));
  cframe->Data[5] |= ((_m->OUTPUT_VALUE_ro >> 8) & (0xFFU));
  cframe->Data[6] |= (_m->OUTPUT_VALUE_ro & (0xFFU));

  cframe->MsgId = ACCEL_RPT_CANID;
  cframe->DLC = ACCEL_RPT_DLC;
  cframe->IDE = ACCEL_RPT_IDE;
  return ACCEL_RPT_CANID;
}

#else

uint32_t Pack_ACCEL_RPT_pacmod12(ACCEL_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < ACCEL_RPT_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->MANUAL_INPUT_ro = PACMOD12_MANUAL_INPUT_ro_toS(_m->MANUAL_INPUT_phys);
  _m->COMMANDED_VALUE_ro = PACMOD12_COMMANDED_VALUE_ro_toS(_m->COMMANDED_VALUE_phys);
  _m->OUTPUT_VALUE_ro = PACMOD12_OUTPUT_VALUE_ro_toS(_m->OUTPUT_VALUE_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  _d[1] |= ((_m->MANUAL_INPUT_ro >> 8) & (0xFFU));
  _d[2] |= (_m->MANUAL_INPUT_ro & (0xFFU));
  _d[3] |= ((_m->COMMANDED_VALUE_ro >> 8) & (0xFFU));
  _d[4] |= (_m->COMMANDED_VALUE_ro & (0xFFU));
  _d[5] |= ((_m->OUTPUT_VALUE_ro >> 8) & (0xFFU));
  _d[6] |= (_m->OUTPUT_VALUE_ro & (0xFFU));

  *_len = ACCEL_RPT_DLC;
  *_ide = ACCEL_RPT_IDE;
  return ACCEL_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_ACCEL_CMD_LIMIT_RPT_pacmod12(ACCEL_CMD_LIMIT_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ACCEL_CMD_LIMIT_ro = ((_d[0] & (0xFFU)) << 8) | (_d[1] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->ACCEL_CMD_LIMIT_phys = (sigfloat_t)(PACMOD12_ACCEL_CMD_LIMIT_ro_fromS(_m->ACCEL_CMD_LIMIT_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->LIMITED_ACCEL_CMD_ro = ((_d[2] & (0xFFU)) << 8) | (_d[3] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->LIMITED_ACCEL_CMD_phys = (sigfloat_t)(PACMOD12_LIMITED_ACCEL_CMD_ro_fromS(_m->LIMITED_ACCEL_CMD_ro));
#endif // PACMOD12_USE_SIGFLOAT

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < ACCEL_CMD_LIMIT_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_ACCEL_CMD_LIMIT_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return ACCEL_CMD_LIMIT_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_ACCEL_CMD_LIMIT_RPT_pacmod12(ACCEL_CMD_LIMIT_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < ACCEL_CMD_LIMIT_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->ACCEL_CMD_LIMIT_ro = PACMOD12_ACCEL_CMD_LIMIT_ro_toS(_m->ACCEL_CMD_LIMIT_phys);
  _m->LIMITED_ACCEL_CMD_ro = PACMOD12_LIMITED_ACCEL_CMD_ro_toS(_m->LIMITED_ACCEL_CMD_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[0] |= ((_m->ACCEL_CMD_LIMIT_ro >> 8) & (0xFFU));
  cframe->Data[1] |= (_m->ACCEL_CMD_LIMIT_ro & (0xFFU));
  cframe->Data[2] |= ((_m->LIMITED_ACCEL_CMD_ro >> 8) & (0xFFU));
  cframe->Data[3] |= (_m->LIMITED_ACCEL_CMD_ro & (0xFFU));

  cframe->MsgId = ACCEL_CMD_LIMIT_RPT_CANID;
  cframe->DLC = ACCEL_CMD_LIMIT_RPT_DLC;
  cframe->IDE = ACCEL_CMD_LIMIT_RPT_IDE;
  return ACCEL_CMD_LIMIT_RPT_CANID;
}

#else

uint32_t Pack_ACCEL_CMD_LIMIT_RPT_pacmod12(ACCEL_CMD_LIMIT_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < ACCEL_CMD_LIMIT_RPT_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->ACCEL_CMD_LIMIT_ro = PACMOD12_ACCEL_CMD_LIMIT_ro_toS(_m->ACCEL_CMD_LIMIT_phys);
  _m->LIMITED_ACCEL_CMD_ro = PACMOD12_LIMITED_ACCEL_CMD_ro_toS(_m->LIMITED_ACCEL_CMD_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[0] |= ((_m->ACCEL_CMD_LIMIT_ro >> 8) & (0xFFU));
  _d[1] |= (_m->ACCEL_CMD_LIMIT_ro & (0xFFU));
  _d[2] |= ((_m->LIMITED_ACCEL_CMD_ro >> 8) & (0xFFU));
  _d[3] |= (_m->LIMITED_ACCEL_CMD_ro & (0xFFU));

  *_len = ACCEL_CMD_LIMIT_RPT_DLC;
  *_ide = ACCEL_CMD_LIMIT_RPT_IDE;
  return ACCEL_CMD_LIMIT_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_BRAKE_RPT_pacmod12(BRAKE_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLED = (_d[0] & (0x01U));
  _m->OVERRIDE_ACTIVE = ((_d[0] >> 1) & (0x01U));
  _m->COMMAND_OUTPUT_FAULT = ((_d[0] >> 2) & (0x01U));
  _m->INPUT_OUTPUT_FAULT = ((_d[0] >> 3) & (0x01U));
  _m->OUTPUT_REPORTED_FAULT = ((_d[0] >> 4) & (0x01U));
  _m->PACMOD_FAULT = ((_d[0] >> 5) & (0x01U));
  _m->VEHICLE_FAULT = ((_d[0] >> 6) & (0x01U));
  _m->COMMAND_TIMEOUT = ((_d[0] >> 7) & (0x01U));
  _m->MANUAL_INPUT_ro = ((_d[1] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->MANUAL_INPUT_phys = (sigfloat_t)(PACMOD12_MANUAL_INPUT_ro_fromS(_m->MANUAL_INPUT_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->COMMANDED_VALUE_ro = ((_d[3] & (0xFFU)) << 8) | (_d[4] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->COMMANDED_VALUE_phys = (sigfloat_t)(PACMOD12_COMMANDED_VALUE_ro_fromS(_m->COMMANDED_VALUE_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->OUTPUT_VALUE_ro = ((_d[5] & (0xFFU)) << 8) | (_d[6] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->OUTPUT_VALUE_phys = (sigfloat_t)(PACMOD12_OUTPUT_VALUE_ro_fromS(_m->OUTPUT_VALUE_ro));
#endif // PACMOD12_USE_SIGFLOAT

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < BRAKE_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_BRAKE_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return BRAKE_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_BRAKE_RPT_pacmod12(BRAKE_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < BRAKE_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->MANUAL_INPUT_ro = PACMOD12_MANUAL_INPUT_ro_toS(_m->MANUAL_INPUT_phys);
  _m->COMMANDED_VALUE_ro = PACMOD12_COMMANDED_VALUE_ro_toS(_m->COMMANDED_VALUE_phys);
  _m->OUTPUT_VALUE_ro = PACMOD12_OUTPUT_VALUE_ro_toS(_m->OUTPUT_VALUE_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  cframe->Data[1] |= ((_m->MANUAL_INPUT_ro >> 8) & (0xFFU));
  cframe->Data[2] |= (_m->MANUAL_INPUT_ro & (0xFFU));
  cframe->Data[3] |= ((_m->COMMANDED_VALUE_ro >> 8) & (0xFFU));
  cframe->Data[4] |= (_m->COMMANDED_VALUE_ro & (0xFFU));
  cframe->Data[5] |= ((_m->OUTPUT_VALUE_ro >> 8) & (0xFFU));
  cframe->Data[6] |= (_m->OUTPUT_VALUE_ro & (0xFFU));

  cframe->MsgId = BRAKE_RPT_CANID;
  cframe->DLC = BRAKE_RPT_DLC;
  cframe->IDE = BRAKE_RPT_IDE;
  return BRAKE_RPT_CANID;
}

#else

uint32_t Pack_BRAKE_RPT_pacmod12(BRAKE_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < BRAKE_RPT_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->MANUAL_INPUT_ro = PACMOD12_MANUAL_INPUT_ro_toS(_m->MANUAL_INPUT_phys);
  _m->COMMANDED_VALUE_ro = PACMOD12_COMMANDED_VALUE_ro_toS(_m->COMMANDED_VALUE_phys);
  _m->OUTPUT_VALUE_ro = PACMOD12_OUTPUT_VALUE_ro_toS(_m->OUTPUT_VALUE_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  _d[1] |= ((_m->MANUAL_INPUT_ro >> 8) & (0xFFU));
  _d[2] |= (_m->MANUAL_INPUT_ro & (0xFFU));
  _d[3] |= ((_m->COMMANDED_VALUE_ro >> 8) & (0xFFU));
  _d[4] |= (_m->COMMANDED_VALUE_ro & (0xFFU));
  _d[5] |= ((_m->OUTPUT_VALUE_ro >> 8) & (0xFFU));
  _d[6] |= (_m->OUTPUT_VALUE_ro & (0xFFU));

  *_len = BRAKE_RPT_DLC;
  *_ide = BRAKE_RPT_IDE;
  return BRAKE_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_BRAKE_CMD_LIMIT_RPT_pacmod12(BRAKE_CMD_LIMIT_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->BRAKE_CMD_LIMIT_ro = ((_d[0] & (0xFFU)) << 8) | (_d[1] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->BRAKE_CMD_LIMIT_phys = (sigfloat_t)(PACMOD12_BRAKE_CMD_LIMIT_ro_fromS(_m->BRAKE_CMD_LIMIT_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->LIMITED_BRAKE_CMD_ro = ((_d[2] & (0xFFU)) << 8) | (_d[3] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->LIMITED_BRAKE_CMD_phys = (sigfloat_t)(PACMOD12_LIMITED_BRAKE_CMD_ro_fromS(_m->LIMITED_BRAKE_CMD_ro));
#endif // PACMOD12_USE_SIGFLOAT

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < BRAKE_CMD_LIMIT_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_BRAKE_CMD_LIMIT_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return BRAKE_CMD_LIMIT_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_BRAKE_CMD_LIMIT_RPT_pacmod12(BRAKE_CMD_LIMIT_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < BRAKE_CMD_LIMIT_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->BRAKE_CMD_LIMIT_ro = PACMOD12_BRAKE_CMD_LIMIT_ro_toS(_m->BRAKE_CMD_LIMIT_phys);
  _m->LIMITED_BRAKE_CMD_ro = PACMOD12_LIMITED_BRAKE_CMD_ro_toS(_m->LIMITED_BRAKE_CMD_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[0] |= ((_m->BRAKE_CMD_LIMIT_ro >> 8) & (0xFFU));
  cframe->Data[1] |= (_m->BRAKE_CMD_LIMIT_ro & (0xFFU));
  cframe->Data[2] |= ((_m->LIMITED_BRAKE_CMD_ro >> 8) & (0xFFU));
  cframe->Data[3] |= (_m->LIMITED_BRAKE_CMD_ro & (0xFFU));

  cframe->MsgId = BRAKE_CMD_LIMIT_RPT_CANID;
  cframe->DLC = BRAKE_CMD_LIMIT_RPT_DLC;
  cframe->IDE = BRAKE_CMD_LIMIT_RPT_IDE;
  return BRAKE_CMD_LIMIT_RPT_CANID;
}

#else

uint32_t Pack_BRAKE_CMD_LIMIT_RPT_pacmod12(BRAKE_CMD_LIMIT_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < BRAKE_CMD_LIMIT_RPT_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->BRAKE_CMD_LIMIT_ro = PACMOD12_BRAKE_CMD_LIMIT_ro_toS(_m->BRAKE_CMD_LIMIT_phys);
  _m->LIMITED_BRAKE_CMD_ro = PACMOD12_LIMITED_BRAKE_CMD_ro_toS(_m->LIMITED_BRAKE_CMD_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[0] |= ((_m->BRAKE_CMD_LIMIT_ro >> 8) & (0xFFU));
  _d[1] |= (_m->BRAKE_CMD_LIMIT_ro & (0xFFU));
  _d[2] |= ((_m->LIMITED_BRAKE_CMD_ro >> 8) & (0xFFU));
  _d[3] |= (_m->LIMITED_BRAKE_CMD_ro & (0xFFU));

  *_len = BRAKE_CMD_LIMIT_RPT_DLC;
  *_ide = BRAKE_CMD_LIMIT_RPT_IDE;
  return BRAKE_CMD_LIMIT_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_CRUISE_CONTROL_BUTTONS_RPT_pacmod12(CRUISE_CONTROL_BUTTONS_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLED = (_d[0] & (0x01U));
  _m->OVERRIDE_ACTIVE = ((_d[0] >> 1) & (0x01U));
  _m->COMMAND_OUTPUT_FAULT = ((_d[0] >> 2) & (0x01U));
  _m->INPUT_OUTPUT_FAULT = ((_d[0] >> 3) & (0x01U));
  _m->OUTPUT_REPORTED_FAULT = ((_d[0] >> 4) & (0x01U));
  _m->PACMOD_FAULT = ((_d[0] >> 5) & (0x01U));
  _m->VEHICLE_FAULT = ((_d[0] >> 6) & (0x01U));
  _m->COMMAND_TIMEOUT = ((_d[0] >> 7) & (0x01U));
  _m->MANUAL_INPUT = (_d[1] & (0xFFU));
  _m->COMMANDED_VALUE = (_d[2] & (0xFFU));
  _m->OUTPUT_VALUE = (_d[3] & (0xFFU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CRUISE_CONTROL_BUTTONS_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CRUISE_CONTROL_BUTTONS_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return CRUISE_CONTROL_BUTTONS_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_CRUISE_CONTROL_BUTTONS_RPT_pacmod12(CRUISE_CONTROL_BUTTONS_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < CRUISE_CONTROL_BUTTONS_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  cframe->Data[1] |= (_m->MANUAL_INPUT & (0xFFU));
  cframe->Data[2] |= (_m->COMMANDED_VALUE & (0xFFU));
  cframe->Data[3] |= (_m->OUTPUT_VALUE & (0xFFU));

  cframe->MsgId = CRUISE_CONTROL_BUTTONS_RPT_CANID;
  cframe->DLC = CRUISE_CONTROL_BUTTONS_RPT_DLC;
  cframe->IDE = CRUISE_CONTROL_BUTTONS_RPT_IDE;
  return CRUISE_CONTROL_BUTTONS_RPT_CANID;
}

#else

uint32_t Pack_CRUISE_CONTROL_BUTTONS_RPT_pacmod12(CRUISE_CONTROL_BUTTONS_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < CRUISE_CONTROL_BUTTONS_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  _d[1] |= (_m->MANUAL_INPUT & (0xFFU));
  _d[2] |= (_m->COMMANDED_VALUE & (0xFFU));
  _d[3] |= (_m->OUTPUT_VALUE & (0xFFU));

  *_len = CRUISE_CONTROL_BUTTONS_RPT_DLC;
  *_ide = CRUISE_CONTROL_BUTTONS_RPT_IDE;
  return CRUISE_CONTROL_BUTTONS_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_DASH_CONTROLS_LEFT_RPT_pacmod12(DASH_CONTROLS_LEFT_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLED = (_d[0] & (0x01U));
  _m->OVERRIDE_ACTIVE = ((_d[0] >> 1) & (0x01U));
  _m->COMMAND_OUTPUT_FAULT = ((_d[0] >> 2) & (0x01U));
  _m->INPUT_OUTPUT_FAULT = ((_d[0] >> 3) & (0x01U));
  _m->OUTPUT_REPORTED_FAULT = ((_d[0] >> 4) & (0x01U));
  _m->PACMOD_FAULT = ((_d[0] >> 5) & (0x01U));
  _m->VEHICLE_FAULT = ((_d[0] >> 6) & (0x01U));
  _m->COMMAND_TIMEOUT = ((_d[0] >> 7) & (0x01U));
  _m->MANUAL_INPUT = (_d[1] & (0xFFU));
  _m->COMMANDED_VALUE = (_d[2] & (0xFFU));
  _m->OUTPUT_VALUE = (_d[3] & (0xFFU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < DASH_CONTROLS_LEFT_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_DASH_CONTROLS_LEFT_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return DASH_CONTROLS_LEFT_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_DASH_CONTROLS_LEFT_RPT_pacmod12(DASH_CONTROLS_LEFT_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < DASH_CONTROLS_LEFT_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  cframe->Data[1] |= (_m->MANUAL_INPUT & (0xFFU));
  cframe->Data[2] |= (_m->COMMANDED_VALUE & (0xFFU));
  cframe->Data[3] |= (_m->OUTPUT_VALUE & (0xFFU));

  cframe->MsgId = DASH_CONTROLS_LEFT_RPT_CANID;
  cframe->DLC = DASH_CONTROLS_LEFT_RPT_DLC;
  cframe->IDE = DASH_CONTROLS_LEFT_RPT_IDE;
  return DASH_CONTROLS_LEFT_RPT_CANID;
}

#else

uint32_t Pack_DASH_CONTROLS_LEFT_RPT_pacmod12(DASH_CONTROLS_LEFT_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < DASH_CONTROLS_LEFT_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  _d[1] |= (_m->MANUAL_INPUT & (0xFFU));
  _d[2] |= (_m->COMMANDED_VALUE & (0xFFU));
  _d[3] |= (_m->OUTPUT_VALUE & (0xFFU));

  *_len = DASH_CONTROLS_LEFT_RPT_DLC;
  *_ide = DASH_CONTROLS_LEFT_RPT_IDE;
  return DASH_CONTROLS_LEFT_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_DASH_CONTROLS_RIGHT_RPT_pacmod12(DASH_CONTROLS_RIGHT_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLED = (_d[0] & (0x01U));
  _m->OVERRIDE_ACTIVE = ((_d[0] >> 1) & (0x01U));
  _m->COMMAND_OUTPUT_FAULT = ((_d[0] >> 2) & (0x01U));
  _m->INPUT_OUTPUT_FAULT = ((_d[0] >> 3) & (0x01U));
  _m->OUTPUT_REPORTED_FAULT = ((_d[0] >> 4) & (0x01U));
  _m->PACMOD_FAULT = ((_d[0] >> 5) & (0x01U));
  _m->VEHICLE_FAULT = ((_d[0] >> 6) & (0x01U));
  _m->COMMAND_TIMEOUT = ((_d[0] >> 7) & (0x01U));
  _m->MANUAL_INPUT = (_d[1] & (0xFFU));
  _m->COMMANDED_VALUE = (_d[2] & (0xFFU));
  _m->OUTPUT_VALUE = (_d[3] & (0xFFU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < DASH_CONTROLS_RIGHT_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_DASH_CONTROLS_RIGHT_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return DASH_CONTROLS_RIGHT_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_DASH_CONTROLS_RIGHT_RPT_pacmod12(DASH_CONTROLS_RIGHT_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < DASH_CONTROLS_RIGHT_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  cframe->Data[1] |= (_m->MANUAL_INPUT & (0xFFU));
  cframe->Data[2] |= (_m->COMMANDED_VALUE & (0xFFU));
  cframe->Data[3] |= (_m->OUTPUT_VALUE & (0xFFU));

  cframe->MsgId = DASH_CONTROLS_RIGHT_RPT_CANID;
  cframe->DLC = DASH_CONTROLS_RIGHT_RPT_DLC;
  cframe->IDE = DASH_CONTROLS_RIGHT_RPT_IDE;
  return DASH_CONTROLS_RIGHT_RPT_CANID;
}

#else

uint32_t Pack_DASH_CONTROLS_RIGHT_RPT_pacmod12(DASH_CONTROLS_RIGHT_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < DASH_CONTROLS_RIGHT_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  _d[1] |= (_m->MANUAL_INPUT & (0xFFU));
  _d[2] |= (_m->COMMANDED_VALUE & (0xFFU));
  _d[3] |= (_m->OUTPUT_VALUE & (0xFFU));

  *_len = DASH_CONTROLS_RIGHT_RPT_DLC;
  *_ide = DASH_CONTROLS_RIGHT_RPT_IDE;
  return DASH_CONTROLS_RIGHT_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_HAZARD_LIGHTS_RPT_pacmod12(HAZARD_LIGHTS_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLED = (_d[0] & (0x01U));
  _m->OVERRIDE_ACTIVE = ((_d[0] >> 1) & (0x01U));
  _m->COMMAND_OUTPUT_FAULT = ((_d[0] >> 2) & (0x01U));
  _m->INPUT_OUTPUT_FAULT = ((_d[0] >> 3) & (0x01U));
  _m->OUTPUT_REPORTED_FAULT = ((_d[0] >> 4) & (0x01U));
  _m->PACMOD_FAULT = ((_d[0] >> 5) & (0x01U));
  _m->VEHICLE_FAULT = ((_d[0] >> 6) & (0x01U));
  _m->COMMAND_TIMEOUT = ((_d[0] >> 7) & (0x01U));
  _m->MANUAL_INPUT = (_d[1] & (0x01U));
  _m->COMMANDED_VALUE = (_d[2] & (0x01U));
  _m->OUTPUT_VALUE = (_d[3] & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < HAZARD_LIGHTS_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_HAZARD_LIGHTS_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return HAZARD_LIGHTS_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_HAZARD_LIGHTS_RPT_pacmod12(HAZARD_LIGHTS_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < HAZARD_LIGHTS_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  cframe->Data[1] |= (_m->MANUAL_INPUT & (0x01U));
  cframe->Data[2] |= (_m->COMMANDED_VALUE & (0x01U));
  cframe->Data[3] |= (_m->OUTPUT_VALUE & (0x01U));

  cframe->MsgId = HAZARD_LIGHTS_RPT_CANID;
  cframe->DLC = HAZARD_LIGHTS_RPT_DLC;
  cframe->IDE = HAZARD_LIGHTS_RPT_IDE;
  return HAZARD_LIGHTS_RPT_CANID;
}

#else

uint32_t Pack_HAZARD_LIGHTS_RPT_pacmod12(HAZARD_LIGHTS_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < HAZARD_LIGHTS_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  _d[1] |= (_m->MANUAL_INPUT & (0x01U));
  _d[2] |= (_m->COMMANDED_VALUE & (0x01U));
  _d[3] |= (_m->OUTPUT_VALUE & (0x01U));

  *_len = HAZARD_LIGHTS_RPT_DLC;
  *_ide = HAZARD_LIGHTS_RPT_IDE;
  return HAZARD_LIGHTS_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_HEADLIGHT_RPT_pacmod12(HEADLIGHT_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLED = (_d[0] & (0x01U));
  _m->OVERRIDE_ACTIVE = ((_d[0] >> 1) & (0x01U));
  _m->COMMAND_OUTPUT_FAULT = ((_d[0] >> 2) & (0x01U));
  _m->INPUT_OUTPUT_FAULT = ((_d[0] >> 3) & (0x01U));
  _m->OUTPUT_REPORTED_FAULT = ((_d[0] >> 4) & (0x01U));
  _m->PACMOD_FAULT = ((_d[0] >> 5) & (0x01U));
  _m->VEHICLE_FAULT = ((_d[0] >> 6) & (0x01U));
  _m->COMMAND_TIMEOUT = ((_d[0] >> 7) & (0x01U));
  _m->MANUAL_INPUT = (_d[1] & (0xFFU));
  _m->COMMANDED_VALUE = (_d[2] & (0xFFU));
  _m->OUTPUT_VALUE = (_d[3] & (0xFFU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < HEADLIGHT_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_HEADLIGHT_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return HEADLIGHT_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_HEADLIGHT_RPT_pacmod12(HEADLIGHT_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < HEADLIGHT_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  cframe->Data[1] |= (_m->MANUAL_INPUT & (0xFFU));
  cframe->Data[2] |= (_m->COMMANDED_VALUE & (0xFFU));
  cframe->Data[3] |= (_m->OUTPUT_VALUE & (0xFFU));

  cframe->MsgId = HEADLIGHT_RPT_CANID;
  cframe->DLC = HEADLIGHT_RPT_DLC;
  cframe->IDE = HEADLIGHT_RPT_IDE;
  return HEADLIGHT_RPT_CANID;
}

#else

uint32_t Pack_HEADLIGHT_RPT_pacmod12(HEADLIGHT_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < HEADLIGHT_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  _d[1] |= (_m->MANUAL_INPUT & (0xFFU));
  _d[2] |= (_m->COMMANDED_VALUE & (0xFFU));
  _d[3] |= (_m->OUTPUT_VALUE & (0xFFU));

  *_len = HEADLIGHT_RPT_DLC;
  *_ide = HEADLIGHT_RPT_IDE;
  return HEADLIGHT_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_HORN_RPT_pacmod12(HORN_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLED = (_d[0] & (0x01U));
  _m->OVERRIDE_ACTIVE = ((_d[0] >> 1) & (0x01U));
  _m->COMMAND_OUTPUT_FAULT = ((_d[0] >> 2) & (0x01U));
  _m->INPUT_OUTPUT_FAULT = ((_d[0] >> 3) & (0x01U));
  _m->OUTPUT_REPORTED_FAULT = ((_d[0] >> 4) & (0x01U));
  _m->PACMOD_FAULT = ((_d[0] >> 5) & (0x01U));
  _m->VEHICLE_FAULT = ((_d[0] >> 6) & (0x01U));
  _m->COMMAND_TIMEOUT = ((_d[0] >> 7) & (0x01U));
  _m->MANUAL_INPUT = (_d[1] & (0xFFU));
  _m->COMMANDED_VALUE = (_d[2] & (0xFFU));
  _m->OUTPUT_VALUE = (_d[3] & (0xFFU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < HORN_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_HORN_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return HORN_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_HORN_RPT_pacmod12(HORN_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < HORN_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  cframe->Data[1] |= (_m->MANUAL_INPUT & (0xFFU));
  cframe->Data[2] |= (_m->COMMANDED_VALUE & (0xFFU));
  cframe->Data[3] |= (_m->OUTPUT_VALUE & (0xFFU));

  cframe->MsgId = HORN_RPT_CANID;
  cframe->DLC = HORN_RPT_DLC;
  cframe->IDE = HORN_RPT_IDE;
  return HORN_RPT_CANID;
}

#else

uint32_t Pack_HORN_RPT_pacmod12(HORN_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < HORN_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  _d[1] |= (_m->MANUAL_INPUT & (0xFFU));
  _d[2] |= (_m->COMMANDED_VALUE & (0xFFU));
  _d[3] |= (_m->OUTPUT_VALUE & (0xFFU));

  *_len = HORN_RPT_DLC;
  *_ide = HORN_RPT_IDE;
  return HORN_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_MEDIA_CONTROLS_RPT_pacmod12(MEDIA_CONTROLS_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLED = (_d[0] & (0x01U));
  _m->OVERRIDE_ACTIVE = ((_d[0] >> 1) & (0x01U));
  _m->COMMAND_OUTPUT_FAULT = ((_d[0] >> 2) & (0x01U));
  _m->INPUT_OUTPUT_FAULT = ((_d[0] >> 3) & (0x01U));
  _m->OUTPUT_REPORTED_FAULT = ((_d[0] >> 4) & (0x01U));
  _m->PACMOD_FAULT = ((_d[0] >> 5) & (0x01U));
  _m->VEHICLE_FAULT = ((_d[0] >> 6) & (0x01U));
  _m->COMMAND_TIMEOUT = ((_d[0] >> 7) & (0x01U));
  _m->MANUAL_INPUT = (_d[1] & (0xFFU));
  _m->COMMANDED_VALUE = (_d[2] & (0xFFU));
  _m->OUTPUT_VALUE = (_d[3] & (0xFFU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < MEDIA_CONTROLS_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_MEDIA_CONTROLS_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return MEDIA_CONTROLS_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_MEDIA_CONTROLS_RPT_pacmod12(MEDIA_CONTROLS_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < MEDIA_CONTROLS_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  cframe->Data[1] |= (_m->MANUAL_INPUT & (0xFFU));
  cframe->Data[2] |= (_m->COMMANDED_VALUE & (0xFFU));
  cframe->Data[3] |= (_m->OUTPUT_VALUE & (0xFFU));

  cframe->MsgId = MEDIA_CONTROLS_RPT_CANID;
  cframe->DLC = MEDIA_CONTROLS_RPT_DLC;
  cframe->IDE = MEDIA_CONTROLS_RPT_IDE;
  return MEDIA_CONTROLS_RPT_CANID;
}

#else

uint32_t Pack_MEDIA_CONTROLS_RPT_pacmod12(MEDIA_CONTROLS_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < MEDIA_CONTROLS_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  _d[1] |= (_m->MANUAL_INPUT & (0xFFU));
  _d[2] |= (_m->COMMANDED_VALUE & (0xFFU));
  _d[3] |= (_m->OUTPUT_VALUE & (0xFFU));

  *_len = MEDIA_CONTROLS_RPT_DLC;
  *_ide = MEDIA_CONTROLS_RPT_IDE;
  return MEDIA_CONTROLS_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_PARKING_BRAKE_RPT_pacmod12(PARKING_BRAKE_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLED = (_d[0] & (0x01U));
  _m->OVERRIDE_ACTIVE = ((_d[0] >> 1) & (0x01U));
  _m->COMMAND_OUTPUT_FAULT = ((_d[0] >> 2) & (0x01U));
  _m->INPUT_OUTPUT_FAULT = ((_d[0] >> 3) & (0x01U));
  _m->OUTPUT_REPORTED_FAULT = ((_d[0] >> 4) & (0x01U));
  _m->PACMOD_FAULT = ((_d[0] >> 5) & (0x01U));
  _m->VEHICLE_FAULT = ((_d[0] >> 6) & (0x01U));
  _m->COMMAND_TIMEOUT = ((_d[0] >> 7) & (0x01U));
  _m->MANUAL_INPUT = (_d[1] & (0x01U));
  _m->COMMANDED_VALUE = (_d[2] & (0x01U));
  _m->OUTPUT_VALUE = (_d[3] & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < PARKING_BRAKE_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_PARKING_BRAKE_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return PARKING_BRAKE_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_PARKING_BRAKE_RPT_pacmod12(PARKING_BRAKE_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < PARKING_BRAKE_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  cframe->Data[1] |= (_m->MANUAL_INPUT & (0x01U));
  cframe->Data[2] |= (_m->COMMANDED_VALUE & (0x01U));
  cframe->Data[3] |= (_m->OUTPUT_VALUE & (0x01U));

  cframe->MsgId = PARKING_BRAKE_RPT_CANID;
  cframe->DLC = PARKING_BRAKE_RPT_DLC;
  cframe->IDE = PARKING_BRAKE_RPT_IDE;
  return PARKING_BRAKE_RPT_CANID;
}

#else

uint32_t Pack_PARKING_BRAKE_RPT_pacmod12(PARKING_BRAKE_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < PARKING_BRAKE_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  _d[1] |= (_m->MANUAL_INPUT & (0x01U));
  _d[2] |= (_m->COMMANDED_VALUE & (0x01U));
  _d[3] |= (_m->OUTPUT_VALUE & (0x01U));

  *_len = PARKING_BRAKE_RPT_DLC;
  *_ide = PARKING_BRAKE_RPT_IDE;
  return PARKING_BRAKE_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_SHIFT_RPT_pacmod12(SHIFT_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLED = (_d[0] & (0x01U));
  _m->OVERRIDE_ACTIVE = ((_d[0] >> 1) & (0x01U));
  _m->COMMAND_OUTPUT_FAULT = ((_d[0] >> 2) & (0x01U));
  _m->INPUT_OUTPUT_FAULT = ((_d[0] >> 3) & (0x01U));
  _m->OUTPUT_REPORTED_FAULT = ((_d[0] >> 4) & (0x01U));
  _m->PACMOD_FAULT = ((_d[0] >> 5) & (0x01U));
  _m->VEHICLE_FAULT = ((_d[0] >> 6) & (0x01U));
  _m->COMMAND_TIMEOUT = ((_d[0] >> 7) & (0x01U));
  _m->MANUAL_INPUT = (_d[1] & (0xFFU));
  _m->COMMANDED_VALUE = (_d[2] & (0xFFU));
  _m->OUTPUT_VALUE = (_d[3] & (0xFFU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < SHIFT_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_SHIFT_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return SHIFT_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_SHIFT_RPT_pacmod12(SHIFT_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < SHIFT_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  cframe->Data[1] |= (_m->MANUAL_INPUT & (0xFFU));
  cframe->Data[2] |= (_m->COMMANDED_VALUE & (0xFFU));
  cframe->Data[3] |= (_m->OUTPUT_VALUE & (0xFFU));

  cframe->MsgId = SHIFT_RPT_CANID;
  cframe->DLC = SHIFT_RPT_DLC;
  cframe->IDE = SHIFT_RPT_IDE;
  return SHIFT_RPT_CANID;
}

#else

uint32_t Pack_SHIFT_RPT_pacmod12(SHIFT_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < SHIFT_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  _d[1] |= (_m->MANUAL_INPUT & (0xFFU));
  _d[2] |= (_m->COMMANDED_VALUE & (0xFFU));
  _d[3] |= (_m->OUTPUT_VALUE & (0xFFU));

  *_len = SHIFT_RPT_DLC;
  *_ide = SHIFT_RPT_IDE;
  return SHIFT_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_STEERING_RPT_pacmod12(STEERING_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLED = (_d[0] & (0x01U));
  _m->OVERRIDE_ACTIVE = ((_d[0] >> 1) & (0x01U));
  _m->COMMAND_OUTPUT_FAULT = ((_d[0] >> 2) & (0x01U));
  _m->INPUT_OUTPUT_FAULT = ((_d[0] >> 3) & (0x01U));
  _m->OUTPUT_REPORTED_FAULT = ((_d[0] >> 4) & (0x01U));
  _m->PACMOD_FAULT = ((_d[0] >> 5) & (0x01U));
  _m->VEHICLE_FAULT = ((_d[0] >> 6) & (0x01U));
  _m->COMMAND_TIMEOUT = ((_d[0] >> 7) & (0x01U));
  _m->MANUAL_INPUT_ro = ((_d[1] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->MANUAL_INPUT_phys = (sigfloat_t)(PACMOD12_MANUAL_INPUT_ro_fromS(_m->MANUAL_INPUT_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->COMMANDED_VALUE_ro = ((_d[3] & (0xFFU)) << 8) | (_d[4] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->COMMANDED_VALUE_phys = (sigfloat_t)(PACMOD12_COMMANDED_VALUE_ro_fromS(_m->COMMANDED_VALUE_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->OUTPUT_VALUE_ro = ((_d[5] & (0xFFU)) << 8) | (_d[6] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->OUTPUT_VALUE_phys = (sigfloat_t)(PACMOD12_OUTPUT_VALUE_ro_fromS(_m->OUTPUT_VALUE_ro));
#endif // PACMOD12_USE_SIGFLOAT

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < STEERING_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_STEERING_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return STEERING_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_STEERING_RPT_pacmod12(STEERING_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < STEERING_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->MANUAL_INPUT_ro = PACMOD12_MANUAL_INPUT_ro_toS(_m->MANUAL_INPUT_phys);
  _m->COMMANDED_VALUE_ro = PACMOD12_COMMANDED_VALUE_ro_toS(_m->COMMANDED_VALUE_phys);
  _m->OUTPUT_VALUE_ro = PACMOD12_OUTPUT_VALUE_ro_toS(_m->OUTPUT_VALUE_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  cframe->Data[1] |= ((_m->MANUAL_INPUT_ro >> 8) & (0xFFU));
  cframe->Data[2] |= (_m->MANUAL_INPUT_ro & (0xFFU));
  cframe->Data[3] |= ((_m->COMMANDED_VALUE_ro >> 8) & (0xFFU));
  cframe->Data[4] |= (_m->COMMANDED_VALUE_ro & (0xFFU));
  cframe->Data[5] |= ((_m->OUTPUT_VALUE_ro >> 8) & (0xFFU));
  cframe->Data[6] |= (_m->OUTPUT_VALUE_ro & (0xFFU));

  cframe->MsgId = STEERING_RPT_CANID;
  cframe->DLC = STEERING_RPT_DLC;
  cframe->IDE = STEERING_RPT_IDE;
  return STEERING_RPT_CANID;
}

#else

uint32_t Pack_STEERING_RPT_pacmod12(STEERING_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < STEERING_RPT_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->MANUAL_INPUT_ro = PACMOD12_MANUAL_INPUT_ro_toS(_m->MANUAL_INPUT_phys);
  _m->COMMANDED_VALUE_ro = PACMOD12_COMMANDED_VALUE_ro_toS(_m->COMMANDED_VALUE_phys);
  _m->OUTPUT_VALUE_ro = PACMOD12_OUTPUT_VALUE_ro_toS(_m->OUTPUT_VALUE_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  _d[1] |= ((_m->MANUAL_INPUT_ro >> 8) & (0xFFU));
  _d[2] |= (_m->MANUAL_INPUT_ro & (0xFFU));
  _d[3] |= ((_m->COMMANDED_VALUE_ro >> 8) & (0xFFU));
  _d[4] |= (_m->COMMANDED_VALUE_ro & (0xFFU));
  _d[5] |= ((_m->OUTPUT_VALUE_ro >> 8) & (0xFFU));
  _d[6] |= (_m->OUTPUT_VALUE_ro & (0xFFU));

  *_len = STEERING_RPT_DLC;
  *_ide = STEERING_RPT_IDE;
  return STEERING_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_STEERING_CMD_LIMIT_RPT_pacmod12(STEERING_CMD_LIMIT_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->POSITION_CMD_LIMIT_ro = ((_d[0] & (0xFFU)) << 8) | (_d[1] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->POSITION_CMD_LIMIT_phys = (sigfloat_t)(PACMOD12_POSITION_CMD_LIMIT_ro_fromS(_m->POSITION_CMD_LIMIT_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->LIMITED_POSITION_CMD_ro = ((_d[2] & (0xFFU)) << 8) | (_d[3] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->LIMITED_POSITION_CMD_phys = (sigfloat_t)(PACMOD12_LIMITED_POSITION_CMD_ro_fromS(_m->LIMITED_POSITION_CMD_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->ROTATION_RATE_CMD_LIMIT_ro = ((_d[4] & (0xFFU)) << 8) | (_d[5] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->ROTATION_RATE_CMD_LIMIT_phys = (sigfloat_t)(PACMOD12_ROTATION_RATE_CMD_LIMIT_ro_fromS(_m->ROTATION_RATE_CMD_LIMIT_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->LIMITED_ROTATION_RATE_ro = ((_d[6] & (0xFFU)) << 8) | (_d[7] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->LIMITED_ROTATION_RATE_phys = (sigfloat_t)(PACMOD12_LIMITED_ROTATION_RATE_ro_fromS(_m->LIMITED_ROTATION_RATE_ro));
#endif // PACMOD12_USE_SIGFLOAT

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < STEERING_CMD_LIMIT_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_STEERING_CMD_LIMIT_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return STEERING_CMD_LIMIT_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_STEERING_CMD_LIMIT_RPT_pacmod12(STEERING_CMD_LIMIT_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < STEERING_CMD_LIMIT_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->POSITION_CMD_LIMIT_ro = PACMOD12_POSITION_CMD_LIMIT_ro_toS(_m->POSITION_CMD_LIMIT_phys);
  _m->LIMITED_POSITION_CMD_ro = PACMOD12_LIMITED_POSITION_CMD_ro_toS(_m->LIMITED_POSITION_CMD_phys);
  _m->ROTATION_RATE_CMD_LIMIT_ro = PACMOD12_ROTATION_RATE_CMD_LIMIT_ro_toS(_m->ROTATION_RATE_CMD_LIMIT_phys);
  _m->LIMITED_ROTATION_RATE_ro = PACMOD12_LIMITED_ROTATION_RATE_ro_toS(_m->LIMITED_ROTATION_RATE_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[0] |= ((_m->POSITION_CMD_LIMIT_ro >> 8) & (0xFFU));
  cframe->Data[1] |= (_m->POSITION_CMD_LIMIT_ro & (0xFFU));
  cframe->Data[2] |= ((_m->LIMITED_POSITION_CMD_ro >> 8) & (0xFFU));
  cframe->Data[3] |= (_m->LIMITED_POSITION_CMD_ro & (0xFFU));
  cframe->Data[4] |= ((_m->ROTATION_RATE_CMD_LIMIT_ro >> 8) & (0xFFU));
  cframe->Data[5] |= (_m->ROTATION_RATE_CMD_LIMIT_ro & (0xFFU));
  cframe->Data[6] |= ((_m->LIMITED_ROTATION_RATE_ro >> 8) & (0xFFU));
  cframe->Data[7] |= (_m->LIMITED_ROTATION_RATE_ro & (0xFFU));

  cframe->MsgId = STEERING_CMD_LIMIT_RPT_CANID;
  cframe->DLC = STEERING_CMD_LIMIT_RPT_DLC;
  cframe->IDE = STEERING_CMD_LIMIT_RPT_IDE;
  return STEERING_CMD_LIMIT_RPT_CANID;
}

#else

uint32_t Pack_STEERING_CMD_LIMIT_RPT_pacmod12(STEERING_CMD_LIMIT_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < STEERING_CMD_LIMIT_RPT_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->POSITION_CMD_LIMIT_ro = PACMOD12_POSITION_CMD_LIMIT_ro_toS(_m->POSITION_CMD_LIMIT_phys);
  _m->LIMITED_POSITION_CMD_ro = PACMOD12_LIMITED_POSITION_CMD_ro_toS(_m->LIMITED_POSITION_CMD_phys);
  _m->ROTATION_RATE_CMD_LIMIT_ro = PACMOD12_ROTATION_RATE_CMD_LIMIT_ro_toS(_m->ROTATION_RATE_CMD_LIMIT_phys);
  _m->LIMITED_ROTATION_RATE_ro = PACMOD12_LIMITED_ROTATION_RATE_ro_toS(_m->LIMITED_ROTATION_RATE_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[0] |= ((_m->POSITION_CMD_LIMIT_ro >> 8) & (0xFFU));
  _d[1] |= (_m->POSITION_CMD_LIMIT_ro & (0xFFU));
  _d[2] |= ((_m->LIMITED_POSITION_CMD_ro >> 8) & (0xFFU));
  _d[3] |= (_m->LIMITED_POSITION_CMD_ro & (0xFFU));
  _d[4] |= ((_m->ROTATION_RATE_CMD_LIMIT_ro >> 8) & (0xFFU));
  _d[5] |= (_m->ROTATION_RATE_CMD_LIMIT_ro & (0xFFU));
  _d[6] |= ((_m->LIMITED_ROTATION_RATE_ro >> 8) & (0xFFU));
  _d[7] |= (_m->LIMITED_ROTATION_RATE_ro & (0xFFU));

  *_len = STEERING_CMD_LIMIT_RPT_DLC;
  *_ide = STEERING_CMD_LIMIT_RPT_IDE;
  return STEERING_CMD_LIMIT_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_TURN_RPT_pacmod12(TURN_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLED = (_d[0] & (0x01U));
  _m->OVERRIDE_ACTIVE = ((_d[0] >> 1) & (0x01U));
  _m->COMMAND_OUTPUT_FAULT = ((_d[0] >> 2) & (0x01U));
  _m->INPUT_OUTPUT_FAULT = ((_d[0] >> 3) & (0x01U));
  _m->OUTPUT_REPORTED_FAULT = ((_d[0] >> 4) & (0x01U));
  _m->PACMOD_FAULT = ((_d[0] >> 5) & (0x01U));
  _m->VEHICLE_FAULT = ((_d[0] >> 6) & (0x01U));
  _m->COMMAND_TIMEOUT = ((_d[0] >> 7) & (0x01U));
  _m->MANUAL_INPUT = (_d[1] & (0xFFU));
  _m->COMMANDED_VALUE = (_d[2] & (0xFFU));
  _m->OUTPUT_VALUE = (_d[3] & (0xFFU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < TURN_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_TURN_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return TURN_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_TURN_RPT_pacmod12(TURN_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < TURN_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  cframe->Data[1] |= (_m->MANUAL_INPUT & (0xFFU));
  cframe->Data[2] |= (_m->COMMANDED_VALUE & (0xFFU));
  cframe->Data[3] |= (_m->OUTPUT_VALUE & (0xFFU));

  cframe->MsgId = TURN_RPT_CANID;
  cframe->DLC = TURN_RPT_DLC;
  cframe->IDE = TURN_RPT_IDE;
  return TURN_RPT_CANID;
}

#else

uint32_t Pack_TURN_RPT_pacmod12(TURN_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < TURN_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  _d[1] |= (_m->MANUAL_INPUT & (0xFFU));
  _d[2] |= (_m->COMMANDED_VALUE & (0xFFU));
  _d[3] |= (_m->OUTPUT_VALUE & (0xFFU));

  *_len = TURN_RPT_DLC;
  *_ide = TURN_RPT_IDE;
  return TURN_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_WIPER_RPT_pacmod12(WIPER_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLED = (_d[0] & (0x01U));
  _m->OVERRIDE_ACTIVE = ((_d[0] >> 1) & (0x01U));
  _m->COMMAND_OUTPUT_FAULT = ((_d[0] >> 2) & (0x01U));
  _m->INPUT_OUTPUT_FAULT = ((_d[0] >> 3) & (0x01U));
  _m->OUTPUT_REPORTED_FAULT = ((_d[0] >> 4) & (0x01U));
  _m->PACMOD_FAULT = ((_d[0] >> 5) & (0x01U));
  _m->VEHICLE_FAULT = ((_d[0] >> 6) & (0x01U));
  _m->COMMAND_TIMEOUT = ((_d[0] >> 7) & (0x01U));
  _m->MANUAL_INPUT = (_d[1] & (0xFFU));
  _m->COMMANDED_VALUE = (_d[2] & (0xFFU));
  _m->OUTPUT_VALUE = (_d[3] & (0xFFU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < WIPER_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_WIPER_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return WIPER_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_WIPER_RPT_pacmod12(WIPER_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < WIPER_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  cframe->Data[1] |= (_m->MANUAL_INPUT & (0xFFU));
  cframe->Data[2] |= (_m->COMMANDED_VALUE & (0xFFU));
  cframe->Data[3] |= (_m->OUTPUT_VALUE & (0xFFU));

  cframe->MsgId = WIPER_RPT_CANID;
  cframe->DLC = WIPER_RPT_DLC;
  cframe->IDE = WIPER_RPT_IDE;
  return WIPER_RPT_CANID;
}

#else

uint32_t Pack_WIPER_RPT_pacmod12(WIPER_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < WIPER_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  _d[1] |= (_m->MANUAL_INPUT & (0xFFU));
  _d[2] |= (_m->COMMANDED_VALUE & (0xFFU));
  _d[3] |= (_m->OUTPUT_VALUE & (0xFFU));

  *_len = WIPER_RPT_DLC;
  *_ide = WIPER_RPT_IDE;
  return WIPER_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_SPRAYER_RPT_pacmod12(SPRAYER_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLED = (_d[0] & (0x01U));
  _m->OVERRIDE_ACTIVE = ((_d[0] >> 1) & (0x01U));
  _m->COMMAND_OUTPUT_FAULT = ((_d[0] >> 2) & (0x01U));
  _m->INPUT_OUTPUT_FAULT = ((_d[0] >> 3) & (0x01U));
  _m->OUTPUT_REPORTED_FAULT = ((_d[0] >> 4) & (0x01U));
  _m->PACMOD_FAULT = ((_d[0] >> 5) & (0x01U));
  _m->VEHICLE_FAULT = ((_d[0] >> 6) & (0x01U));
  _m->COMMAND_TIMEOUT = ((_d[0] >> 7) & (0x01U));
  _m->MANUAL_INPUT = (_d[1] & (0x01U));
  _m->COMMANDED_VALUE = (_d[2] & (0x01U));
  _m->OUTPUT_VALUE = (_d[3] & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < SPRAYER_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_SPRAYER_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return SPRAYER_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_SPRAYER_RPT_pacmod12(SPRAYER_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < SPRAYER_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  cframe->Data[1] |= (_m->MANUAL_INPUT & (0x01U));
  cframe->Data[2] |= (_m->COMMANDED_VALUE & (0x01U));
  cframe->Data[3] |= (_m->OUTPUT_VALUE & (0x01U));

  cframe->MsgId = SPRAYER_RPT_CANID;
  cframe->DLC = SPRAYER_RPT_DLC;
  cframe->IDE = SPRAYER_RPT_IDE;
  return SPRAYER_RPT_CANID;
}

#else

uint32_t Pack_SPRAYER_RPT_pacmod12(SPRAYER_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < SPRAYER_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  _d[1] |= (_m->MANUAL_INPUT & (0x01U));
  _d[2] |= (_m->COMMANDED_VALUE & (0x01U));
  _d[3] |= (_m->OUTPUT_VALUE & (0x01U));

  *_len = SPRAYER_RPT_DLC;
  *_ide = SPRAYER_RPT_IDE;
  return SPRAYER_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_BRAKE_DECEL_RPT_pacmod12(BRAKE_DECEL_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLED = (_d[0] & (0x01U));
  _m->OVERRIDE_ACTIVE = ((_d[0] >> 1) & (0x01U));
  _m->COMMAND_OUTPUT_FAULT = ((_d[0] >> 2) & (0x01U));
  _m->INPUT_OUTPUT_FAULT = ((_d[0] >> 3) & (0x01U));
  _m->OUTPUT_REPORTED_FAULT = ((_d[0] >> 4) & (0x01U));
  _m->PACMOD_FAULT = ((_d[0] >> 5) & (0x01U));
  _m->VEHICLE_FAULT = ((_d[0] >> 6) & (0x01U));
  _m->COMMAND_TIMEOUT = ((_d[0] >> 7) & (0x01U));
  _m->MANUAL_INPUT_ro = ((_d[1] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->MANUAL_INPUT_phys = (sigfloat_t)(PACMOD12_MANUAL_INPUT_ro_fromS(_m->MANUAL_INPUT_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->COMMANDED_VALUE_ro = ((_d[3] & (0xFFU)) << 8) | (_d[4] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->COMMANDED_VALUE_phys = (sigfloat_t)(PACMOD12_COMMANDED_VALUE_ro_fromS(_m->COMMANDED_VALUE_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->OUTPUT_VALUE_ro = ((_d[5] & (0xFFU)) << 8) | (_d[6] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->OUTPUT_VALUE_phys = (sigfloat_t)(PACMOD12_OUTPUT_VALUE_ro_fromS(_m->OUTPUT_VALUE_ro));
#endif // PACMOD12_USE_SIGFLOAT

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < BRAKE_DECEL_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_BRAKE_DECEL_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return BRAKE_DECEL_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_BRAKE_DECEL_RPT_pacmod12(BRAKE_DECEL_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < BRAKE_DECEL_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->MANUAL_INPUT_ro = PACMOD12_MANUAL_INPUT_ro_toS(_m->MANUAL_INPUT_phys);
  _m->COMMANDED_VALUE_ro = PACMOD12_COMMANDED_VALUE_ro_toS(_m->COMMANDED_VALUE_phys);
  _m->OUTPUT_VALUE_ro = PACMOD12_OUTPUT_VALUE_ro_toS(_m->OUTPUT_VALUE_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  cframe->Data[1] |= ((_m->MANUAL_INPUT_ro >> 8) & (0xFFU));
  cframe->Data[2] |= (_m->MANUAL_INPUT_ro & (0xFFU));
  cframe->Data[3] |= ((_m->COMMANDED_VALUE_ro >> 8) & (0xFFU));
  cframe->Data[4] |= (_m->COMMANDED_VALUE_ro & (0xFFU));
  cframe->Data[5] |= ((_m->OUTPUT_VALUE_ro >> 8) & (0xFFU));
  cframe->Data[6] |= (_m->OUTPUT_VALUE_ro & (0xFFU));

  cframe->MsgId = BRAKE_DECEL_RPT_CANID;
  cframe->DLC = BRAKE_DECEL_RPT_DLC;
  cframe->IDE = BRAKE_DECEL_RPT_IDE;
  return BRAKE_DECEL_RPT_CANID;
}

#else

uint32_t Pack_BRAKE_DECEL_RPT_pacmod12(BRAKE_DECEL_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < BRAKE_DECEL_RPT_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->MANUAL_INPUT_ro = PACMOD12_MANUAL_INPUT_ro_toS(_m->MANUAL_INPUT_phys);
  _m->COMMANDED_VALUE_ro = PACMOD12_COMMANDED_VALUE_ro_toS(_m->COMMANDED_VALUE_phys);
  _m->OUTPUT_VALUE_ro = PACMOD12_OUTPUT_VALUE_ro_toS(_m->OUTPUT_VALUE_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  _d[1] |= ((_m->MANUAL_INPUT_ro >> 8) & (0xFFU));
  _d[2] |= (_m->MANUAL_INPUT_ro & (0xFFU));
  _d[3] |= ((_m->COMMANDED_VALUE_ro >> 8) & (0xFFU));
  _d[4] |= (_m->COMMANDED_VALUE_ro & (0xFFU));
  _d[5] |= ((_m->OUTPUT_VALUE_ro >> 8) & (0xFFU));
  _d[6] |= (_m->OUTPUT_VALUE_ro & (0xFFU));

  *_len = BRAKE_DECEL_RPT_DLC;
  *_ide = BRAKE_DECEL_RPT_IDE;
  return BRAKE_DECEL_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_REAR_PASS_DOOR_RPT_pacmod12(REAR_PASS_DOOR_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLED = (_d[0] & (0x01U));
  _m->OVERRIDE_ACTIVE = ((_d[0] >> 1) & (0x01U));
  _m->COMMAND_OUTPUT_FAULT = ((_d[0] >> 2) & (0x01U));
  _m->INPUT_OUTPUT_FAULT = ((_d[0] >> 3) & (0x01U));
  _m->OUTPUT_REPORTED_FAULT = ((_d[0] >> 4) & (0x01U));
  _m->PACMOD_FAULT = ((_d[0] >> 5) & (0x01U));
  _m->VEHICLE_FAULT = ((_d[0] >> 6) & (0x01U));
  _m->COMMAND_TIMEOUT = ((_d[0] >> 7) & (0x01U));
  _m->MANUAL_INPUT = (_d[1] & (0xFFU));
  _m->COMMANDED_VALUE = (_d[2] & (0xFFU));
  _m->OUTPUT_VALUE = (_d[3] & (0xFFU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < REAR_PASS_DOOR_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_REAR_PASS_DOOR_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return REAR_PASS_DOOR_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_REAR_PASS_DOOR_RPT_pacmod12(REAR_PASS_DOOR_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < REAR_PASS_DOOR_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  cframe->Data[1] |= (_m->MANUAL_INPUT & (0xFFU));
  cframe->Data[2] |= (_m->COMMANDED_VALUE & (0xFFU));
  cframe->Data[3] |= (_m->OUTPUT_VALUE & (0xFFU));

  cframe->MsgId = REAR_PASS_DOOR_RPT_CANID;
  cframe->DLC = REAR_PASS_DOOR_RPT_DLC;
  cframe->IDE = REAR_PASS_DOOR_RPT_IDE;
  return REAR_PASS_DOOR_RPT_CANID;
}

#else

uint32_t Pack_REAR_PASS_DOOR_RPT_pacmod12(REAR_PASS_DOOR_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < REAR_PASS_DOOR_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  _d[1] |= (_m->MANUAL_INPUT & (0xFFU));
  _d[2] |= (_m->COMMANDED_VALUE & (0xFFU));
  _d[3] |= (_m->OUTPUT_VALUE & (0xFFU));

  *_len = REAR_PASS_DOOR_RPT_DLC;
  *_ide = REAR_PASS_DOOR_RPT_IDE;
  return REAR_PASS_DOOR_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_ENGINE_BRAKE_RPT_pacmod12(ENGINE_BRAKE_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLED = (_d[0] & (0x01U));
  _m->OVERRIDE_ACTIVE = ((_d[0] >> 1) & (0x01U));
  _m->COMMAND_OUTPUT_FAULT = ((_d[0] >> 2) & (0x01U));
  _m->INPUT_OUTPUT_FAULT = ((_d[0] >> 3) & (0x01U));
  _m->OUTPUT_REPORTED_FAULT = ((_d[0] >> 4) & (0x01U));
  _m->PACMOD_FAULT = ((_d[0] >> 5) & (0x01U));
  _m->VEHICLE_FAULT = ((_d[0] >> 6) & (0x01U));
  _m->COMMAND_TIMEOUT = ((_d[0] >> 7) & (0x01U));
  _m->MANUAL_INPUT = (_d[1] & (0xFFU));
  _m->COMMANDED_VALUE = (_d[2] & (0xFFU));
  _m->OUTPUT_VALUE = (_d[3] & (0xFFU));
  _m->MAN_AUTO = (_d[4] & (0x03U));
  _m->CMD_AUTO = ((_d[4] >> 2) & (0x03U));
  _m->OUT_AUTO = ((_d[4] >> 4) & (0x03U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < ENGINE_BRAKE_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_ENGINE_BRAKE_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return ENGINE_BRAKE_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_ENGINE_BRAKE_RPT_pacmod12(ENGINE_BRAKE_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < ENGINE_BRAKE_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  cframe->Data[1] |= (_m->MANUAL_INPUT & (0xFFU));
  cframe->Data[2] |= (_m->COMMANDED_VALUE & (0xFFU));
  cframe->Data[3] |= (_m->OUTPUT_VALUE & (0xFFU));
  cframe->Data[4] |= (_m->MAN_AUTO & (0x03U)) | ((_m->CMD_AUTO & (0x03U)) << 2) | ((_m->OUT_AUTO & (0x03U)) << 4);

  cframe->MsgId = ENGINE_BRAKE_RPT_CANID;
  cframe->DLC = ENGINE_BRAKE_RPT_DLC;
  cframe->IDE = ENGINE_BRAKE_RPT_IDE;
  return ENGINE_BRAKE_RPT_CANID;
}

#else

uint32_t Pack_ENGINE_BRAKE_RPT_pacmod12(ENGINE_BRAKE_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < ENGINE_BRAKE_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  _d[1] |= (_m->MANUAL_INPUT & (0xFFU));
  _d[2] |= (_m->COMMANDED_VALUE & (0xFFU));
  _d[3] |= (_m->OUTPUT_VALUE & (0xFFU));
  _d[4] |= (_m->MAN_AUTO & (0x03U)) | ((_m->CMD_AUTO & (0x03U)) << 2) | ((_m->OUT_AUTO & (0x03U)) << 4);

  *_len = ENGINE_BRAKE_RPT_DLC;
  *_ide = ENGINE_BRAKE_RPT_IDE;
  return ENGINE_BRAKE_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_EXHAUST_BRAKE_RPT_pacmod12(EXHAUST_BRAKE_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLED = (_d[0] & (0x01U));
  _m->OVERRIDE_ACTIVE = ((_d[0] >> 1) & (0x01U));
  _m->COMMAND_OUTPUT_FAULT = ((_d[0] >> 2) & (0x01U));
  _m->INPUT_OUTPUT_FAULT = ((_d[0] >> 3) & (0x01U));
  _m->OUTPUT_REPORTED_FAULT = ((_d[0] >> 4) & (0x01U));
  _m->PACMOD_FAULT = ((_d[0] >> 5) & (0x01U));
  _m->VEHICLE_FAULT = ((_d[0] >> 6) & (0x01U));
  _m->COMMAND_TIMEOUT = ((_d[0] >> 7) & (0x01U));
  _m->MANUAL_INPUT = (_d[1] & (0xFFU));
  _m->COMMANDED_VALUE = (_d[2] & (0xFFU));
  _m->OUTPUT_VALUE = (_d[3] & (0xFFU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < EXHAUST_BRAKE_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_EXHAUST_BRAKE_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return EXHAUST_BRAKE_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_EXHAUST_BRAKE_RPT_pacmod12(EXHAUST_BRAKE_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < EXHAUST_BRAKE_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  cframe->Data[1] |= (_m->MANUAL_INPUT & (0xFFU));
  cframe->Data[2] |= (_m->COMMANDED_VALUE & (0xFFU));
  cframe->Data[3] |= (_m->OUTPUT_VALUE & (0xFFU));

  cframe->MsgId = EXHAUST_BRAKE_RPT_CANID;
  cframe->DLC = EXHAUST_BRAKE_RPT_DLC;
  cframe->IDE = EXHAUST_BRAKE_RPT_IDE;
  return EXHAUST_BRAKE_RPT_CANID;
}

#else

uint32_t Pack_EXHAUST_BRAKE_RPT_pacmod12(EXHAUST_BRAKE_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < EXHAUST_BRAKE_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  _d[1] |= (_m->MANUAL_INPUT & (0xFFU));
  _d[2] |= (_m->COMMANDED_VALUE & (0xFFU));
  _d[3] |= (_m->OUTPUT_VALUE & (0xFFU));

  *_len = EXHAUST_BRAKE_RPT_DLC;
  *_ide = EXHAUST_BRAKE_RPT_IDE;
  return EXHAUST_BRAKE_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_MARKER_LAMP_RPT_pacmod12(MARKER_LAMP_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLED = (_d[0] & (0x01U));
  _m->OVERRIDE_ACTIVE = ((_d[0] >> 1) & (0x01U));
  _m->COMMAND_OUTPUT_FAULT = ((_d[0] >> 2) & (0x01U));
  _m->INPUT_OUTPUT_FAULT = ((_d[0] >> 3) & (0x01U));
  _m->OUTPUT_REPORTED_FAULT = ((_d[0] >> 4) & (0x01U));
  _m->PACMOD_FAULT = ((_d[0] >> 5) & (0x01U));
  _m->VEHICLE_FAULT = ((_d[0] >> 6) & (0x01U));
  _m->COMMAND_TIMEOUT = ((_d[0] >> 7) & (0x01U));
  _m->MANUAL_INPUT = (_d[1] & (0x01U));
  _m->COMMANDED_VALUE = (_d[2] & (0x01U));
  _m->OUTPUT_VALUE = (_d[3] & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < MARKER_LAMP_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_MARKER_LAMP_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return MARKER_LAMP_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_MARKER_LAMP_RPT_pacmod12(MARKER_LAMP_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < MARKER_LAMP_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  cframe->Data[1] |= (_m->MANUAL_INPUT & (0x01U));
  cframe->Data[2] |= (_m->COMMANDED_VALUE & (0x01U));
  cframe->Data[3] |= (_m->OUTPUT_VALUE & (0x01U));

  cframe->MsgId = MARKER_LAMP_RPT_CANID;
  cframe->DLC = MARKER_LAMP_RPT_DLC;
  cframe->IDE = MARKER_LAMP_RPT_IDE;
  return MARKER_LAMP_RPT_CANID;
}

#else

uint32_t Pack_MARKER_LAMP_RPT_pacmod12(MARKER_LAMP_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < MARKER_LAMP_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  _d[1] |= (_m->MANUAL_INPUT & (0x01U));
  _d[2] |= (_m->COMMANDED_VALUE & (0x01U));
  _d[3] |= (_m->OUTPUT_VALUE & (0x01U));

  *_len = MARKER_LAMP_RPT_DLC;
  *_ide = MARKER_LAMP_RPT_IDE;
  return MARKER_LAMP_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_CABIN_TEMP_RPT_pacmod12(CABIN_TEMP_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLED = (_d[0] & (0x01U));
  _m->OVERRIDE_ACTIVE = ((_d[0] >> 1) & (0x01U));
  _m->COMMAND_OUTPUT_FAULT = ((_d[0] >> 2) & (0x01U));
  _m->INPUT_OUTPUT_FAULT = ((_d[0] >> 3) & (0x01U));
  _m->OUTPUT_REPORTED_FAULT = ((_d[0] >> 4) & (0x01U));
  _m->PACMOD_FAULT = ((_d[0] >> 5) & (0x01U));
  _m->VEHICLE_FAULT = ((_d[0] >> 6) & (0x01U));
  _m->COMMAND_TIMEOUT = ((_d[0] >> 7) & (0x01U));
  _m->MANUAL_INPUT_ro = (_d[1] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->MANUAL_INPUT_phys = (sigfloat_t)(PACMOD12_MANUAL_INPUT_ro_fromS(_m->MANUAL_INPUT_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->COMMANDED_VALUE_ro = (_d[2] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->COMMANDED_VALUE_phys = (sigfloat_t)(PACMOD12_COMMANDED_VALUE_ro_fromS(_m->COMMANDED_VALUE_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->OUTPUT_VALUE_ro = (_d[3] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->OUTPUT_VALUE_phys = (sigfloat_t)(PACMOD12_OUTPUT_VALUE_ro_fromS(_m->OUTPUT_VALUE_ro));
#endif // PACMOD12_USE_SIGFLOAT

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CABIN_TEMP_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CABIN_TEMP_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return CABIN_TEMP_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_CABIN_TEMP_RPT_pacmod12(CABIN_TEMP_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < CABIN_TEMP_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->MANUAL_INPUT_ro = PACMOD12_MANUAL_INPUT_ro_toS(_m->MANUAL_INPUT_phys);
  _m->COMMANDED_VALUE_ro = PACMOD12_COMMANDED_VALUE_ro_toS(_m->COMMANDED_VALUE_phys);
  _m->OUTPUT_VALUE_ro = PACMOD12_OUTPUT_VALUE_ro_toS(_m->OUTPUT_VALUE_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  cframe->Data[1] |= (_m->MANUAL_INPUT_ro & (0xFFU));
  cframe->Data[2] |= (_m->COMMANDED_VALUE_ro & (0xFFU));
  cframe->Data[3] |= (_m->OUTPUT_VALUE_ro & (0xFFU));

  cframe->MsgId = CABIN_TEMP_RPT_CANID;
  cframe->DLC = CABIN_TEMP_RPT_DLC;
  cframe->IDE = CABIN_TEMP_RPT_IDE;
  return CABIN_TEMP_RPT_CANID;
}

#else

uint32_t Pack_CABIN_TEMP_RPT_pacmod12(CABIN_TEMP_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < CABIN_TEMP_RPT_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->MANUAL_INPUT_ro = PACMOD12_MANUAL_INPUT_ro_toS(_m->MANUAL_INPUT_phys);
  _m->COMMANDED_VALUE_ro = PACMOD12_COMMANDED_VALUE_ro_toS(_m->COMMANDED_VALUE_phys);
  _m->OUTPUT_VALUE_ro = PACMOD12_OUTPUT_VALUE_ro_toS(_m->OUTPUT_VALUE_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  _d[1] |= (_m->MANUAL_INPUT_ro & (0xFFU));
  _d[2] |= (_m->COMMANDED_VALUE_ro & (0xFFU));
  _d[3] |= (_m->OUTPUT_VALUE_ro & (0xFFU));

  *_len = CABIN_TEMP_RPT_DLC;
  *_ide = CABIN_TEMP_RPT_IDE;
  return CABIN_TEMP_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_CABIN_FAN_SPEED_RPT_pacmod12(CABIN_FAN_SPEED_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLED = (_d[0] & (0x01U));
  _m->OVERRIDE_ACTIVE = ((_d[0] >> 1) & (0x01U));
  _m->COMMAND_OUTPUT_FAULT = ((_d[0] >> 2) & (0x01U));
  _m->INPUT_OUTPUT_FAULT = ((_d[0] >> 3) & (0x01U));
  _m->OUTPUT_REPORTED_FAULT = ((_d[0] >> 4) & (0x01U));
  _m->PACMOD_FAULT = ((_d[0] >> 5) & (0x01U));
  _m->VEHICLE_FAULT = ((_d[0] >> 6) & (0x01U));
  _m->COMMAND_TIMEOUT = ((_d[0] >> 7) & (0x01U));
  _m->MANUAL_INPUT = (_d[1] & (0xFFU));
  _m->COMMANDED_VALUE = (_d[2] & (0xFFU));
  _m->OUTPUT_VALUE = (_d[3] & (0xFFU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CABIN_FAN_SPEED_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CABIN_FAN_SPEED_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return CABIN_FAN_SPEED_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_CABIN_FAN_SPEED_RPT_pacmod12(CABIN_FAN_SPEED_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < CABIN_FAN_SPEED_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  cframe->Data[1] |= (_m->MANUAL_INPUT & (0xFFU));
  cframe->Data[2] |= (_m->COMMANDED_VALUE & (0xFFU));
  cframe->Data[3] |= (_m->OUTPUT_VALUE & (0xFFU));

  cframe->MsgId = CABIN_FAN_SPEED_RPT_CANID;
  cframe->DLC = CABIN_FAN_SPEED_RPT_DLC;
  cframe->IDE = CABIN_FAN_SPEED_RPT_IDE;
  return CABIN_FAN_SPEED_RPT_CANID;
}

#else

uint32_t Pack_CABIN_FAN_SPEED_RPT_pacmod12(CABIN_FAN_SPEED_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < CABIN_FAN_SPEED_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  _d[1] |= (_m->MANUAL_INPUT & (0xFFU));
  _d[2] |= (_m->COMMANDED_VALUE & (0xFFU));
  _d[3] |= (_m->OUTPUT_VALUE & (0xFFU));

  *_len = CABIN_FAN_SPEED_RPT_DLC;
  *_ide = CABIN_FAN_SPEED_RPT_IDE;
  return CABIN_FAN_SPEED_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_CABIN_CLIMATE_RPT_pacmod12(CABIN_CLIMATE_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENABLED = (_d[0] & (0x01U));
  _m->OVERRIDE_ACTIVE = ((_d[0] >> 1) & (0x01U));
  _m->COMMAND_OUTPUT_FAULT = ((_d[0] >> 2) & (0x01U));
  _m->INPUT_OUTPUT_FAULT = ((_d[0] >> 3) & (0x01U));
  _m->OUTPUT_REPORTED_FAULT = ((_d[0] >> 4) & (0x01U));
  _m->PACMOD_FAULT = ((_d[0] >> 5) & (0x01U));
  _m->VEHICLE_FAULT = ((_d[0] >> 6) & (0x01U));
  _m->COMMAND_TIMEOUT = ((_d[0] >> 7) & (0x01U));
  _m->MAN_AC_OFF_ON = (_d[1] & (0x03U));
  _m->MAN_MAX_AC_OFF_ON = ((_d[1] >> 2) & (0x03U));
  _m->MAN_DEFROST_OFF_ON = ((_d[1] >> 4) & (0x03U));
  _m->MAN_MAX_DEFROST_OFF_ON = ((_d[1] >> 6) & (0x03U));
  _m->MAN_DIR_UP_OFF_ON = (_d[2] & (0x03U));
  _m->MAN_DIR_DOWN_OFF_ON = ((_d[2] >> 2) & (0x03U));
  _m->CMD_AC_OFF_ON = (_d[3] & (0x03U));
  _m->CMD_MAX_AC_OFF_ON = ((_d[3] >> 2) & (0x03U));
  _m->CMD_DEFROST_OFF_ON = ((_d[3] >> 4) & (0x03U));
  _m->CMD_MAX_DEFROST_OFF_ON = ((_d[3] >> 6) & (0x03U));
  _m->CMD_DIR_UP_OFF_ON = (_d[4] & (0x03U));
  _m->CMD_DIR_DOWN_OFF_ON = ((_d[4] >> 2) & (0x03U));
  _m->OUT_AC_OFF_ON = (_d[5] & (0x03U));
  _m->OUT_MAX_AC_OFF_ON = ((_d[5] >> 2) & (0x03U));
  _m->OUT_DEFROST_OFF_ON = ((_d[5] >> 4) & (0x03U));
  _m->OUT_MAX_DEFROST_OFF_ON = ((_d[5] >> 6) & (0x03U));
  _m->OUT_DIR_UP_OFF_ON = (_d[6] & (0x03U));
  _m->OUT_DIR_DOWN_OFF_ON = ((_d[6] >> 2) & (0x03U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CABIN_CLIMATE_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CABIN_CLIMATE_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return CABIN_CLIMATE_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_CABIN_CLIMATE_RPT_pacmod12(CABIN_CLIMATE_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < CABIN_CLIMATE_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  cframe->Data[1] |= (_m->MAN_AC_OFF_ON & (0x03U)) | ((_m->MAN_MAX_AC_OFF_ON & (0x03U)) << 2) | ((_m->MAN_DEFROST_OFF_ON & (0x03U)) << 4) | ((_m->MAN_MAX_DEFROST_OFF_ON & (0x03U)) << 6);
  cframe->Data[2] |= (_m->MAN_DIR_UP_OFF_ON & (0x03U)) | ((_m->MAN_DIR_DOWN_OFF_ON & (0x03U)) << 2);
  cframe->Data[3] |= (_m->CMD_AC_OFF_ON & (0x03U)) | ((_m->CMD_MAX_AC_OFF_ON & (0x03U)) << 2) | ((_m->CMD_DEFROST_OFF_ON & (0x03U)) << 4) | ((_m->CMD_MAX_DEFROST_OFF_ON & (0x03U)) << 6);
  cframe->Data[4] |= (_m->CMD_DIR_UP_OFF_ON & (0x03U)) | ((_m->CMD_DIR_DOWN_OFF_ON & (0x03U)) << 2);
  cframe->Data[5] |= (_m->OUT_AC_OFF_ON & (0x03U)) | ((_m->OUT_MAX_AC_OFF_ON & (0x03U)) << 2) | ((_m->OUT_DEFROST_OFF_ON & (0x03U)) << 4) | ((_m->OUT_MAX_DEFROST_OFF_ON & (0x03U)) << 6);
  cframe->Data[6] |= (_m->OUT_DIR_UP_OFF_ON & (0x03U)) | ((_m->OUT_DIR_DOWN_OFF_ON & (0x03U)) << 2);

  cframe->MsgId = CABIN_CLIMATE_RPT_CANID;
  cframe->DLC = CABIN_CLIMATE_RPT_DLC;
  cframe->IDE = CABIN_CLIMATE_RPT_IDE;
  return CABIN_CLIMATE_RPT_CANID;
}

#else

uint32_t Pack_CABIN_CLIMATE_RPT_pacmod12(CABIN_CLIMATE_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < CABIN_CLIMATE_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ENABLED & (0x01U)) | ((_m->OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->PACMOD_FAULT & (0x01U)) << 5) | ((_m->VEHICLE_FAULT & (0x01U)) << 6) | ((_m->COMMAND_TIMEOUT & (0x01U)) << 7);
  _d[1] |= (_m->MAN_AC_OFF_ON & (0x03U)) | ((_m->MAN_MAX_AC_OFF_ON & (0x03U)) << 2) | ((_m->MAN_DEFROST_OFF_ON & (0x03U)) << 4) | ((_m->MAN_MAX_DEFROST_OFF_ON & (0x03U)) << 6);
  _d[2] |= (_m->MAN_DIR_UP_OFF_ON & (0x03U)) | ((_m->MAN_DIR_DOWN_OFF_ON & (0x03U)) << 2);
  _d[3] |= (_m->CMD_AC_OFF_ON & (0x03U)) | ((_m->CMD_MAX_AC_OFF_ON & (0x03U)) << 2) | ((_m->CMD_DEFROST_OFF_ON & (0x03U)) << 4) | ((_m->CMD_MAX_DEFROST_OFF_ON & (0x03U)) << 6);
  _d[4] |= (_m->CMD_DIR_UP_OFF_ON & (0x03U)) | ((_m->CMD_DIR_DOWN_OFF_ON & (0x03U)) << 2);
  _d[5] |= (_m->OUT_AC_OFF_ON & (0x03U)) | ((_m->OUT_MAX_AC_OFF_ON & (0x03U)) << 2) | ((_m->OUT_DEFROST_OFF_ON & (0x03U)) << 4) | ((_m->OUT_MAX_DEFROST_OFF_ON & (0x03U)) << 6);
  _d[6] |= (_m->OUT_DIR_UP_OFF_ON & (0x03U)) | ((_m->OUT_DIR_DOWN_OFF_ON & (0x03U)) << 2);

  *_len = CABIN_CLIMATE_RPT_DLC;
  *_ide = CABIN_CLIMATE_RPT_IDE;
  return CABIN_CLIMATE_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_ACCEL_AUX_RPT_pacmod12(ACCEL_AUX_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->OPERATOR_INTERACTION = (_d[4] & (0x01U));
  _m->ACCEL_LIMITING_ACTIVE = ((_d[4] >> 1) & (0x01U));
  _m->PRK_BRK_INTERLOCK_ACTIVE = ((_d[4] >> 2) & (0x01U));
  _m->BRAKE_INTERLOCK_ACTIVE = ((_d[4] >> 3) & (0x01U));
  _m->CALIBRATION_STATUS = ((_d[4] >> 4) & (0x07U));
  _m->OPERATOR_INTERACTION_AVAIL = ((_d[5] >> 2) & (0x01U));
  _m->ACCEL_LIMITING_ACTIVE_AVAIL = ((_d[5] >> 3) & (0x01U));
  _m->PRK_BRK_INTERLOCK_ACTIVE_AVAIL = ((_d[5] >> 4) & (0x01U));
  _m->BRAKE_INTERLOCK_ACTIVE_AVAIL = ((_d[5] >> 5) & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < ACCEL_AUX_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_ACCEL_AUX_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return ACCEL_AUX_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_ACCEL_AUX_RPT_pacmod12(ACCEL_AUX_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < ACCEL_AUX_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[4] |= (_m->OPERATOR_INTERACTION & (0x01U)) | ((_m->ACCEL_LIMITING_ACTIVE & (0x01U)) << 1) | ((_m->PRK_BRK_INTERLOCK_ACTIVE & (0x01U)) << 2) | ((_m->BRAKE_INTERLOCK_ACTIVE & (0x01U)) << 3) | ((_m->CALIBRATION_STATUS & (0x07U)) << 4);
  cframe->Data[5] |= ((_m->OPERATOR_INTERACTION_AVAIL & (0x01U)) << 2) | ((_m->ACCEL_LIMITING_ACTIVE_AVAIL & (0x01U)) << 3) | ((_m->PRK_BRK_INTERLOCK_ACTIVE_AVAIL & (0x01U)) << 4) | ((_m->BRAKE_INTERLOCK_ACTIVE_AVAIL & (0x01U)) << 5);

  cframe->MsgId = ACCEL_AUX_RPT_CANID;
  cframe->DLC = ACCEL_AUX_RPT_DLC;
  cframe->IDE = ACCEL_AUX_RPT_IDE;
  return ACCEL_AUX_RPT_CANID;
}

#else

uint32_t Pack_ACCEL_AUX_RPT_pacmod12(ACCEL_AUX_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < ACCEL_AUX_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[4] |= (_m->OPERATOR_INTERACTION & (0x01U)) | ((_m->ACCEL_LIMITING_ACTIVE & (0x01U)) << 1) | ((_m->PRK_BRK_INTERLOCK_ACTIVE & (0x01U)) << 2) | ((_m->BRAKE_INTERLOCK_ACTIVE & (0x01U)) << 3) | ((_m->CALIBRATION_STATUS & (0x07U)) << 4);
  _d[5] |= ((_m->OPERATOR_INTERACTION_AVAIL & (0x01U)) << 2) | ((_m->ACCEL_LIMITING_ACTIVE_AVAIL & (0x01U)) << 3) | ((_m->PRK_BRK_INTERLOCK_ACTIVE_AVAIL & (0x01U)) << 4) | ((_m->BRAKE_INTERLOCK_ACTIVE_AVAIL & (0x01U)) << 5);

  *_len = ACCEL_AUX_RPT_DLC;
  *_ide = ACCEL_AUX_RPT_IDE;
  return ACCEL_AUX_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_BRAKE_AUX_RPT_pacmod12(BRAKE_AUX_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->BRAKE_PRESSURE_ro = ((_d[4] & (0xFFU)) << 8) | (_d[5] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->BRAKE_PRESSURE_phys = (sigfloat_t)(PACMOD12_BRAKE_PRESSURE_ro_fromS(_m->BRAKE_PRESSURE_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->OPERATOR_INTERACTION = (_d[6] & (0x01U));
  _m->BRAKE_ON_OFF = ((_d[6] >> 1) & (0x01U));
  _m->BRAKE_LIMITING_ACTIVE = ((_d[6] >> 2) & (0x01U));
  _m->BRAKE_REDUCED_ASSIST = ((_d[6] >> 3) & (0x01U));
  _m->CALIBRATION_STATUS = ((_d[6] >> 4) & (0x07U));
  _m->BRAKE_PRESSURE_AVAIL = ((_d[7] >> 2) & (0x01U));
  _m->OPERATOR_INTERACTION_AVAIL = ((_d[7] >> 3) & (0x01U));
  _m->BRAKE_ON_OFF_AVAIL = ((_d[7] >> 4) & (0x01U));
  _m->BRAKE_LIMITING_ACTIVE_AVAIL = ((_d[7] >> 5) & (0x01U));
  _m->BRAKE_REDUCED_ASSIST_AVAIL = ((_d[7] >> 6) & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < BRAKE_AUX_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_BRAKE_AUX_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return BRAKE_AUX_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_BRAKE_AUX_RPT_pacmod12(BRAKE_AUX_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < BRAKE_AUX_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->BRAKE_PRESSURE_ro = PACMOD12_BRAKE_PRESSURE_ro_toS(_m->BRAKE_PRESSURE_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[4] |= ((_m->BRAKE_PRESSURE_ro >> 8) & (0xFFU));
  cframe->Data[5] |= (_m->BRAKE_PRESSURE_ro & (0xFFU));
  cframe->Data[6] |= (_m->OPERATOR_INTERACTION & (0x01U)) | ((_m->BRAKE_ON_OFF & (0x01U)) << 1) | ((_m->BRAKE_LIMITING_ACTIVE & (0x01U)) << 2) | ((_m->BRAKE_REDUCED_ASSIST & (0x01U)) << 3) | ((_m->CALIBRATION_STATUS & (0x07U)) << 4);
  cframe->Data[7] |= ((_m->BRAKE_PRESSURE_AVAIL & (0x01U)) << 2) | ((_m->OPERATOR_INTERACTION_AVAIL & (0x01U)) << 3) | ((_m->BRAKE_ON_OFF_AVAIL & (0x01U)) << 4) | ((_m->BRAKE_LIMITING_ACTIVE_AVAIL & (0x01U)) << 5) | ((_m->BRAKE_REDUCED_ASSIST_AVAIL & (0x01U)) << 6);

  cframe->MsgId = BRAKE_AUX_RPT_CANID;
  cframe->DLC = BRAKE_AUX_RPT_DLC;
  cframe->IDE = BRAKE_AUX_RPT_IDE;
  return BRAKE_AUX_RPT_CANID;
}

#else

uint32_t Pack_BRAKE_AUX_RPT_pacmod12(BRAKE_AUX_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < BRAKE_AUX_RPT_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->BRAKE_PRESSURE_ro = PACMOD12_BRAKE_PRESSURE_ro_toS(_m->BRAKE_PRESSURE_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[4] |= ((_m->BRAKE_PRESSURE_ro >> 8) & (0xFFU));
  _d[5] |= (_m->BRAKE_PRESSURE_ro & (0xFFU));
  _d[6] |= (_m->OPERATOR_INTERACTION & (0x01U)) | ((_m->BRAKE_ON_OFF & (0x01U)) << 1) | ((_m->BRAKE_LIMITING_ACTIVE & (0x01U)) << 2) | ((_m->BRAKE_REDUCED_ASSIST & (0x01U)) << 3) | ((_m->CALIBRATION_STATUS & (0x07U)) << 4);
  _d[7] |= ((_m->BRAKE_PRESSURE_AVAIL & (0x01U)) << 2) | ((_m->OPERATOR_INTERACTION_AVAIL & (0x01U)) << 3) | ((_m->BRAKE_ON_OFF_AVAIL & (0x01U)) << 4) | ((_m->BRAKE_LIMITING_ACTIVE_AVAIL & (0x01U)) << 5) | ((_m->BRAKE_REDUCED_ASSIST_AVAIL & (0x01U)) << 6);

  *_len = BRAKE_AUX_RPT_DLC;
  *_ide = BRAKE_AUX_RPT_IDE;
  return BRAKE_AUX_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_HEADLIGHT_AUX_RPT_pacmod12(HEADLIGHT_AUX_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->HEADLIGHTS_ON = (_d[0] & (0x01U));
  _m->HEADLIGHTS_ON_BRIGHT = ((_d[0] >> 1) & (0x01U));
  _m->FOG_LIGHTS_ON = ((_d[0] >> 2) & (0x01U));
  _m->HEADLIGHTS_MODE = (_d[1] & (0xFFU));
  _m->HEADLIGHTS_ON_AVAIL = (_d[2] & (0x01U));
  _m->HEADLIGHTS_ON_BRIGHT_AVAIL = ((_d[2] >> 1) & (0x01U));
  _m->FOG_LIGHTS_ON_AVAIL = ((_d[2] >> 2) & (0x01U));
  _m->HEADLIGHTS_MODE_AVAIL = ((_d[2] >> 3) & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < HEADLIGHT_AUX_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_HEADLIGHT_AUX_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return HEADLIGHT_AUX_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_HEADLIGHT_AUX_RPT_pacmod12(HEADLIGHT_AUX_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < HEADLIGHT_AUX_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->HEADLIGHTS_ON & (0x01U)) | ((_m->HEADLIGHTS_ON_BRIGHT & (0x01U)) << 1) | ((_m->FOG_LIGHTS_ON & (0x01U)) << 2);
  cframe->Data[1] |= (_m->HEADLIGHTS_MODE & (0xFFU));
  cframe->Data[2] |= (_m->HEADLIGHTS_ON_AVAIL & (0x01U)) | ((_m->HEADLIGHTS_ON_BRIGHT_AVAIL & (0x01U)) << 1) | ((_m->FOG_LIGHTS_ON_AVAIL & (0x01U)) << 2) | ((_m->HEADLIGHTS_MODE_AVAIL & (0x01U)) << 3);

  cframe->MsgId = HEADLIGHT_AUX_RPT_CANID;
  cframe->DLC = HEADLIGHT_AUX_RPT_DLC;
  cframe->IDE = HEADLIGHT_AUX_RPT_IDE;
  return HEADLIGHT_AUX_RPT_CANID;
}

#else

uint32_t Pack_HEADLIGHT_AUX_RPT_pacmod12(HEADLIGHT_AUX_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < HEADLIGHT_AUX_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->HEADLIGHTS_ON & (0x01U)) | ((_m->HEADLIGHTS_ON_BRIGHT & (0x01U)) << 1) | ((_m->FOG_LIGHTS_ON & (0x01U)) << 2);
  _d[1] |= (_m->HEADLIGHTS_MODE & (0xFFU));
  _d[2] |= (_m->HEADLIGHTS_ON_AVAIL & (0x01U)) | ((_m->HEADLIGHTS_ON_BRIGHT_AVAIL & (0x01U)) << 1) | ((_m->FOG_LIGHTS_ON_AVAIL & (0x01U)) << 2) | ((_m->HEADLIGHTS_MODE_AVAIL & (0x01U)) << 3);

  *_len = HEADLIGHT_AUX_RPT_DLC;
  *_ide = HEADLIGHT_AUX_RPT_IDE;
  return HEADLIGHT_AUX_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_PARKING_BRAKE_AUX_RPT_pacmod12(PARKING_BRAKE_AUX_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->PARKING_BRAKE_STATUS = (_d[0] & (0x03U));
  _m->PARKING_BRAKE_STATUS_AVAIL = (_d[1] & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < PARKING_BRAKE_AUX_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_PARKING_BRAKE_AUX_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return PARKING_BRAKE_AUX_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_PARKING_BRAKE_AUX_RPT_pacmod12(PARKING_BRAKE_AUX_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < PARKING_BRAKE_AUX_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->PARKING_BRAKE_STATUS & (0x03U));
  cframe->Data[1] |= (_m->PARKING_BRAKE_STATUS_AVAIL & (0x01U));

  cframe->MsgId = PARKING_BRAKE_AUX_RPT_CANID;
  cframe->DLC = PARKING_BRAKE_AUX_RPT_DLC;
  cframe->IDE = PARKING_BRAKE_AUX_RPT_IDE;
  return PARKING_BRAKE_AUX_RPT_CANID;
}

#else

uint32_t Pack_PARKING_BRAKE_AUX_RPT_pacmod12(PARKING_BRAKE_AUX_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < PARKING_BRAKE_AUX_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->PARKING_BRAKE_STATUS & (0x03U));
  _d[1] |= (_m->PARKING_BRAKE_STATUS_AVAIL & (0x01U));

  *_len = PARKING_BRAKE_AUX_RPT_DLC;
  *_ide = PARKING_BRAKE_AUX_RPT_IDE;
  return PARKING_BRAKE_AUX_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_SHIFT_AUX_RPT_pacmod12(SHIFT_AUX_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->BETWEEN_GEARS = (_d[0] & (0x01U));
  _m->STAY_IN_NEUTRAL_MODE = ((_d[0] >> 1) & (0x01U));
  _m->BRAKE_INTERLOCK_ACTIVE = ((_d[0] >> 2) & (0x01U));
  _m->SPEED_INTERLOCK_ACTIVE = ((_d[0] >> 3) & (0x01U));
  _m->WRITE_TO_CONFIG = ((_d[0] >> 4) & (0x01U));
  _m->CALIBRATION_STATUS = ((_d[0] >> 5) & (0x07U));
  _m->BETWEEN_GEARS_AVAIL = (_d[1] & (0x01U));
  _m->STAY_IN_NEUTRAL_MODE_AVAIL = ((_d[1] >> 1) & (0x01U));
  _m->BRAKE_INTERLOCK_ACTIVE_AVAIL = ((_d[1] >> 2) & (0x01U));
  _m->SPEED_INTERLOCK_ACTIVE_AVAIL = ((_d[1] >> 3) & (0x01U));
  _m->WRITE_TO_CONFIG_IS_VALID = ((_d[1] >> 4) & (0x01U));
  _m->GEAR_NUMBER_AVAIL = ((_d[1] >> 5) & (0x01U));
  _m->SHIFT_MODE_AVAIL = ((_d[1] >> 6) & (0x01U));
  _m->GEAR_NUMBER = (_d[2] & (0x3FU));
  _m->SHIFT_MODE = ((_d[2] >> 6) & (0x03U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < SHIFT_AUX_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_SHIFT_AUX_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return SHIFT_AUX_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_SHIFT_AUX_RPT_pacmod12(SHIFT_AUX_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < SHIFT_AUX_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->BETWEEN_GEARS & (0x01U)) | ((_m->STAY_IN_NEUTRAL_MODE & (0x01U)) << 1) | ((_m->BRAKE_INTERLOCK_ACTIVE & (0x01U)) << 2) | ((_m->SPEED_INTERLOCK_ACTIVE & (0x01U)) << 3) | ((_m->WRITE_TO_CONFIG & (0x01U)) << 4) | ((_m->CALIBRATION_STATUS & (0x07U)) << 5);
  cframe->Data[1] |= (_m->BETWEEN_GEARS_AVAIL & (0x01U)) | ((_m->STAY_IN_NEUTRAL_MODE_AVAIL & (0x01U)) << 1) | ((_m->BRAKE_INTERLOCK_ACTIVE_AVAIL & (0x01U)) << 2) | ((_m->SPEED_INTERLOCK_ACTIVE_AVAIL & (0x01U)) << 3) | ((_m->WRITE_TO_CONFIG_IS_VALID & (0x01U)) << 4) | ((_m->GEAR_NUMBER_AVAIL & (0x01U)) << 5) | ((_m->SHIFT_MODE_AVAIL & (0x01U)) << 6);
  cframe->Data[2] |= (_m->GEAR_NUMBER & (0x3FU)) | ((_m->SHIFT_MODE & (0x03U)) << 6);

  cframe->MsgId = SHIFT_AUX_RPT_CANID;
  cframe->DLC = SHIFT_AUX_RPT_DLC;
  cframe->IDE = SHIFT_AUX_RPT_IDE;
  return SHIFT_AUX_RPT_CANID;
}

#else

uint32_t Pack_SHIFT_AUX_RPT_pacmod12(SHIFT_AUX_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < SHIFT_AUX_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->BETWEEN_GEARS & (0x01U)) | ((_m->STAY_IN_NEUTRAL_MODE & (0x01U)) << 1) | ((_m->BRAKE_INTERLOCK_ACTIVE & (0x01U)) << 2) | ((_m->SPEED_INTERLOCK_ACTIVE & (0x01U)) << 3) | ((_m->WRITE_TO_CONFIG & (0x01U)) << 4) | ((_m->CALIBRATION_STATUS & (0x07U)) << 5);
  _d[1] |= (_m->BETWEEN_GEARS_AVAIL & (0x01U)) | ((_m->STAY_IN_NEUTRAL_MODE_AVAIL & (0x01U)) << 1) | ((_m->BRAKE_INTERLOCK_ACTIVE_AVAIL & (0x01U)) << 2) | ((_m->SPEED_INTERLOCK_ACTIVE_AVAIL & (0x01U)) << 3) | ((_m->WRITE_TO_CONFIG_IS_VALID & (0x01U)) << 4) | ((_m->GEAR_NUMBER_AVAIL & (0x01U)) << 5) | ((_m->SHIFT_MODE_AVAIL & (0x01U)) << 6);
  _d[2] |= (_m->GEAR_NUMBER & (0x3FU)) | ((_m->SHIFT_MODE & (0x03U)) << 6);

  *_len = SHIFT_AUX_RPT_DLC;
  *_ide = SHIFT_AUX_RPT_IDE;
  return SHIFT_AUX_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_STEERING_AUX_RPT_pacmod12(STEERING_AUX_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->STEERING_TORQUE_ro = ((_d[2] & (0xFFU)) << 8) | (_d[3] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->STEERING_TORQUE_phys = (sigfloat_t)(PACMOD12_STEERING_TORQUE_ro_fromS(_m->STEERING_TORQUE_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->ROTATION_RATE_ro = ((_d[4] & (0xFFU)) << 8) | (_d[5] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->ROTATION_RATE_phys = (sigfloat_t)(PACMOD12_ROTATION_RATE_ro_fromS(_m->ROTATION_RATE_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->OPERATOR_INTERACTION = (_d[6] & (0x01U));
  _m->ROTATION_RATE_SIGN = ((_d[6] >> 1) & (0x01U));
  _m->VEHICLE_ANGLE_CALIB_STATUS = ((_d[6] >> 2) & (0x01U));
  _m->STEERING_LIMITING_ACTIVE = ((_d[6] >> 3) & (0x01U));
  _m->CALIBRATION_STATUS = ((_d[6] >> 4) & (0x07U));
  _m->STEERING_TORQUE_AVAIL = ((_d[7] >> 1) & (0x01U));
  _m->ROTATION_RATE_AVAIL = ((_d[7] >> 2) & (0x01U));
  _m->OPERATOR_INTERACTION_AVAIL = ((_d[7] >> 3) & (0x01U));
  _m->ROTATION_RATE_SIGN_AVAIL = ((_d[7] >> 4) & (0x01U));
  _m->VEHICLE_ANGLE_CALIB_STATUS_AVAIL = ((_d[7] >> 5) & (0x01U));
  _m->STEERING_LIMITING_ACTIVE_AVAIL = ((_d[7] >> 6) & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < STEERING_AUX_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_STEERING_AUX_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return STEERING_AUX_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_STEERING_AUX_RPT_pacmod12(STEERING_AUX_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < STEERING_AUX_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->STEERING_TORQUE_ro = PACMOD12_STEERING_TORQUE_ro_toS(_m->STEERING_TORQUE_phys);
  _m->ROTATION_RATE_ro = PACMOD12_ROTATION_RATE_ro_toS(_m->ROTATION_RATE_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[2] |= ((_m->STEERING_TORQUE_ro >> 8) & (0xFFU));
  cframe->Data[3] |= (_m->STEERING_TORQUE_ro & (0xFFU));
  cframe->Data[4] |= ((_m->ROTATION_RATE_ro >> 8) & (0xFFU));
  cframe->Data[5] |= (_m->ROTATION_RATE_ro & (0xFFU));
  cframe->Data[6] |= (_m->OPERATOR_INTERACTION & (0x01U)) | ((_m->ROTATION_RATE_SIGN & (0x01U)) << 1) | ((_m->VEHICLE_ANGLE_CALIB_STATUS & (0x01U)) << 2) | ((_m->STEERING_LIMITING_ACTIVE & (0x01U)) << 3) | ((_m->CALIBRATION_STATUS & (0x07U)) << 4);
  cframe->Data[7] |= ((_m->STEERING_TORQUE_AVAIL & (0x01U)) << 1) | ((_m->ROTATION_RATE_AVAIL & (0x01U)) << 2) | ((_m->OPERATOR_INTERACTION_AVAIL & (0x01U)) << 3) | ((_m->ROTATION_RATE_SIGN_AVAIL & (0x01U)) << 4) | ((_m->VEHICLE_ANGLE_CALIB_STATUS_AVAIL & (0x01U)) << 5) | ((_m->STEERING_LIMITING_ACTIVE_AVAIL & (0x01U)) << 6);

  cframe->MsgId = STEERING_AUX_RPT_CANID;
  cframe->DLC = STEERING_AUX_RPT_DLC;
  cframe->IDE = STEERING_AUX_RPT_IDE;
  return STEERING_AUX_RPT_CANID;
}

#else

uint32_t Pack_STEERING_AUX_RPT_pacmod12(STEERING_AUX_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < STEERING_AUX_RPT_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->STEERING_TORQUE_ro = PACMOD12_STEERING_TORQUE_ro_toS(_m->STEERING_TORQUE_phys);
  _m->ROTATION_RATE_ro = PACMOD12_ROTATION_RATE_ro_toS(_m->ROTATION_RATE_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[2] |= ((_m->STEERING_TORQUE_ro >> 8) & (0xFFU));
  _d[3] |= (_m->STEERING_TORQUE_ro & (0xFFU));
  _d[4] |= ((_m->ROTATION_RATE_ro >> 8) & (0xFFU));
  _d[5] |= (_m->ROTATION_RATE_ro & (0xFFU));
  _d[6] |= (_m->OPERATOR_INTERACTION & (0x01U)) | ((_m->ROTATION_RATE_SIGN & (0x01U)) << 1) | ((_m->VEHICLE_ANGLE_CALIB_STATUS & (0x01U)) << 2) | ((_m->STEERING_LIMITING_ACTIVE & (0x01U)) << 3) | ((_m->CALIBRATION_STATUS & (0x07U)) << 4);
  _d[7] |= ((_m->STEERING_TORQUE_AVAIL & (0x01U)) << 1) | ((_m->ROTATION_RATE_AVAIL & (0x01U)) << 2) | ((_m->OPERATOR_INTERACTION_AVAIL & (0x01U)) << 3) | ((_m->ROTATION_RATE_SIGN_AVAIL & (0x01U)) << 4) | ((_m->VEHICLE_ANGLE_CALIB_STATUS_AVAIL & (0x01U)) << 5) | ((_m->STEERING_LIMITING_ACTIVE_AVAIL & (0x01U)) << 6);

  *_len = STEERING_AUX_RPT_DLC;
  *_ide = STEERING_AUX_RPT_IDE;
  return STEERING_AUX_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_TURN_AUX_RPT_pacmod12(TURN_AUX_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->DRIVER_BLINKER_BULB_ON = (_d[0] & (0x01U));
  _m->PASS_BLINKER_BULB_ON = ((_d[0] >> 1) & (0x01U));
  _m->DRIVER_BLINKER_BULB_ON_AVAIL = (_d[1] & (0x01U));
  _m->PASS_BLINKER_BULB_ON_AVAIL = ((_d[1] >> 1) & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < TURN_AUX_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_TURN_AUX_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return TURN_AUX_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_TURN_AUX_RPT_pacmod12(TURN_AUX_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < TURN_AUX_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->DRIVER_BLINKER_BULB_ON & (0x01U)) | ((_m->PASS_BLINKER_BULB_ON & (0x01U)) << 1);
  cframe->Data[1] |= (_m->DRIVER_BLINKER_BULB_ON_AVAIL & (0x01U)) | ((_m->PASS_BLINKER_BULB_ON_AVAIL & (0x01U)) << 1);

  cframe->MsgId = TURN_AUX_RPT_CANID;
  cframe->DLC = TURN_AUX_RPT_DLC;
  cframe->IDE = TURN_AUX_RPT_IDE;
  return TURN_AUX_RPT_CANID;
}

#else

uint32_t Pack_TURN_AUX_RPT_pacmod12(TURN_AUX_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < TURN_AUX_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->DRIVER_BLINKER_BULB_ON & (0x01U)) | ((_m->PASS_BLINKER_BULB_ON & (0x01U)) << 1);
  _d[1] |= (_m->DRIVER_BLINKER_BULB_ON_AVAIL & (0x01U)) | ((_m->PASS_BLINKER_BULB_ON_AVAIL & (0x01U)) << 1);

  *_len = TURN_AUX_RPT_DLC;
  *_ide = TURN_AUX_RPT_IDE;
  return TURN_AUX_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_WIPER_AUX_RPT_pacmod12(WIPER_AUX_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->FRONT_WIPING = (_d[0] & (0x01U));
  _m->FRONT_SPRAYING = ((_d[0] >> 1) & (0x01U));
  _m->REAR_WIPING = ((_d[0] >> 2) & (0x01U));
  _m->REAR_SPRAYING = ((_d[0] >> 3) & (0x01U));
  _m->SPRAY_NEAR_EMPTY = ((_d[0] >> 4) & (0x01U));
  _m->SPRAY_EMPTY = ((_d[0] >> 5) & (0x01U));
  _m->FRONT_WIPING_AVAIL = (_d[1] & (0x01U));
  _m->FRONT_SPRAYING_AVAIL = ((_d[1] >> 1) & (0x01U));
  _m->REAR_WIPING_AVAIL = ((_d[1] >> 2) & (0x01U));
  _m->REAR_SPRAYING_AVAIL = ((_d[1] >> 3) & (0x01U));
  _m->SPRAY_NEAR_EMPTY_AVAIL = ((_d[1] >> 4) & (0x01U));
  _m->SPRAY_EMPTY_AVAIL = ((_d[1] >> 5) & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < WIPER_AUX_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_WIPER_AUX_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return WIPER_AUX_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_WIPER_AUX_RPT_pacmod12(WIPER_AUX_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < WIPER_AUX_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->FRONT_WIPING & (0x01U)) | ((_m->FRONT_SPRAYING & (0x01U)) << 1) | ((_m->REAR_WIPING & (0x01U)) << 2) | ((_m->REAR_SPRAYING & (0x01U)) << 3) | ((_m->SPRAY_NEAR_EMPTY & (0x01U)) << 4) | ((_m->SPRAY_EMPTY & (0x01U)) << 5);
  cframe->Data[1] |= (_m->FRONT_WIPING_AVAIL & (0x01U)) | ((_m->FRONT_SPRAYING_AVAIL & (0x01U)) << 1) | ((_m->REAR_WIPING_AVAIL & (0x01U)) << 2) | ((_m->REAR_SPRAYING_AVAIL & (0x01U)) << 3) | ((_m->SPRAY_NEAR_EMPTY_AVAIL & (0x01U)) << 4) | ((_m->SPRAY_EMPTY_AVAIL & (0x01U)) << 5);

  cframe->MsgId = WIPER_AUX_RPT_CANID;
  cframe->DLC = WIPER_AUX_RPT_DLC;
  cframe->IDE = WIPER_AUX_RPT_IDE;
  return WIPER_AUX_RPT_CANID;
}

#else

uint32_t Pack_WIPER_AUX_RPT_pacmod12(WIPER_AUX_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < WIPER_AUX_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->FRONT_WIPING & (0x01U)) | ((_m->FRONT_SPRAYING & (0x01U)) << 1) | ((_m->REAR_WIPING & (0x01U)) << 2) | ((_m->REAR_SPRAYING & (0x01U)) << 3) | ((_m->SPRAY_NEAR_EMPTY & (0x01U)) << 4) | ((_m->SPRAY_EMPTY & (0x01U)) << 5);
  _d[1] |= (_m->FRONT_WIPING_AVAIL & (0x01U)) | ((_m->FRONT_SPRAYING_AVAIL & (0x01U)) << 1) | ((_m->REAR_WIPING_AVAIL & (0x01U)) << 2) | ((_m->REAR_SPRAYING_AVAIL & (0x01U)) << 3) | ((_m->SPRAY_NEAR_EMPTY_AVAIL & (0x01U)) << 4) | ((_m->SPRAY_EMPTY_AVAIL & (0x01U)) << 5);

  *_len = WIPER_AUX_RPT_DLC;
  *_ide = WIPER_AUX_RPT_IDE;
  return WIPER_AUX_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_BRAKE_DECEL_AUX_RPT_pacmod12(BRAKE_DECEL_AUX_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->XBR_ACTIVE_CONTROL_MODE = (_d[0] & (0x0FU));
  _m->XBR_SYSTEM_STATE = ((_d[0] >> 4) & (0x03U));
  _m->FOUNDATION_BRAKE_USE = ((_d[0] >> 6) & (0x03U));
  _m->HILL_HOLDER_MODE = (_d[1] & (0x07U));
  _m->XBR_ACTIVE_CONTROL_MODE_AVAIL = (_d[2] & (0x01U));
  _m->XBR_SYSTEM_STATE_AVAIL = ((_d[2] >> 1) & (0x01U));
  _m->FOUNDATION_BRAKE_USE_AVAIL = ((_d[2] >> 2) & (0x01U));
  _m->HILL_HOLDER_MODE_AVAIL = ((_d[2] >> 3) & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < BRAKE_DECEL_AUX_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_BRAKE_DECEL_AUX_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return BRAKE_DECEL_AUX_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_BRAKE_DECEL_AUX_RPT_pacmod12(BRAKE_DECEL_AUX_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < BRAKE_DECEL_AUX_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->XBR_ACTIVE_CONTROL_MODE & (0x0FU)) | ((_m->XBR_SYSTEM_STATE & (0x03U)) << 4) | ((_m->FOUNDATION_BRAKE_USE & (0x03U)) << 6);
  cframe->Data[1] |= (_m->HILL_HOLDER_MODE & (0x07U));
  cframe->Data[2] |= (_m->XBR_ACTIVE_CONTROL_MODE_AVAIL & (0x01U)) | ((_m->XBR_SYSTEM_STATE_AVAIL & (0x01U)) << 1) | ((_m->FOUNDATION_BRAKE_USE_AVAIL & (0x01U)) << 2) | ((_m->HILL_HOLDER_MODE_AVAIL & (0x01U)) << 3);

  cframe->MsgId = BRAKE_DECEL_AUX_RPT_CANID;
  cframe->DLC = BRAKE_DECEL_AUX_RPT_DLC;
  cframe->IDE = BRAKE_DECEL_AUX_RPT_IDE;
  return BRAKE_DECEL_AUX_RPT_CANID;
}

#else

uint32_t Pack_BRAKE_DECEL_AUX_RPT_pacmod12(BRAKE_DECEL_AUX_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < BRAKE_DECEL_AUX_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->XBR_ACTIVE_CONTROL_MODE & (0x0FU)) | ((_m->XBR_SYSTEM_STATE & (0x03U)) << 4) | ((_m->FOUNDATION_BRAKE_USE & (0x03U)) << 6);
  _d[1] |= (_m->HILL_HOLDER_MODE & (0x07U));
  _d[2] |= (_m->XBR_ACTIVE_CONTROL_MODE_AVAIL & (0x01U)) | ((_m->XBR_SYSTEM_STATE_AVAIL & (0x01U)) << 1) | ((_m->FOUNDATION_BRAKE_USE_AVAIL & (0x01U)) << 2) | ((_m->HILL_HOLDER_MODE_AVAIL & (0x01U)) << 3);

  *_len = BRAKE_DECEL_AUX_RPT_DLC;
  *_ide = BRAKE_DECEL_AUX_RPT_IDE;
  return BRAKE_DECEL_AUX_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_ENGINE_BRAKE_AUX_RPT_pacmod12(ENGINE_BRAKE_AUX_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENGINE_BRAKE_STATUS = (_d[0] & (0x03U));
  _m->ACTUAL_ENGINE_BRK_TORQUE_ro = (_d[1] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->ACTUAL_ENGINE_BRK_TORQUE_phys = PACMOD12_ACTUAL_ENGINE_BRK_TORQUE_ro_fromS(_m->ACTUAL_ENGINE_BRK_TORQUE_ro);
#endif // PACMOD12_USE_SIGFLOAT

  _m->INTENDED_ENGINE_BRK_TORQUE_ro = (_d[2] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->INTENDED_ENGINE_BRK_TORQUE_phys = PACMOD12_INTENDED_ENGINE_BRK_TORQUE_ro_fromS(_m->INTENDED_ENGINE_BRK_TORQUE_ro);
#endif // PACMOD12_USE_SIGFLOAT

  _m->ENGINE_BRAKE_STATUS_AVAIL = (_d[3] & (0x01U));
  _m->ACTUAL_ENGINE_BRK_TORQUE_AVAIL = ((_d[3] >> 1) & (0x01U));
  _m->INTENDED_ENGINE_BRK_TORQUE_AVAIL = ((_d[3] >> 2) & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < ENGINE_BRAKE_AUX_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_ENGINE_BRAKE_AUX_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return ENGINE_BRAKE_AUX_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_ENGINE_BRAKE_AUX_RPT_pacmod12(ENGINE_BRAKE_AUX_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < ENGINE_BRAKE_AUX_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->ACTUAL_ENGINE_BRK_TORQUE_ro = PACMOD12_ACTUAL_ENGINE_BRK_TORQUE_ro_toS(_m->ACTUAL_ENGINE_BRK_TORQUE_phys);
  _m->INTENDED_ENGINE_BRK_TORQUE_ro = PACMOD12_INTENDED_ENGINE_BRK_TORQUE_ro_toS(_m->INTENDED_ENGINE_BRK_TORQUE_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[0] |= (_m->ENGINE_BRAKE_STATUS & (0x03U));
  cframe->Data[1] |= (_m->ACTUAL_ENGINE_BRK_TORQUE_ro & (0xFFU));
  cframe->Data[2] |= (_m->INTENDED_ENGINE_BRK_TORQUE_ro & (0xFFU));
  cframe->Data[3] |= (_m->ENGINE_BRAKE_STATUS_AVAIL & (0x01U)) | ((_m->ACTUAL_ENGINE_BRK_TORQUE_AVAIL & (0x01U)) << 1) | ((_m->INTENDED_ENGINE_BRK_TORQUE_AVAIL & (0x01U)) << 2);

  cframe->MsgId = ENGINE_BRAKE_AUX_RPT_CANID;
  cframe->DLC = ENGINE_BRAKE_AUX_RPT_DLC;
  cframe->IDE = ENGINE_BRAKE_AUX_RPT_IDE;
  return ENGINE_BRAKE_AUX_RPT_CANID;
}

#else

uint32_t Pack_ENGINE_BRAKE_AUX_RPT_pacmod12(ENGINE_BRAKE_AUX_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < ENGINE_BRAKE_AUX_RPT_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->ACTUAL_ENGINE_BRK_TORQUE_ro = PACMOD12_ACTUAL_ENGINE_BRK_TORQUE_ro_toS(_m->ACTUAL_ENGINE_BRK_TORQUE_phys);
  _m->INTENDED_ENGINE_BRK_TORQUE_ro = PACMOD12_INTENDED_ENGINE_BRK_TORQUE_ro_toS(_m->INTENDED_ENGINE_BRK_TORQUE_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[0] |= (_m->ENGINE_BRAKE_STATUS & (0x03U));
  _d[1] |= (_m->ACTUAL_ENGINE_BRK_TORQUE_ro & (0xFFU));
  _d[2] |= (_m->INTENDED_ENGINE_BRK_TORQUE_ro & (0xFFU));
  _d[3] |= (_m->ENGINE_BRAKE_STATUS_AVAIL & (0x01U)) | ((_m->ACTUAL_ENGINE_BRK_TORQUE_AVAIL & (0x01U)) << 1) | ((_m->INTENDED_ENGINE_BRK_TORQUE_AVAIL & (0x01U)) << 2);

  *_len = ENGINE_BRAKE_AUX_RPT_DLC;
  *_ide = ENGINE_BRAKE_AUX_RPT_IDE;
  return ENGINE_BRAKE_AUX_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_VEHICLE_SPEED_RPT_pacmod12(VEHICLE_SPEED_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->VEHICLE_SPEED_ro = ((_d[0] & (0xFFU)) << 8) | (_d[1] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->VEHICLE_SPEED_phys = (sigfloat_t)(PACMOD12_VEHICLE_SPEED_ro_fromS(_m->VEHICLE_SPEED_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->VEHICLE_SPEED_VALID = (_d[2] & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < VEHICLE_SPEED_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_VEHICLE_SPEED_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return VEHICLE_SPEED_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_VEHICLE_SPEED_RPT_pacmod12(VEHICLE_SPEED_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < VEHICLE_SPEED_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->VEHICLE_SPEED_ro = PACMOD12_VEHICLE_SPEED_ro_toS(_m->VEHICLE_SPEED_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[0] |= ((_m->VEHICLE_SPEED_ro >> 8) & (0xFFU));
  cframe->Data[1] |= (_m->VEHICLE_SPEED_ro & (0xFFU));
  cframe->Data[2] |= (_m->VEHICLE_SPEED_VALID & (0x01U));

  cframe->MsgId = VEHICLE_SPEED_RPT_CANID;
  cframe->DLC = VEHICLE_SPEED_RPT_DLC;
  cframe->IDE = VEHICLE_SPEED_RPT_IDE;
  return VEHICLE_SPEED_RPT_CANID;
}

#else

uint32_t Pack_VEHICLE_SPEED_RPT_pacmod12(VEHICLE_SPEED_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < VEHICLE_SPEED_RPT_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->VEHICLE_SPEED_ro = PACMOD12_VEHICLE_SPEED_ro_toS(_m->VEHICLE_SPEED_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[0] |= ((_m->VEHICLE_SPEED_ro >> 8) & (0xFFU));
  _d[1] |= (_m->VEHICLE_SPEED_ro & (0xFFU));
  _d[2] |= (_m->VEHICLE_SPEED_VALID & (0x01U));

  *_len = VEHICLE_SPEED_RPT_DLC;
  *_ide = VEHICLE_SPEED_RPT_IDE;
  return VEHICLE_SPEED_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_BRAKE_MOTOR_RPT_1_pacmod12(BRAKE_MOTOR_RPT_1_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->MOTOR_CURRENT_ro = ((_d[0] & (0xFFU)) << 24) | ((_d[1] & (0xFFU)) << 16) | ((_d[2] & (0xFFU)) << 8) | (_d[3] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->MOTOR_CURRENT_phys = (sigfloat_t)(PACMOD12_MOTOR_CURRENT_ro_fromS(_m->MOTOR_CURRENT_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->SHAFT_POSITION_ro = ((_d[4] & (0xFFU)) << 24) | ((_d[5] & (0xFFU)) << 16) | ((_d[6] & (0xFFU)) << 8) | (_d[7] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->SHAFT_POSITION_phys = (sigfloat_t)(PACMOD12_SHAFT_POSITION_ro_fromS(_m->SHAFT_POSITION_ro));
#endif // PACMOD12_USE_SIGFLOAT

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < BRAKE_MOTOR_RPT_1_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_BRAKE_MOTOR_RPT_1_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return BRAKE_MOTOR_RPT_1_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_BRAKE_MOTOR_RPT_1_pacmod12(BRAKE_MOTOR_RPT_1_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < BRAKE_MOTOR_RPT_1_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->MOTOR_CURRENT_ro = PACMOD12_MOTOR_CURRENT_ro_toS(_m->MOTOR_CURRENT_phys);
  _m->SHAFT_POSITION_ro = PACMOD12_SHAFT_POSITION_ro_toS(_m->SHAFT_POSITION_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[0] |= ((_m->MOTOR_CURRENT_ro >> 24) & (0xFFU));
  cframe->Data[1] |= ((_m->MOTOR_CURRENT_ro >> 16) & (0xFFU));
  cframe->Data[2] |= ((_m->MOTOR_CURRENT_ro >> 8) & (0xFFU));
  cframe->Data[3] |= (_m->MOTOR_CURRENT_ro & (0xFFU));
  cframe->Data[4] |= ((_m->SHAFT_POSITION_ro >> 24) & (0xFFU));
  cframe->Data[5] |= ((_m->SHAFT_POSITION_ro >> 16) & (0xFFU));
  cframe->Data[6] |= ((_m->SHAFT_POSITION_ro >> 8) & (0xFFU));
  cframe->Data[7] |= (_m->SHAFT_POSITION_ro & (0xFFU));

  cframe->MsgId = BRAKE_MOTOR_RPT_1_CANID;
  cframe->DLC = BRAKE_MOTOR_RPT_1_DLC;
  cframe->IDE = BRAKE_MOTOR_RPT_1_IDE;
  return BRAKE_MOTOR_RPT_1_CANID;
}

#else

uint32_t Pack_BRAKE_MOTOR_RPT_1_pacmod12(BRAKE_MOTOR_RPT_1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < BRAKE_MOTOR_RPT_1_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->MOTOR_CURRENT_ro = PACMOD12_MOTOR_CURRENT_ro_toS(_m->MOTOR_CURRENT_phys);
  _m->SHAFT_POSITION_ro = PACMOD12_SHAFT_POSITION_ro_toS(_m->SHAFT_POSITION_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[0] |= ((_m->MOTOR_CURRENT_ro >> 24) & (0xFFU));
  _d[1] |= ((_m->MOTOR_CURRENT_ro >> 16) & (0xFFU));
  _d[2] |= ((_m->MOTOR_CURRENT_ro >> 8) & (0xFFU));
  _d[3] |= (_m->MOTOR_CURRENT_ro & (0xFFU));
  _d[4] |= ((_m->SHAFT_POSITION_ro >> 24) & (0xFFU));
  _d[5] |= ((_m->SHAFT_POSITION_ro >> 16) & (0xFFU));
  _d[6] |= ((_m->SHAFT_POSITION_ro >> 8) & (0xFFU));
  _d[7] |= (_m->SHAFT_POSITION_ro & (0xFFU));

  *_len = BRAKE_MOTOR_RPT_1_DLC;
  *_ide = BRAKE_MOTOR_RPT_1_IDE;
  return BRAKE_MOTOR_RPT_1_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_BRAKE_MOTOR_RPT_2_pacmod12(BRAKE_MOTOR_RPT_2_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENCODER_TEMPERATURE = ((_d[0] & (0xFFU)) << 8) | (_d[1] & (0xFFU));
  _m->MOTOR_TEMPERATURE = ((_d[2] & (0xFFU)) << 8) | (_d[3] & (0xFFU));
  _m->ANGULAR_SPEED_ro = ((_d[4] & (0xFFU)) << 24) | ((_d[5] & (0xFFU)) << 16) | ((_d[6] & (0xFFU)) << 8) | (_d[7] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->ANGULAR_SPEED_phys = (sigfloat_t)(PACMOD12_ANGULAR_SPEED_ro_fromS(_m->ANGULAR_SPEED_ro));
#endif // PACMOD12_USE_SIGFLOAT

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < BRAKE_MOTOR_RPT_2_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_BRAKE_MOTOR_RPT_2_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return BRAKE_MOTOR_RPT_2_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_BRAKE_MOTOR_RPT_2_pacmod12(BRAKE_MOTOR_RPT_2_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < BRAKE_MOTOR_RPT_2_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->ANGULAR_SPEED_ro = PACMOD12_ANGULAR_SPEED_ro_toS(_m->ANGULAR_SPEED_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[0] |= ((_m->ENCODER_TEMPERATURE >> 8) & (0xFFU));
  cframe->Data[1] |= (_m->ENCODER_TEMPERATURE & (0xFFU));
  cframe->Data[2] |= ((_m->MOTOR_TEMPERATURE >> 8) & (0xFFU));
  cframe->Data[3] |= (_m->MOTOR_TEMPERATURE & (0xFFU));
  cframe->Data[4] |= ((_m->ANGULAR_SPEED_ro >> 24) & (0xFFU));
  cframe->Data[5] |= ((_m->ANGULAR_SPEED_ro >> 16) & (0xFFU));
  cframe->Data[6] |= ((_m->ANGULAR_SPEED_ro >> 8) & (0xFFU));
  cframe->Data[7] |= (_m->ANGULAR_SPEED_ro & (0xFFU));

  cframe->MsgId = BRAKE_MOTOR_RPT_2_CANID;
  cframe->DLC = BRAKE_MOTOR_RPT_2_DLC;
  cframe->IDE = BRAKE_MOTOR_RPT_2_IDE;
  return BRAKE_MOTOR_RPT_2_CANID;
}

#else

uint32_t Pack_BRAKE_MOTOR_RPT_2_pacmod12(BRAKE_MOTOR_RPT_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < BRAKE_MOTOR_RPT_2_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->ANGULAR_SPEED_ro = PACMOD12_ANGULAR_SPEED_ro_toS(_m->ANGULAR_SPEED_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[0] |= ((_m->ENCODER_TEMPERATURE >> 8) & (0xFFU));
  _d[1] |= (_m->ENCODER_TEMPERATURE & (0xFFU));
  _d[2] |= ((_m->MOTOR_TEMPERATURE >> 8) & (0xFFU));
  _d[3] |= (_m->MOTOR_TEMPERATURE & (0xFFU));
  _d[4] |= ((_m->ANGULAR_SPEED_ro >> 24) & (0xFFU));
  _d[5] |= ((_m->ANGULAR_SPEED_ro >> 16) & (0xFFU));
  _d[6] |= ((_m->ANGULAR_SPEED_ro >> 8) & (0xFFU));
  _d[7] |= (_m->ANGULAR_SPEED_ro & (0xFFU));

  *_len = BRAKE_MOTOR_RPT_2_DLC;
  *_ide = BRAKE_MOTOR_RPT_2_IDE;
  return BRAKE_MOTOR_RPT_2_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_BRAKE_MOTOR_RPT_3_pacmod12(BRAKE_MOTOR_RPT_3_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->TORQUE_OUTPUT_ro = ((_d[0] & (0xFFU)) << 24) | ((_d[1] & (0xFFU)) << 16) | ((_d[2] & (0xFFU)) << 8) | (_d[3] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->TORQUE_OUTPUT_phys = (sigfloat_t)(PACMOD12_TORQUE_OUTPUT_ro_fromS(_m->TORQUE_OUTPUT_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->TORQUE_INPUT_ro = ((_d[4] & (0xFFU)) << 24) | ((_d[5] & (0xFFU)) << 16) | ((_d[6] & (0xFFU)) << 8) | (_d[7] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->TORQUE_INPUT_phys = (sigfloat_t)(PACMOD12_TORQUE_INPUT_ro_fromS(_m->TORQUE_INPUT_ro));
#endif // PACMOD12_USE_SIGFLOAT

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < BRAKE_MOTOR_RPT_3_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_BRAKE_MOTOR_RPT_3_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return BRAKE_MOTOR_RPT_3_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_BRAKE_MOTOR_RPT_3_pacmod12(BRAKE_MOTOR_RPT_3_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < BRAKE_MOTOR_RPT_3_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->TORQUE_OUTPUT_ro = PACMOD12_TORQUE_OUTPUT_ro_toS(_m->TORQUE_OUTPUT_phys);
  _m->TORQUE_INPUT_ro = PACMOD12_TORQUE_INPUT_ro_toS(_m->TORQUE_INPUT_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[0] |= ((_m->TORQUE_OUTPUT_ro >> 24) & (0xFFU));
  cframe->Data[1] |= ((_m->TORQUE_OUTPUT_ro >> 16) & (0xFFU));
  cframe->Data[2] |= ((_m->TORQUE_OUTPUT_ro >> 8) & (0xFFU));
  cframe->Data[3] |= (_m->TORQUE_OUTPUT_ro & (0xFFU));
  cframe->Data[4] |= ((_m->TORQUE_INPUT_ro >> 24) & (0xFFU));
  cframe->Data[5] |= ((_m->TORQUE_INPUT_ro >> 16) & (0xFFU));
  cframe->Data[6] |= ((_m->TORQUE_INPUT_ro >> 8) & (0xFFU));
  cframe->Data[7] |= (_m->TORQUE_INPUT_ro & (0xFFU));

  cframe->MsgId = BRAKE_MOTOR_RPT_3_CANID;
  cframe->DLC = BRAKE_MOTOR_RPT_3_DLC;
  cframe->IDE = BRAKE_MOTOR_RPT_3_IDE;
  return BRAKE_MOTOR_RPT_3_CANID;
}

#else

uint32_t Pack_BRAKE_MOTOR_RPT_3_pacmod12(BRAKE_MOTOR_RPT_3_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < BRAKE_MOTOR_RPT_3_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->TORQUE_OUTPUT_ro = PACMOD12_TORQUE_OUTPUT_ro_toS(_m->TORQUE_OUTPUT_phys);
  _m->TORQUE_INPUT_ro = PACMOD12_TORQUE_INPUT_ro_toS(_m->TORQUE_INPUT_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[0] |= ((_m->TORQUE_OUTPUT_ro >> 24) & (0xFFU));
  _d[1] |= ((_m->TORQUE_OUTPUT_ro >> 16) & (0xFFU));
  _d[2] |= ((_m->TORQUE_OUTPUT_ro >> 8) & (0xFFU));
  _d[3] |= (_m->TORQUE_OUTPUT_ro & (0xFFU));
  _d[4] |= ((_m->TORQUE_INPUT_ro >> 24) & (0xFFU));
  _d[5] |= ((_m->TORQUE_INPUT_ro >> 16) & (0xFFU));
  _d[6] |= ((_m->TORQUE_INPUT_ro >> 8) & (0xFFU));
  _d[7] |= (_m->TORQUE_INPUT_ro & (0xFFU));

  *_len = BRAKE_MOTOR_RPT_3_DLC;
  *_ide = BRAKE_MOTOR_RPT_3_IDE;
  return BRAKE_MOTOR_RPT_3_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_STEERING_MOTOR_RPT_1_pacmod12(STEERING_MOTOR_RPT_1_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->MOTOR_CURRENT_ro = ((_d[0] & (0xFFU)) << 24) | ((_d[1] & (0xFFU)) << 16) | ((_d[2] & (0xFFU)) << 8) | (_d[3] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->MOTOR_CURRENT_phys = (sigfloat_t)(PACMOD12_MOTOR_CURRENT_ro_fromS(_m->MOTOR_CURRENT_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->SHAFT_POSITION_ro = ((_d[4] & (0xFFU)) << 24) | ((_d[5] & (0xFFU)) << 16) | ((_d[6] & (0xFFU)) << 8) | (_d[7] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->SHAFT_POSITION_phys = (sigfloat_t)(PACMOD12_SHAFT_POSITION_ro_fromS(_m->SHAFT_POSITION_ro));
#endif // PACMOD12_USE_SIGFLOAT

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < STEERING_MOTOR_RPT_1_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_STEERING_MOTOR_RPT_1_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return STEERING_MOTOR_RPT_1_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_STEERING_MOTOR_RPT_1_pacmod12(STEERING_MOTOR_RPT_1_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < STEERING_MOTOR_RPT_1_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->MOTOR_CURRENT_ro = PACMOD12_MOTOR_CURRENT_ro_toS(_m->MOTOR_CURRENT_phys);
  _m->SHAFT_POSITION_ro = PACMOD12_SHAFT_POSITION_ro_toS(_m->SHAFT_POSITION_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[0] |= ((_m->MOTOR_CURRENT_ro >> 24) & (0xFFU));
  cframe->Data[1] |= ((_m->MOTOR_CURRENT_ro >> 16) & (0xFFU));
  cframe->Data[2] |= ((_m->MOTOR_CURRENT_ro >> 8) & (0xFFU));
  cframe->Data[3] |= (_m->MOTOR_CURRENT_ro & (0xFFU));
  cframe->Data[4] |= ((_m->SHAFT_POSITION_ro >> 24) & (0xFFU));
  cframe->Data[5] |= ((_m->SHAFT_POSITION_ro >> 16) & (0xFFU));
  cframe->Data[6] |= ((_m->SHAFT_POSITION_ro >> 8) & (0xFFU));
  cframe->Data[7] |= (_m->SHAFT_POSITION_ro & (0xFFU));

  cframe->MsgId = STEERING_MOTOR_RPT_1_CANID;
  cframe->DLC = STEERING_MOTOR_RPT_1_DLC;
  cframe->IDE = STEERING_MOTOR_RPT_1_IDE;
  return STEERING_MOTOR_RPT_1_CANID;
}

#else

uint32_t Pack_STEERING_MOTOR_RPT_1_pacmod12(STEERING_MOTOR_RPT_1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < STEERING_MOTOR_RPT_1_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->MOTOR_CURRENT_ro = PACMOD12_MOTOR_CURRENT_ro_toS(_m->MOTOR_CURRENT_phys);
  _m->SHAFT_POSITION_ro = PACMOD12_SHAFT_POSITION_ro_toS(_m->SHAFT_POSITION_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[0] |= ((_m->MOTOR_CURRENT_ro >> 24) & (0xFFU));
  _d[1] |= ((_m->MOTOR_CURRENT_ro >> 16) & (0xFFU));
  _d[2] |= ((_m->MOTOR_CURRENT_ro >> 8) & (0xFFU));
  _d[3] |= (_m->MOTOR_CURRENT_ro & (0xFFU));
  _d[4] |= ((_m->SHAFT_POSITION_ro >> 24) & (0xFFU));
  _d[5] |= ((_m->SHAFT_POSITION_ro >> 16) & (0xFFU));
  _d[6] |= ((_m->SHAFT_POSITION_ro >> 8) & (0xFFU));
  _d[7] |= (_m->SHAFT_POSITION_ro & (0xFFU));

  *_len = STEERING_MOTOR_RPT_1_DLC;
  *_ide = STEERING_MOTOR_RPT_1_IDE;
  return STEERING_MOTOR_RPT_1_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_STEERING_MOTOR_RPT_2_pacmod12(STEERING_MOTOR_RPT_2_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENCODER_TEMPERATURE = ((_d[0] & (0xFFU)) << 8) | (_d[1] & (0xFFU));
  _m->MOTOR_TEMPERATURE = ((_d[2] & (0xFFU)) << 8) | (_d[3] & (0xFFU));
  _m->ANGULAR_SPEED_ro = ((_d[4] & (0xFFU)) << 24) | ((_d[5] & (0xFFU)) << 16) | ((_d[6] & (0xFFU)) << 8) | (_d[7] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->ANGULAR_SPEED_phys = (sigfloat_t)(PACMOD12_ANGULAR_SPEED_ro_fromS(_m->ANGULAR_SPEED_ro));
#endif // PACMOD12_USE_SIGFLOAT

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < STEERING_MOTOR_RPT_2_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_STEERING_MOTOR_RPT_2_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return STEERING_MOTOR_RPT_2_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_STEERING_MOTOR_RPT_2_pacmod12(STEERING_MOTOR_RPT_2_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < STEERING_MOTOR_RPT_2_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->ANGULAR_SPEED_ro = PACMOD12_ANGULAR_SPEED_ro_toS(_m->ANGULAR_SPEED_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[0] |= ((_m->ENCODER_TEMPERATURE >> 8) & (0xFFU));
  cframe->Data[1] |= (_m->ENCODER_TEMPERATURE & (0xFFU));
  cframe->Data[2] |= ((_m->MOTOR_TEMPERATURE >> 8) & (0xFFU));
  cframe->Data[3] |= (_m->MOTOR_TEMPERATURE & (0xFFU));
  cframe->Data[4] |= ((_m->ANGULAR_SPEED_ro >> 24) & (0xFFU));
  cframe->Data[5] |= ((_m->ANGULAR_SPEED_ro >> 16) & (0xFFU));
  cframe->Data[6] |= ((_m->ANGULAR_SPEED_ro >> 8) & (0xFFU));
  cframe->Data[7] |= (_m->ANGULAR_SPEED_ro & (0xFFU));

  cframe->MsgId = STEERING_MOTOR_RPT_2_CANID;
  cframe->DLC = STEERING_MOTOR_RPT_2_DLC;
  cframe->IDE = STEERING_MOTOR_RPT_2_IDE;
  return STEERING_MOTOR_RPT_2_CANID;
}

#else

uint32_t Pack_STEERING_MOTOR_RPT_2_pacmod12(STEERING_MOTOR_RPT_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < STEERING_MOTOR_RPT_2_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->ANGULAR_SPEED_ro = PACMOD12_ANGULAR_SPEED_ro_toS(_m->ANGULAR_SPEED_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[0] |= ((_m->ENCODER_TEMPERATURE >> 8) & (0xFFU));
  _d[1] |= (_m->ENCODER_TEMPERATURE & (0xFFU));
  _d[2] |= ((_m->MOTOR_TEMPERATURE >> 8) & (0xFFU));
  _d[3] |= (_m->MOTOR_TEMPERATURE & (0xFFU));
  _d[4] |= ((_m->ANGULAR_SPEED_ro >> 24) & (0xFFU));
  _d[5] |= ((_m->ANGULAR_SPEED_ro >> 16) & (0xFFU));
  _d[6] |= ((_m->ANGULAR_SPEED_ro >> 8) & (0xFFU));
  _d[7] |= (_m->ANGULAR_SPEED_ro & (0xFFU));

  *_len = STEERING_MOTOR_RPT_2_DLC;
  *_ide = STEERING_MOTOR_RPT_2_IDE;
  return STEERING_MOTOR_RPT_2_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_STEERING_MOTOR_RPT_3_pacmod12(STEERING_MOTOR_RPT_3_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->TORQUE_OUTPUT_ro = ((_d[0] & (0xFFU)) << 24) | ((_d[1] & (0xFFU)) << 16) | ((_d[2] & (0xFFU)) << 8) | (_d[3] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->TORQUE_OUTPUT_phys = (sigfloat_t)(PACMOD12_TORQUE_OUTPUT_ro_fromS(_m->TORQUE_OUTPUT_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->TORQUE_INPUT_ro = ((_d[4] & (0xFFU)) << 24) | ((_d[5] & (0xFFU)) << 16) | ((_d[6] & (0xFFU)) << 8) | (_d[7] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->TORQUE_INPUT_phys = (sigfloat_t)(PACMOD12_TORQUE_INPUT_ro_fromS(_m->TORQUE_INPUT_ro));
#endif // PACMOD12_USE_SIGFLOAT

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < STEERING_MOTOR_RPT_3_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_STEERING_MOTOR_RPT_3_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return STEERING_MOTOR_RPT_3_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_STEERING_MOTOR_RPT_3_pacmod12(STEERING_MOTOR_RPT_3_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < STEERING_MOTOR_RPT_3_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->TORQUE_OUTPUT_ro = PACMOD12_TORQUE_OUTPUT_ro_toS(_m->TORQUE_OUTPUT_phys);
  _m->TORQUE_INPUT_ro = PACMOD12_TORQUE_INPUT_ro_toS(_m->TORQUE_INPUT_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[0] |= ((_m->TORQUE_OUTPUT_ro >> 24) & (0xFFU));
  cframe->Data[1] |= ((_m->TORQUE_OUTPUT_ro >> 16) & (0xFFU));
  cframe->Data[2] |= ((_m->TORQUE_OUTPUT_ro >> 8) & (0xFFU));
  cframe->Data[3] |= (_m->TORQUE_OUTPUT_ro & (0xFFU));
  cframe->Data[4] |= ((_m->TORQUE_INPUT_ro >> 24) & (0xFFU));
  cframe->Data[5] |= ((_m->TORQUE_INPUT_ro >> 16) & (0xFFU));
  cframe->Data[6] |= ((_m->TORQUE_INPUT_ro >> 8) & (0xFFU));
  cframe->Data[7] |= (_m->TORQUE_INPUT_ro & (0xFFU));

  cframe->MsgId = STEERING_MOTOR_RPT_3_CANID;
  cframe->DLC = STEERING_MOTOR_RPT_3_DLC;
  cframe->IDE = STEERING_MOTOR_RPT_3_IDE;
  return STEERING_MOTOR_RPT_3_CANID;
}

#else

uint32_t Pack_STEERING_MOTOR_RPT_3_pacmod12(STEERING_MOTOR_RPT_3_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < STEERING_MOTOR_RPT_3_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->TORQUE_OUTPUT_ro = PACMOD12_TORQUE_OUTPUT_ro_toS(_m->TORQUE_OUTPUT_phys);
  _m->TORQUE_INPUT_ro = PACMOD12_TORQUE_INPUT_ro_toS(_m->TORQUE_INPUT_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[0] |= ((_m->TORQUE_OUTPUT_ro >> 24) & (0xFFU));
  _d[1] |= ((_m->TORQUE_OUTPUT_ro >> 16) & (0xFFU));
  _d[2] |= ((_m->TORQUE_OUTPUT_ro >> 8) & (0xFFU));
  _d[3] |= (_m->TORQUE_OUTPUT_ro & (0xFFU));
  _d[4] |= ((_m->TORQUE_INPUT_ro >> 24) & (0xFFU));
  _d[5] |= ((_m->TORQUE_INPUT_ro >> 16) & (0xFFU));
  _d[6] |= ((_m->TORQUE_INPUT_ro >> 8) & (0xFFU));
  _d[7] |= (_m->TORQUE_INPUT_ro & (0xFFU));

  *_len = STEERING_MOTOR_RPT_3_DLC;
  *_ide = STEERING_MOTOR_RPT_3_IDE;
  return STEERING_MOTOR_RPT_3_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_WHEEL_SPEED_RPT_pacmod12(WHEEL_SPEED_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->WHEEL_SPD_AXLE_1_LEFT_ro = ((_d[0] & (0xFFU)) << 8) | (_d[1] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->WHEEL_SPD_AXLE_1_LEFT_phys = (sigfloat_t)(PACMOD12_WHEEL_SPD_AXLE_1_LEFT_ro_fromS(_m->WHEEL_SPD_AXLE_1_LEFT_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->WHEEL_SPD_AXLE_1_RIGHT_ro = ((_d[2] & (0xFFU)) << 8) | (_d[3] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->WHEEL_SPD_AXLE_1_RIGHT_phys = (sigfloat_t)(PACMOD12_WHEEL_SPD_AXLE_1_RIGHT_ro_fromS(_m->WHEEL_SPD_AXLE_1_RIGHT_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->WHEEL_SPD_AXLE_2_LEFT_ro = ((_d[4] & (0xFFU)) << 8) | (_d[5] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->WHEEL_SPD_AXLE_2_LEFT_phys = (sigfloat_t)(PACMOD12_WHEEL_SPD_AXLE_2_LEFT_ro_fromS(_m->WHEEL_SPD_AXLE_2_LEFT_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->WHEEL_SPD_AXLE_2_RIGHT_ro = ((_d[6] & (0xFFU)) << 8) | (_d[7] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->WHEEL_SPD_AXLE_2_RIGHT_phys = (sigfloat_t)(PACMOD12_WHEEL_SPD_AXLE_2_RIGHT_ro_fromS(_m->WHEEL_SPD_AXLE_2_RIGHT_ro));
#endif // PACMOD12_USE_SIGFLOAT

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < WHEEL_SPEED_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_WHEEL_SPEED_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return WHEEL_SPEED_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_WHEEL_SPEED_RPT_pacmod12(WHEEL_SPEED_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < WHEEL_SPEED_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->WHEEL_SPD_AXLE_1_LEFT_ro = PACMOD12_WHEEL_SPD_AXLE_1_LEFT_ro_toS(_m->WHEEL_SPD_AXLE_1_LEFT_phys);
  _m->WHEEL_SPD_AXLE_1_RIGHT_ro = PACMOD12_WHEEL_SPD_AXLE_1_RIGHT_ro_toS(_m->WHEEL_SPD_AXLE_1_RIGHT_phys);
  _m->WHEEL_SPD_AXLE_2_LEFT_ro = PACMOD12_WHEEL_SPD_AXLE_2_LEFT_ro_toS(_m->WHEEL_SPD_AXLE_2_LEFT_phys);
  _m->WHEEL_SPD_AXLE_2_RIGHT_ro = PACMOD12_WHEEL_SPD_AXLE_2_RIGHT_ro_toS(_m->WHEEL_SPD_AXLE_2_RIGHT_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[0] |= ((_m->WHEEL_SPD_AXLE_1_LEFT_ro >> 8) & (0xFFU));
  cframe->Data[1] |= (_m->WHEEL_SPD_AXLE_1_LEFT_ro & (0xFFU));
  cframe->Data[2] |= ((_m->WHEEL_SPD_AXLE_1_RIGHT_ro >> 8) & (0xFFU));
  cframe->Data[3] |= (_m->WHEEL_SPD_AXLE_1_RIGHT_ro & (0xFFU));
  cframe->Data[4] |= ((_m->WHEEL_SPD_AXLE_2_LEFT_ro >> 8) & (0xFFU));
  cframe->Data[5] |= (_m->WHEEL_SPD_AXLE_2_LEFT_ro & (0xFFU));
  cframe->Data[6] |= ((_m->WHEEL_SPD_AXLE_2_RIGHT_ro >> 8) & (0xFFU));
  cframe->Data[7] |= (_m->WHEEL_SPD_AXLE_2_RIGHT_ro & (0xFFU));

  cframe->MsgId = WHEEL_SPEED_RPT_CANID;
  cframe->DLC = WHEEL_SPEED_RPT_DLC;
  cframe->IDE = WHEEL_SPEED_RPT_IDE;
  return WHEEL_SPEED_RPT_CANID;
}

#else

uint32_t Pack_WHEEL_SPEED_RPT_pacmod12(WHEEL_SPEED_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < WHEEL_SPEED_RPT_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->WHEEL_SPD_AXLE_1_LEFT_ro = PACMOD12_WHEEL_SPD_AXLE_1_LEFT_ro_toS(_m->WHEEL_SPD_AXLE_1_LEFT_phys);
  _m->WHEEL_SPD_AXLE_1_RIGHT_ro = PACMOD12_WHEEL_SPD_AXLE_1_RIGHT_ro_toS(_m->WHEEL_SPD_AXLE_1_RIGHT_phys);
  _m->WHEEL_SPD_AXLE_2_LEFT_ro = PACMOD12_WHEEL_SPD_AXLE_2_LEFT_ro_toS(_m->WHEEL_SPD_AXLE_2_LEFT_phys);
  _m->WHEEL_SPD_AXLE_2_RIGHT_ro = PACMOD12_WHEEL_SPD_AXLE_2_RIGHT_ro_toS(_m->WHEEL_SPD_AXLE_2_RIGHT_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[0] |= ((_m->WHEEL_SPD_AXLE_1_LEFT_ro >> 8) & (0xFFU));
  _d[1] |= (_m->WHEEL_SPD_AXLE_1_LEFT_ro & (0xFFU));
  _d[2] |= ((_m->WHEEL_SPD_AXLE_1_RIGHT_ro >> 8) & (0xFFU));
  _d[3] |= (_m->WHEEL_SPD_AXLE_1_RIGHT_ro & (0xFFU));
  _d[4] |= ((_m->WHEEL_SPD_AXLE_2_LEFT_ro >> 8) & (0xFFU));
  _d[5] |= (_m->WHEEL_SPD_AXLE_2_LEFT_ro & (0xFFU));
  _d[6] |= ((_m->WHEEL_SPD_AXLE_2_RIGHT_ro >> 8) & (0xFFU));
  _d[7] |= (_m->WHEEL_SPD_AXLE_2_RIGHT_ro & (0xFFU));

  *_len = WHEEL_SPEED_RPT_DLC;
  *_ide = WHEEL_SPEED_RPT_IDE;
  return WHEEL_SPEED_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_SOFTWARE_VERSION_RPT_00_pacmod12(SOFTWARE_VERSION_RPT_00_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->MAJOR = (_d[0] & (0xFFU));
  _m->MINOR = (_d[1] & (0xFFU));
  _m->PATCH = (_d[2] & (0xFFU));
  _m->BUILD0 = (_d[3] & (0xFFU));
  _m->BUILD1 = (_d[4] & (0xFFU));
  _m->BUILD2 = (_d[5] & (0xFFU));
  _m->BUILD3 = (_d[6] & (0xFFU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < SOFTWARE_VERSION_RPT_00_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_SOFTWARE_VERSION_RPT_00_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return SOFTWARE_VERSION_RPT_00_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_SOFTWARE_VERSION_RPT_00_pacmod12(SOFTWARE_VERSION_RPT_00_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < SOFTWARE_VERSION_RPT_00_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->MAJOR & (0xFFU));
  cframe->Data[1] |= (_m->MINOR & (0xFFU));
  cframe->Data[2] |= (_m->PATCH & (0xFFU));
  cframe->Data[3] |= (_m->BUILD0 & (0xFFU));
  cframe->Data[4] |= (_m->BUILD1 & (0xFFU));
  cframe->Data[5] |= (_m->BUILD2 & (0xFFU));
  cframe->Data[6] |= (_m->BUILD3 & (0xFFU));

  cframe->MsgId = SOFTWARE_VERSION_RPT_00_CANID;
  cframe->DLC = SOFTWARE_VERSION_RPT_00_DLC;
  cframe->IDE = SOFTWARE_VERSION_RPT_00_IDE;
  return SOFTWARE_VERSION_RPT_00_CANID;
}

#else

uint32_t Pack_SOFTWARE_VERSION_RPT_00_pacmod12(SOFTWARE_VERSION_RPT_00_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < SOFTWARE_VERSION_RPT_00_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->MAJOR & (0xFFU));
  _d[1] |= (_m->MINOR & (0xFFU));
  _d[2] |= (_m->PATCH & (0xFFU));
  _d[3] |= (_m->BUILD0 & (0xFFU));
  _d[4] |= (_m->BUILD1 & (0xFFU));
  _d[5] |= (_m->BUILD2 & (0xFFU));
  _d[6] |= (_m->BUILD3 & (0xFFU));

  *_len = SOFTWARE_VERSION_RPT_00_DLC;
  *_ide = SOFTWARE_VERSION_RPT_00_IDE;
  return SOFTWARE_VERSION_RPT_00_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_SOFTWARE_VERSION_RPT_01_pacmod12(SOFTWARE_VERSION_RPT_01_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->MAJOR = (_d[0] & (0xFFU));
  _m->MINOR = (_d[1] & (0xFFU));
  _m->PATCH = (_d[2] & (0xFFU));
  _m->BUILD0 = (_d[3] & (0xFFU));
  _m->BUILD1 = (_d[4] & (0xFFU));
  _m->BUILD2 = (_d[5] & (0xFFU));
  _m->BUILD3 = (_d[6] & (0xFFU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < SOFTWARE_VERSION_RPT_01_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_SOFTWARE_VERSION_RPT_01_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return SOFTWARE_VERSION_RPT_01_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_SOFTWARE_VERSION_RPT_01_pacmod12(SOFTWARE_VERSION_RPT_01_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < SOFTWARE_VERSION_RPT_01_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->MAJOR & (0xFFU));
  cframe->Data[1] |= (_m->MINOR & (0xFFU));
  cframe->Data[2] |= (_m->PATCH & (0xFFU));
  cframe->Data[3] |= (_m->BUILD0 & (0xFFU));
  cframe->Data[4] |= (_m->BUILD1 & (0xFFU));
  cframe->Data[5] |= (_m->BUILD2 & (0xFFU));
  cframe->Data[6] |= (_m->BUILD3 & (0xFFU));

  cframe->MsgId = SOFTWARE_VERSION_RPT_01_CANID;
  cframe->DLC = SOFTWARE_VERSION_RPT_01_DLC;
  cframe->IDE = SOFTWARE_VERSION_RPT_01_IDE;
  return SOFTWARE_VERSION_RPT_01_CANID;
}

#else

uint32_t Pack_SOFTWARE_VERSION_RPT_01_pacmod12(SOFTWARE_VERSION_RPT_01_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < SOFTWARE_VERSION_RPT_01_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->MAJOR & (0xFFU));
  _d[1] |= (_m->MINOR & (0xFFU));
  _d[2] |= (_m->PATCH & (0xFFU));
  _d[3] |= (_m->BUILD0 & (0xFFU));
  _d[4] |= (_m->BUILD1 & (0xFFU));
  _d[5] |= (_m->BUILD2 & (0xFFU));
  _d[6] |= (_m->BUILD3 & (0xFFU));

  *_len = SOFTWARE_VERSION_RPT_01_DLC;
  *_ide = SOFTWARE_VERSION_RPT_01_IDE;
  return SOFTWARE_VERSION_RPT_01_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_SOFTWARE_VERSION_RPT_02_pacmod12(SOFTWARE_VERSION_RPT_02_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->MAJOR = (_d[0] & (0xFFU));
  _m->MINOR = (_d[1] & (0xFFU));
  _m->PATCH = (_d[2] & (0xFFU));
  _m->BUILD0 = (_d[3] & (0xFFU));
  _m->BUILD1 = (_d[4] & (0xFFU));
  _m->BUILD2 = (_d[5] & (0xFFU));
  _m->BUILD3 = (_d[6] & (0xFFU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < SOFTWARE_VERSION_RPT_02_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_SOFTWARE_VERSION_RPT_02_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return SOFTWARE_VERSION_RPT_02_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_SOFTWARE_VERSION_RPT_02_pacmod12(SOFTWARE_VERSION_RPT_02_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < SOFTWARE_VERSION_RPT_02_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->MAJOR & (0xFFU));
  cframe->Data[1] |= (_m->MINOR & (0xFFU));
  cframe->Data[2] |= (_m->PATCH & (0xFFU));
  cframe->Data[3] |= (_m->BUILD0 & (0xFFU));
  cframe->Data[4] |= (_m->BUILD1 & (0xFFU));
  cframe->Data[5] |= (_m->BUILD2 & (0xFFU));
  cframe->Data[6] |= (_m->BUILD3 & (0xFFU));

  cframe->MsgId = SOFTWARE_VERSION_RPT_02_CANID;
  cframe->DLC = SOFTWARE_VERSION_RPT_02_DLC;
  cframe->IDE = SOFTWARE_VERSION_RPT_02_IDE;
  return SOFTWARE_VERSION_RPT_02_CANID;
}

#else

uint32_t Pack_SOFTWARE_VERSION_RPT_02_pacmod12(SOFTWARE_VERSION_RPT_02_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < SOFTWARE_VERSION_RPT_02_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->MAJOR & (0xFFU));
  _d[1] |= (_m->MINOR & (0xFFU));
  _d[2] |= (_m->PATCH & (0xFFU));
  _d[3] |= (_m->BUILD0 & (0xFFU));
  _d[4] |= (_m->BUILD1 & (0xFFU));
  _d[5] |= (_m->BUILD2 & (0xFFU));
  _d[6] |= (_m->BUILD3 & (0xFFU));

  *_len = SOFTWARE_VERSION_RPT_02_DLC;
  *_ide = SOFTWARE_VERSION_RPT_02_IDE;
  return SOFTWARE_VERSION_RPT_02_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_SOFTWARE_VERSION_RPT_03_pacmod12(SOFTWARE_VERSION_RPT_03_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->MAJOR = (_d[0] & (0xFFU));
  _m->MINOR = (_d[1] & (0xFFU));
  _m->PATCH = (_d[2] & (0xFFU));
  _m->BUILD0 = (_d[3] & (0xFFU));
  _m->BUILD1 = (_d[4] & (0xFFU));
  _m->BUILD2 = (_d[5] & (0xFFU));
  _m->BUILD3 = (_d[6] & (0xFFU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < SOFTWARE_VERSION_RPT_03_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_SOFTWARE_VERSION_RPT_03_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return SOFTWARE_VERSION_RPT_03_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_SOFTWARE_VERSION_RPT_03_pacmod12(SOFTWARE_VERSION_RPT_03_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < SOFTWARE_VERSION_RPT_03_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->MAJOR & (0xFFU));
  cframe->Data[1] |= (_m->MINOR & (0xFFU));
  cframe->Data[2] |= (_m->PATCH & (0xFFU));
  cframe->Data[3] |= (_m->BUILD0 & (0xFFU));
  cframe->Data[4] |= (_m->BUILD1 & (0xFFU));
  cframe->Data[5] |= (_m->BUILD2 & (0xFFU));
  cframe->Data[6] |= (_m->BUILD3 & (0xFFU));

  cframe->MsgId = SOFTWARE_VERSION_RPT_03_CANID;
  cframe->DLC = SOFTWARE_VERSION_RPT_03_DLC;
  cframe->IDE = SOFTWARE_VERSION_RPT_03_IDE;
  return SOFTWARE_VERSION_RPT_03_CANID;
}

#else

uint32_t Pack_SOFTWARE_VERSION_RPT_03_pacmod12(SOFTWARE_VERSION_RPT_03_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < SOFTWARE_VERSION_RPT_03_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->MAJOR & (0xFFU));
  _d[1] |= (_m->MINOR & (0xFFU));
  _d[2] |= (_m->PATCH & (0xFFU));
  _d[3] |= (_m->BUILD0 & (0xFFU));
  _d[4] |= (_m->BUILD1 & (0xFFU));
  _d[5] |= (_m->BUILD2 & (0xFFU));
  _d[6] |= (_m->BUILD3 & (0xFFU));

  *_len = SOFTWARE_VERSION_RPT_03_DLC;
  *_ide = SOFTWARE_VERSION_RPT_03_IDE;
  return SOFTWARE_VERSION_RPT_03_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_SOFTWARE_VERSION_RPT_04_pacmod12(SOFTWARE_VERSION_RPT_04_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->MAJOR = (_d[0] & (0xFFU));
  _m->MINOR = (_d[1] & (0xFFU));
  _m->PATCH = (_d[2] & (0xFFU));
  _m->BUILD0 = (_d[3] & (0xFFU));
  _m->BUILD1 = (_d[4] & (0xFFU));
  _m->BUILD2 = (_d[5] & (0xFFU));
  _m->BUILD3 = (_d[6] & (0xFFU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < SOFTWARE_VERSION_RPT_04_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_SOFTWARE_VERSION_RPT_04_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return SOFTWARE_VERSION_RPT_04_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_SOFTWARE_VERSION_RPT_04_pacmod12(SOFTWARE_VERSION_RPT_04_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < SOFTWARE_VERSION_RPT_04_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->MAJOR & (0xFFU));
  cframe->Data[1] |= (_m->MINOR & (0xFFU));
  cframe->Data[2] |= (_m->PATCH & (0xFFU));
  cframe->Data[3] |= (_m->BUILD0 & (0xFFU));
  cframe->Data[4] |= (_m->BUILD1 & (0xFFU));
  cframe->Data[5] |= (_m->BUILD2 & (0xFFU));
  cframe->Data[6] |= (_m->BUILD3 & (0xFFU));

  cframe->MsgId = SOFTWARE_VERSION_RPT_04_CANID;
  cframe->DLC = SOFTWARE_VERSION_RPT_04_DLC;
  cframe->IDE = SOFTWARE_VERSION_RPT_04_IDE;
  return SOFTWARE_VERSION_RPT_04_CANID;
}

#else

uint32_t Pack_SOFTWARE_VERSION_RPT_04_pacmod12(SOFTWARE_VERSION_RPT_04_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < SOFTWARE_VERSION_RPT_04_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->MAJOR & (0xFFU));
  _d[1] |= (_m->MINOR & (0xFFU));
  _d[2] |= (_m->PATCH & (0xFFU));
  _d[3] |= (_m->BUILD0 & (0xFFU));
  _d[4] |= (_m->BUILD1 & (0xFFU));
  _d[5] |= (_m->BUILD2 & (0xFFU));
  _d[6] |= (_m->BUILD3 & (0xFFU));

  *_len = SOFTWARE_VERSION_RPT_04_DLC;
  *_ide = SOFTWARE_VERSION_RPT_04_IDE;
  return SOFTWARE_VERSION_RPT_04_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_YAW_RATE_RPT_pacmod12(YAW_RATE_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->YAW_RATE_ro = ((_d[0] & (0xFFU)) << 8) | (_d[1] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->YAW_RATE_phys = (sigfloat_t)(PACMOD12_YAW_RATE_ro_fromS(_m->YAW_RATE_ro));
#endif // PACMOD12_USE_SIGFLOAT

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < YAW_RATE_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_YAW_RATE_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return YAW_RATE_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_YAW_RATE_RPT_pacmod12(YAW_RATE_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < YAW_RATE_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->YAW_RATE_ro = PACMOD12_YAW_RATE_ro_toS(_m->YAW_RATE_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[0] |= ((_m->YAW_RATE_ro >> 8) & (0xFFU));
  cframe->Data[1] |= (_m->YAW_RATE_ro & (0xFFU));

  cframe->MsgId = YAW_RATE_RPT_CANID;
  cframe->DLC = YAW_RATE_RPT_DLC;
  cframe->IDE = YAW_RATE_RPT_IDE;
  return YAW_RATE_RPT_CANID;
}

#else

uint32_t Pack_YAW_RATE_RPT_pacmod12(YAW_RATE_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < YAW_RATE_RPT_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->YAW_RATE_ro = PACMOD12_YAW_RATE_ro_toS(_m->YAW_RATE_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[0] |= ((_m->YAW_RATE_ro >> 8) & (0xFFU));
  _d[1] |= (_m->YAW_RATE_ro & (0xFFU));

  *_len = YAW_RATE_RPT_DLC;
  *_ide = YAW_RATE_RPT_IDE;
  return YAW_RATE_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_LAT_LON_HEADING_RPT_pacmod12(LAT_LON_HEADING_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->LATITUDE_DEGREES = (_d[0] & (0xFFU));
  _m->LATITUDE_MINUTES = (_d[1] & (0xFFU));
  _m->LATITUDE_SECONDS = (_d[2] & (0xFFU));
  _m->LONGITUDE_DEGREES = (_d[3] & (0xFFU));
  _m->LONGITUDE_MINUTES = (_d[4] & (0xFFU));
  _m->LONGITUDE_SECONDS = (_d[5] & (0xFFU));
  _m->HEADING_ro = ((_d[6] & (0xFFU)) << 8) | (_d[7] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->HEADING_phys = (sigfloat_t)(PACMOD12_HEADING_ro_fromS(_m->HEADING_ro));
#endif // PACMOD12_USE_SIGFLOAT

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < LAT_LON_HEADING_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_LAT_LON_HEADING_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return LAT_LON_HEADING_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_LAT_LON_HEADING_RPT_pacmod12(LAT_LON_HEADING_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < LAT_LON_HEADING_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->HEADING_ro = PACMOD12_HEADING_ro_toS(_m->HEADING_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[0] |= (_m->LATITUDE_DEGREES & (0xFFU));
  cframe->Data[1] |= (_m->LATITUDE_MINUTES & (0xFFU));
  cframe->Data[2] |= (_m->LATITUDE_SECONDS & (0xFFU));
  cframe->Data[3] |= (_m->LONGITUDE_DEGREES & (0xFFU));
  cframe->Data[4] |= (_m->LONGITUDE_MINUTES & (0xFFU));
  cframe->Data[5] |= (_m->LONGITUDE_SECONDS & (0xFFU));
  cframe->Data[6] |= ((_m->HEADING_ro >> 8) & (0xFFU));
  cframe->Data[7] |= (_m->HEADING_ro & (0xFFU));

  cframe->MsgId = LAT_LON_HEADING_RPT_CANID;
  cframe->DLC = LAT_LON_HEADING_RPT_DLC;
  cframe->IDE = LAT_LON_HEADING_RPT_IDE;
  return LAT_LON_HEADING_RPT_CANID;
}

#else

uint32_t Pack_LAT_LON_HEADING_RPT_pacmod12(LAT_LON_HEADING_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < LAT_LON_HEADING_RPT_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->HEADING_ro = PACMOD12_HEADING_ro_toS(_m->HEADING_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[0] |= (_m->LATITUDE_DEGREES & (0xFFU));
  _d[1] |= (_m->LATITUDE_MINUTES & (0xFFU));
  _d[2] |= (_m->LATITUDE_SECONDS & (0xFFU));
  _d[3] |= (_m->LONGITUDE_DEGREES & (0xFFU));
  _d[4] |= (_m->LONGITUDE_MINUTES & (0xFFU));
  _d[5] |= (_m->LONGITUDE_SECONDS & (0xFFU));
  _d[6] |= ((_m->HEADING_ro >> 8) & (0xFFU));
  _d[7] |= (_m->HEADING_ro & (0xFFU));

  *_len = LAT_LON_HEADING_RPT_DLC;
  *_ide = LAT_LON_HEADING_RPT_IDE;
  return LAT_LON_HEADING_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_DATE_TIME_RPT_pacmod12(DATE_TIME_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->DATE_YEAR_ro = (_d[0] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->DATE_YEAR_phys = PACMOD12_DATE_YEAR_ro_fromS(_m->DATE_YEAR_ro);
#endif // PACMOD12_USE_SIGFLOAT

  _m->DATE_MONTH_ro = (_d[1] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->DATE_MONTH_phys = PACMOD12_DATE_MONTH_ro_fromS(_m->DATE_MONTH_ro);
#endif // PACMOD12_USE_SIGFLOAT

  _m->DATE_DAY_ro = (_d[2] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->DATE_DAY_phys = PACMOD12_DATE_DAY_ro_fromS(_m->DATE_DAY_ro);
#endif // PACMOD12_USE_SIGFLOAT

  _m->TIME_HOUR = (_d[3] & (0xFFU));
  _m->TIME_MINUTE = (_d[4] & (0xFFU));
  _m->TIME_SECOND = (_d[5] & (0xFFU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < DATE_TIME_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_DATE_TIME_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return DATE_TIME_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_DATE_TIME_RPT_pacmod12(DATE_TIME_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < DATE_TIME_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->DATE_YEAR_ro = PACMOD12_DATE_YEAR_ro_toS(_m->DATE_YEAR_phys);
  _m->DATE_MONTH_ro = PACMOD12_DATE_MONTH_ro_toS(_m->DATE_MONTH_phys);
  _m->DATE_DAY_ro = PACMOD12_DATE_DAY_ro_toS(_m->DATE_DAY_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[0] |= (_m->DATE_YEAR_ro & (0xFFU));
  cframe->Data[1] |= (_m->DATE_MONTH_ro & (0xFFU));
  cframe->Data[2] |= (_m->DATE_DAY_ro & (0xFFU));
  cframe->Data[3] |= (_m->TIME_HOUR & (0xFFU));
  cframe->Data[4] |= (_m->TIME_MINUTE & (0xFFU));
  cframe->Data[5] |= (_m->TIME_SECOND & (0xFFU));

  cframe->MsgId = DATE_TIME_RPT_CANID;
  cframe->DLC = DATE_TIME_RPT_DLC;
  cframe->IDE = DATE_TIME_RPT_IDE;
  return DATE_TIME_RPT_CANID;
}

#else

uint32_t Pack_DATE_TIME_RPT_pacmod12(DATE_TIME_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < DATE_TIME_RPT_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->DATE_YEAR_ro = PACMOD12_DATE_YEAR_ro_toS(_m->DATE_YEAR_phys);
  _m->DATE_MONTH_ro = PACMOD12_DATE_MONTH_ro_toS(_m->DATE_MONTH_phys);
  _m->DATE_DAY_ro = PACMOD12_DATE_DAY_ro_toS(_m->DATE_DAY_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[0] |= (_m->DATE_YEAR_ro & (0xFFU));
  _d[1] |= (_m->DATE_MONTH_ro & (0xFFU));
  _d[2] |= (_m->DATE_DAY_ro & (0xFFU));
  _d[3] |= (_m->TIME_HOUR & (0xFFU));
  _d[4] |= (_m->TIME_MINUTE & (0xFFU));
  _d[5] |= (_m->TIME_SECOND & (0xFFU));

  *_len = DATE_TIME_RPT_DLC;
  *_ide = DATE_TIME_RPT_IDE;
  return DATE_TIME_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_ENGINE_RPT_pacmod12(ENGINE_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ENGINE_SPEED_ro = ((_d[0] & (0xFFU)) << 8) | (_d[1] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->ENGINE_SPEED_phys = (sigfloat_t)(PACMOD12_ENGINE_SPEED_ro_fromS(_m->ENGINE_SPEED_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->ENGINE_TORQUE_ro = ((_d[2] & (0xFFU)) << 8) | (_d[3] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->ENGINE_TORQUE_phys = (sigfloat_t)(PACMOD12_ENGINE_TORQUE_ro_fromS(_m->ENGINE_TORQUE_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->ENGINE_COOLANT_TEMP_ro = (_d[4] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->ENGINE_COOLANT_TEMP_phys = PACMOD12_ENGINE_COOLANT_TEMP_ro_fromS(_m->ENGINE_COOLANT_TEMP_ro);
#endif // PACMOD12_USE_SIGFLOAT

  _m->ENGINE_SPEED_AVAIL = (_d[5] & (0x01U));
  _m->ENGINE_TORQUE_AVAIL = ((_d[5] >> 1) & (0x01U));
  _m->ENGINE_COOLANT_TEMP_AVAIL = ((_d[5] >> 2) & (0x01U));
  _m->FUEL_LEVEL_AVAIL = ((_d[5] >> 3) & (0x01U));
  _m->FUEL_LEVEL_ro = (_d[6] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->FUEL_LEVEL_phys = (sigfloat_t)(PACMOD12_FUEL_LEVEL_ro_fromS(_m->FUEL_LEVEL_ro));
#endif // PACMOD12_USE_SIGFLOAT

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < ENGINE_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_ENGINE_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return ENGINE_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_ENGINE_RPT_pacmod12(ENGINE_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < ENGINE_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->ENGINE_SPEED_ro = PACMOD12_ENGINE_SPEED_ro_toS(_m->ENGINE_SPEED_phys);
  _m->ENGINE_TORQUE_ro = PACMOD12_ENGINE_TORQUE_ro_toS(_m->ENGINE_TORQUE_phys);
  _m->ENGINE_COOLANT_TEMP_ro = PACMOD12_ENGINE_COOLANT_TEMP_ro_toS(_m->ENGINE_COOLANT_TEMP_phys);
  _m->FUEL_LEVEL_ro = PACMOD12_FUEL_LEVEL_ro_toS(_m->FUEL_LEVEL_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[0] |= ((_m->ENGINE_SPEED_ro >> 8) & (0xFFU));
  cframe->Data[1] |= (_m->ENGINE_SPEED_ro & (0xFFU));
  cframe->Data[2] |= ((_m->ENGINE_TORQUE_ro >> 8) & (0xFFU));
  cframe->Data[3] |= (_m->ENGINE_TORQUE_ro & (0xFFU));
  cframe->Data[4] |= (_m->ENGINE_COOLANT_TEMP_ro & (0xFFU));
  cframe->Data[5] |= (_m->ENGINE_SPEED_AVAIL & (0x01U)) | ((_m->ENGINE_TORQUE_AVAIL & (0x01U)) << 1) | ((_m->ENGINE_COOLANT_TEMP_AVAIL & (0x01U)) << 2) | ((_m->FUEL_LEVEL_AVAIL & (0x01U)) << 3);
  cframe->Data[6] |= (_m->FUEL_LEVEL_ro & (0xFFU));

  cframe->MsgId = ENGINE_RPT_CANID;
  cframe->DLC = ENGINE_RPT_DLC;
  cframe->IDE = ENGINE_RPT_IDE;
  return ENGINE_RPT_CANID;
}

#else

uint32_t Pack_ENGINE_RPT_pacmod12(ENGINE_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < ENGINE_RPT_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->ENGINE_SPEED_ro = PACMOD12_ENGINE_SPEED_ro_toS(_m->ENGINE_SPEED_phys);
  _m->ENGINE_TORQUE_ro = PACMOD12_ENGINE_TORQUE_ro_toS(_m->ENGINE_TORQUE_phys);
  _m->ENGINE_COOLANT_TEMP_ro = PACMOD12_ENGINE_COOLANT_TEMP_ro_toS(_m->ENGINE_COOLANT_TEMP_phys);
  _m->FUEL_LEVEL_ro = PACMOD12_FUEL_LEVEL_ro_toS(_m->FUEL_LEVEL_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[0] |= ((_m->ENGINE_SPEED_ro >> 8) & (0xFFU));
  _d[1] |= (_m->ENGINE_SPEED_ro & (0xFFU));
  _d[2] |= ((_m->ENGINE_TORQUE_ro >> 8) & (0xFFU));
  _d[3] |= (_m->ENGINE_TORQUE_ro & (0xFFU));
  _d[4] |= (_m->ENGINE_COOLANT_TEMP_ro & (0xFFU));
  _d[5] |= (_m->ENGINE_SPEED_AVAIL & (0x01U)) | ((_m->ENGINE_TORQUE_AVAIL & (0x01U)) << 1) | ((_m->ENGINE_COOLANT_TEMP_AVAIL & (0x01U)) << 2) | ((_m->FUEL_LEVEL_AVAIL & (0x01U)) << 3);
  _d[6] |= (_m->FUEL_LEVEL_ro & (0xFFU));

  *_len = ENGINE_RPT_DLC;
  *_ide = ENGINE_RPT_IDE;
  return ENGINE_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_DETECTED_OBJECT_RPT_pacmod12(DETECTED_OBJECT_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->FRONT_OBJECT_DISTANCE_LOW_RES_ro = ((_d[0] & (0xFFU)) << 16) | ((_d[1] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->FRONT_OBJECT_DISTANCE_LOW_RES_phys = (sigfloat_t)(PACMOD12_FRONT_OBJECT_DISTANCE_LOW_RES_ro_fromS(_m->FRONT_OBJECT_DISTANCE_LOW_RES_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->FRONT_OBJECT_DISTANCE_HIGH_RES_ro = ((_d[3] & (0xFFU)) << 16) | ((_d[4] & (0xFFU)) << 8) | (_d[5] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->FRONT_OBJECT_DISTANCE_HIGH_RES_phys = (sigfloat_t)(PACMOD12_FRONT_OBJECT_DISTANCE_HIGH_RES_ro_fromS(_m->FRONT_OBJECT_DISTANCE_HIGH_RES_ro));
#endif // PACMOD12_USE_SIGFLOAT

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < DETECTED_OBJECT_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_DETECTED_OBJECT_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return DETECTED_OBJECT_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_DETECTED_OBJECT_RPT_pacmod12(DETECTED_OBJECT_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < DETECTED_OBJECT_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->FRONT_OBJECT_DISTANCE_LOW_RES_ro = PACMOD12_FRONT_OBJECT_DISTANCE_LOW_RES_ro_toS(_m->FRONT_OBJECT_DISTANCE_LOW_RES_phys);
  _m->FRONT_OBJECT_DISTANCE_HIGH_RES_ro = PACMOD12_FRONT_OBJECT_DISTANCE_HIGH_RES_ro_toS(_m->FRONT_OBJECT_DISTANCE_HIGH_RES_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[0] |= ((_m->FRONT_OBJECT_DISTANCE_LOW_RES_ro >> 16) & (0xFFU));
  cframe->Data[1] |= ((_m->FRONT_OBJECT_DISTANCE_LOW_RES_ro >> 8) & (0xFFU));
  cframe->Data[2] |= (_m->FRONT_OBJECT_DISTANCE_LOW_RES_ro & (0xFFU));
  cframe->Data[3] |= ((_m->FRONT_OBJECT_DISTANCE_HIGH_RES_ro >> 16) & (0xFFU));
  cframe->Data[4] |= ((_m->FRONT_OBJECT_DISTANCE_HIGH_RES_ro >> 8) & (0xFFU));
  cframe->Data[5] |= (_m->FRONT_OBJECT_DISTANCE_HIGH_RES_ro & (0xFFU));

  cframe->MsgId = DETECTED_OBJECT_RPT_CANID;
  cframe->DLC = DETECTED_OBJECT_RPT_DLC;
  cframe->IDE = DETECTED_OBJECT_RPT_IDE;
  return DETECTED_OBJECT_RPT_CANID;
}

#else

uint32_t Pack_DETECTED_OBJECT_RPT_pacmod12(DETECTED_OBJECT_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < DETECTED_OBJECT_RPT_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->FRONT_OBJECT_DISTANCE_LOW_RES_ro = PACMOD12_FRONT_OBJECT_DISTANCE_LOW_RES_ro_toS(_m->FRONT_OBJECT_DISTANCE_LOW_RES_phys);
  _m->FRONT_OBJECT_DISTANCE_HIGH_RES_ro = PACMOD12_FRONT_OBJECT_DISTANCE_HIGH_RES_ro_toS(_m->FRONT_OBJECT_DISTANCE_HIGH_RES_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[0] |= ((_m->FRONT_OBJECT_DISTANCE_LOW_RES_ro >> 16) & (0xFFU));
  _d[1] |= ((_m->FRONT_OBJECT_DISTANCE_LOW_RES_ro >> 8) & (0xFFU));
  _d[2] |= (_m->FRONT_OBJECT_DISTANCE_LOW_RES_ro & (0xFFU));
  _d[3] |= ((_m->FRONT_OBJECT_DISTANCE_HIGH_RES_ro >> 16) & (0xFFU));
  _d[4] |= ((_m->FRONT_OBJECT_DISTANCE_HIGH_RES_ro >> 8) & (0xFFU));
  _d[5] |= (_m->FRONT_OBJECT_DISTANCE_HIGH_RES_ro & (0xFFU));

  *_len = DETECTED_OBJECT_RPT_DLC;
  *_ide = DETECTED_OBJECT_RPT_IDE;
  return DETECTED_OBJECT_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_VEH_DYNAMICS_RPT_pacmod12(VEH_DYNAMICS_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->VEH_G_FORCES_ro = (_d[0] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->VEH_G_FORCES_phys = (sigfloat_t)(PACMOD12_VEH_G_FORCES_ro_fromS(_m->VEH_G_FORCES_ro));
#endif // PACMOD12_USE_SIGFLOAT

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < VEH_DYNAMICS_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_VEH_DYNAMICS_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return VEH_DYNAMICS_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_VEH_DYNAMICS_RPT_pacmod12(VEH_DYNAMICS_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < VEH_DYNAMICS_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->VEH_G_FORCES_ro = PACMOD12_VEH_G_FORCES_ro_toS(_m->VEH_G_FORCES_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[0] |= (_m->VEH_G_FORCES_ro & (0xFFU));

  cframe->MsgId = VEH_DYNAMICS_RPT_CANID;
  cframe->DLC = VEH_DYNAMICS_RPT_DLC;
  cframe->IDE = VEH_DYNAMICS_RPT_IDE;
  return VEH_DYNAMICS_RPT_CANID;
}

#else

uint32_t Pack_VEH_DYNAMICS_RPT_pacmod12(VEH_DYNAMICS_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < VEH_DYNAMICS_RPT_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->VEH_G_FORCES_ro = PACMOD12_VEH_G_FORCES_ro_toS(_m->VEH_G_FORCES_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[0] |= (_m->VEH_G_FORCES_ro & (0xFFU));

  *_len = VEH_DYNAMICS_RPT_DLC;
  *_ide = VEH_DYNAMICS_RPT_IDE;
  return VEH_DYNAMICS_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_VIN_RPT_pacmod12(VIN_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->VEH_MFG_CODE = ((_d[0] & (0xFFU)) << 16) | ((_d[1] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
  _m->VEH_MY_CODE = (_d[3] & (0xFFU));
  _m->VEH_SERIAL = ((_d[4] & (0xFFU)) << 16) | ((_d[5] & (0xFFU)) << 8) | (_d[6] & (0xFFU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < VIN_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_VIN_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return VIN_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_VIN_RPT_pacmod12(VIN_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < VIN_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= ((_m->VEH_MFG_CODE >> 16) & (0xFFU));
  cframe->Data[1] |= ((_m->VEH_MFG_CODE >> 8) & (0xFFU));
  cframe->Data[2] |= (_m->VEH_MFG_CODE & (0xFFU));
  cframe->Data[3] |= (_m->VEH_MY_CODE & (0xFFU));
  cframe->Data[4] |= ((_m->VEH_SERIAL >> 16) & (0xFFU));
  cframe->Data[5] |= ((_m->VEH_SERIAL >> 8) & (0xFFU));
  cframe->Data[6] |= (_m->VEH_SERIAL & (0xFFU));

  cframe->MsgId = VIN_RPT_CANID;
  cframe->DLC = VIN_RPT_DLC;
  cframe->IDE = VIN_RPT_IDE;
  return VIN_RPT_CANID;
}

#else

uint32_t Pack_VIN_RPT_pacmod12(VIN_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < VIN_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= ((_m->VEH_MFG_CODE >> 16) & (0xFFU));
  _d[1] |= ((_m->VEH_MFG_CODE >> 8) & (0xFFU));
  _d[2] |= (_m->VEH_MFG_CODE & (0xFFU));
  _d[3] |= (_m->VEH_MY_CODE & (0xFFU));
  _d[4] |= ((_m->VEH_SERIAL >> 16) & (0xFFU));
  _d[5] |= ((_m->VEH_SERIAL >> 8) & (0xFFU));
  _d[6] |= (_m->VEH_SERIAL & (0xFFU));

  *_len = VIN_RPT_DLC;
  *_ide = VIN_RPT_IDE;
  return VIN_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_OCCUPANCY_RPT_pacmod12(OCCUPANCY_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->DRIVER_SEAT_OCCUPIED = (_d[0] & (0x01U));
  _m->PASS_SEAT_OCCUPIED = ((_d[0] >> 1) & (0x01U));
  _m->REAR_SEAT_OCCUPIED = ((_d[0] >> 2) & (0x01U));
  _m->DRIVER_SEATBELT_BUCKLED = ((_d[0] >> 3) & (0x01U));
  _m->PASS_SEATBELT_BUCKLED = ((_d[0] >> 4) & (0x01U));
  _m->DRVR_REAR_SEATBELT_BUCKLED = ((_d[0] >> 5) & (0x01U));
  _m->PASS_REAR_SEATBELT_BUCKLED = ((_d[0] >> 6) & (0x01U));
  _m->CTR_REAR_SEATBELT_BUCKLED = ((_d[0] >> 7) & (0x01U));
  _m->DRIVER_SEAT_OCCUPIED_AVAIL = (_d[1] & (0x01U));
  _m->PASS_SEAT_OCCUPIED_AVAIL = ((_d[1] >> 1) & (0x01U));
  _m->REAR_SEAT_OCCUPIED_AVAIL = ((_d[1] >> 2) & (0x01U));
  _m->DRIVER_SEATBELT_BUCKLED_AVAIL = ((_d[1] >> 3) & (0x01U));
  _m->PASS_SEATBELT_BUCKLED_AVAIL = ((_d[1] >> 4) & (0x01U));
  _m->DRVR_REAR_SEATBELT_BUCKLED_AVAIL = ((_d[1] >> 5) & (0x01U));
  _m->PASS_REAR_SEATBELT_BUCKLED_AVAIL = ((_d[1] >> 6) & (0x01U));
  _m->CTR_REAR_SEATBELT_BUCKLED_AVAIL = ((_d[1] >> 7) & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < OCCUPANCY_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_OCCUPANCY_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return OCCUPANCY_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_OCCUPANCY_RPT_pacmod12(OCCUPANCY_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < OCCUPANCY_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->DRIVER_SEAT_OCCUPIED & (0x01U)) | ((_m->PASS_SEAT_OCCUPIED & (0x01U)) << 1) | ((_m->REAR_SEAT_OCCUPIED & (0x01U)) << 2) | ((_m->DRIVER_SEATBELT_BUCKLED & (0x01U)) << 3) | ((_m->PASS_SEATBELT_BUCKLED & (0x01U)) << 4) | ((_m->DRVR_REAR_SEATBELT_BUCKLED & (0x01U)) << 5) | ((_m->PASS_REAR_SEATBELT_BUCKLED & (0x01U)) << 6) | ((_m->CTR_REAR_SEATBELT_BUCKLED & (0x01U)) << 7);
  cframe->Data[1] |= (_m->DRIVER_SEAT_OCCUPIED_AVAIL & (0x01U)) | ((_m->PASS_SEAT_OCCUPIED_AVAIL & (0x01U)) << 1) | ((_m->REAR_SEAT_OCCUPIED_AVAIL & (0x01U)) << 2) | ((_m->DRIVER_SEATBELT_BUCKLED_AVAIL & (0x01U)) << 3) | ((_m->PASS_SEATBELT_BUCKLED_AVAIL & (0x01U)) << 4) | ((_m->DRVR_REAR_SEATBELT_BUCKLED_AVAIL & (0x01U)) << 5) | ((_m->PASS_REAR_SEATBELT_BUCKLED_AVAIL & (0x01U)) << 6) | ((_m->CTR_REAR_SEATBELT_BUCKLED_AVAIL & (0x01U)) << 7);

  cframe->MsgId = OCCUPANCY_RPT_CANID;
  cframe->DLC = OCCUPANCY_RPT_DLC;
  cframe->IDE = OCCUPANCY_RPT_IDE;
  return OCCUPANCY_RPT_CANID;
}

#else

uint32_t Pack_OCCUPANCY_RPT_pacmod12(OCCUPANCY_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < OCCUPANCY_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->DRIVER_SEAT_OCCUPIED & (0x01U)) | ((_m->PASS_SEAT_OCCUPIED & (0x01U)) << 1) | ((_m->REAR_SEAT_OCCUPIED & (0x01U)) << 2) | ((_m->DRIVER_SEATBELT_BUCKLED & (0x01U)) << 3) | ((_m->PASS_SEATBELT_BUCKLED & (0x01U)) << 4) | ((_m->DRVR_REAR_SEATBELT_BUCKLED & (0x01U)) << 5) | ((_m->PASS_REAR_SEATBELT_BUCKLED & (0x01U)) << 6) | ((_m->CTR_REAR_SEATBELT_BUCKLED & (0x01U)) << 7);
  _d[1] |= (_m->DRIVER_SEAT_OCCUPIED_AVAIL & (0x01U)) | ((_m->PASS_SEAT_OCCUPIED_AVAIL & (0x01U)) << 1) | ((_m->REAR_SEAT_OCCUPIED_AVAIL & (0x01U)) << 2) | ((_m->DRIVER_SEATBELT_BUCKLED_AVAIL & (0x01U)) << 3) | ((_m->PASS_SEATBELT_BUCKLED_AVAIL & (0x01U)) << 4) | ((_m->DRVR_REAR_SEATBELT_BUCKLED_AVAIL & (0x01U)) << 5) | ((_m->PASS_REAR_SEATBELT_BUCKLED_AVAIL & (0x01U)) << 6) | ((_m->CTR_REAR_SEATBELT_BUCKLED_AVAIL & (0x01U)) << 7);

  *_len = OCCUPANCY_RPT_DLC;
  *_ide = OCCUPANCY_RPT_IDE;
  return OCCUPANCY_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_INTERIOR_LIGHTS_RPT_pacmod12(INTERIOR_LIGHTS_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->FRONT_DOME_LIGHTS_ON = (_d[0] & (0x01U));
  _m->REAR_DOME_LIGHTS_ON = ((_d[0] >> 1) & (0x01U));
  _m->MOOD_LIGHTS_ON = ((_d[0] >> 2) & (0x01U));
  _m->AMBIENT_LIGHT_SENSOR = ((_d[0] >> 3) & (0x01U));
  _m->DIM_LEVEL = (_d[1] & (0xFFU));
  _m->FRONT_DOME_LIGHTS_ON_AVAIL = (_d[2] & (0x01U));
  _m->REAR_DOME_LIGHTS_ON_AVAIL = ((_d[2] >> 1) & (0x01U));
  _m->MOOD_LIGHTS_ON_AVAIL = ((_d[2] >> 2) & (0x01U));
  _m->DIM_LEVEL_AVAIL = ((_d[2] >> 3) & (0x01U));
  _m->AMBIENT_LIGHT_SENSOR_AVAIL = ((_d[2] >> 4) & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INTERIOR_LIGHTS_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INTERIOR_LIGHTS_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return INTERIOR_LIGHTS_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_INTERIOR_LIGHTS_RPT_pacmod12(INTERIOR_LIGHTS_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < INTERIOR_LIGHTS_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->FRONT_DOME_LIGHTS_ON & (0x01U)) | ((_m->REAR_DOME_LIGHTS_ON & (0x01U)) << 1) | ((_m->MOOD_LIGHTS_ON & (0x01U)) << 2) | ((_m->AMBIENT_LIGHT_SENSOR & (0x01U)) << 3);
  cframe->Data[1] |= (_m->DIM_LEVEL & (0xFFU));
  cframe->Data[2] |= (_m->FRONT_DOME_LIGHTS_ON_AVAIL & (0x01U)) | ((_m->REAR_DOME_LIGHTS_ON_AVAIL & (0x01U)) << 1) | ((_m->MOOD_LIGHTS_ON_AVAIL & (0x01U)) << 2) | ((_m->DIM_LEVEL_AVAIL & (0x01U)) << 3) | ((_m->AMBIENT_LIGHT_SENSOR_AVAIL & (0x01U)) << 4);

  cframe->MsgId = INTERIOR_LIGHTS_RPT_CANID;
  cframe->DLC = INTERIOR_LIGHTS_RPT_DLC;
  cframe->IDE = INTERIOR_LIGHTS_RPT_IDE;
  return INTERIOR_LIGHTS_RPT_CANID;
}

#else

uint32_t Pack_INTERIOR_LIGHTS_RPT_pacmod12(INTERIOR_LIGHTS_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < INTERIOR_LIGHTS_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->FRONT_DOME_LIGHTS_ON & (0x01U)) | ((_m->REAR_DOME_LIGHTS_ON & (0x01U)) << 1) | ((_m->MOOD_LIGHTS_ON & (0x01U)) << 2) | ((_m->AMBIENT_LIGHT_SENSOR & (0x01U)) << 3);
  _d[1] |= (_m->DIM_LEVEL & (0xFFU));
  _d[2] |= (_m->FRONT_DOME_LIGHTS_ON_AVAIL & (0x01U)) | ((_m->REAR_DOME_LIGHTS_ON_AVAIL & (0x01U)) << 1) | ((_m->MOOD_LIGHTS_ON_AVAIL & (0x01U)) << 2) | ((_m->DIM_LEVEL_AVAIL & (0x01U)) << 3) | ((_m->AMBIENT_LIGHT_SENSOR_AVAIL & (0x01U)) << 4);

  *_len = INTERIOR_LIGHTS_RPT_DLC;
  *_ide = INTERIOR_LIGHTS_RPT_IDE;
  return INTERIOR_LIGHTS_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_DOOR_RPT_pacmod12(DOOR_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->DRIVER_DOOR_OPEN = (_d[0] & (0x01U));
  _m->PASS_DOOR_OPEN = ((_d[0] >> 1) & (0x01U));
  _m->REAR_DRIVER_DOOR_OPEN = ((_d[0] >> 2) & (0x01U));
  _m->REAR_PASS_DOOR_OPEN = ((_d[0] >> 3) & (0x01U));
  _m->HOOD_OPEN = ((_d[0] >> 4) & (0x01U));
  _m->TRUNK_OPEN = ((_d[0] >> 5) & (0x01U));
  _m->FUEL_DOOR_OPEN = ((_d[0] >> 6) & (0x01U));
  _m->DRIVER_DOOR_OPEN_AVAIL = (_d[1] & (0x01U));
  _m->PASS_DOOR_OPEN_AVAIL = ((_d[1] >> 1) & (0x01U));
  _m->REAR_DRIVER_DOOR_OPEN_AVAIL = ((_d[1] >> 2) & (0x01U));
  _m->REAR_PASS_DOOR_OPEN_AVAIL = ((_d[1] >> 3) & (0x01U));
  _m->HOOD_OPEN_AVAIL = ((_d[1] >> 4) & (0x01U));
  _m->TRUNK_OPEN_AVAIL = ((_d[1] >> 5) & (0x01U));
  _m->FUEL_DOOR_OPEN_AVAIL = ((_d[1] >> 6) & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < DOOR_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_DOOR_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return DOOR_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_DOOR_RPT_pacmod12(DOOR_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < DOOR_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->DRIVER_DOOR_OPEN & (0x01U)) | ((_m->PASS_DOOR_OPEN & (0x01U)) << 1) | ((_m->REAR_DRIVER_DOOR_OPEN & (0x01U)) << 2) | ((_m->REAR_PASS_DOOR_OPEN & (0x01U)) << 3) | ((_m->HOOD_OPEN & (0x01U)) << 4) | ((_m->TRUNK_OPEN & (0x01U)) << 5) | ((_m->FUEL_DOOR_OPEN & (0x01U)) << 6);
  cframe->Data[1] |= (_m->DRIVER_DOOR_OPEN_AVAIL & (0x01U)) | ((_m->PASS_DOOR_OPEN_AVAIL & (0x01U)) << 1) | ((_m->REAR_DRIVER_DOOR_OPEN_AVAIL & (0x01U)) << 2) | ((_m->REAR_PASS_DOOR_OPEN_AVAIL & (0x01U)) << 3) | ((_m->HOOD_OPEN_AVAIL & (0x01U)) << 4) | ((_m->TRUNK_OPEN_AVAIL & (0x01U)) << 5) | ((_m->FUEL_DOOR_OPEN_AVAIL & (0x01U)) << 6);

  cframe->MsgId = DOOR_RPT_CANID;
  cframe->DLC = DOOR_RPT_DLC;
  cframe->IDE = DOOR_RPT_IDE;
  return DOOR_RPT_CANID;
}

#else

uint32_t Pack_DOOR_RPT_pacmod12(DOOR_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < DOOR_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->DRIVER_DOOR_OPEN & (0x01U)) | ((_m->PASS_DOOR_OPEN & (0x01U)) << 1) | ((_m->REAR_DRIVER_DOOR_OPEN & (0x01U)) << 2) | ((_m->REAR_PASS_DOOR_OPEN & (0x01U)) << 3) | ((_m->HOOD_OPEN & (0x01U)) << 4) | ((_m->TRUNK_OPEN & (0x01U)) << 5) | ((_m->FUEL_DOOR_OPEN & (0x01U)) << 6);
  _d[1] |= (_m->DRIVER_DOOR_OPEN_AVAIL & (0x01U)) | ((_m->PASS_DOOR_OPEN_AVAIL & (0x01U)) << 1) | ((_m->REAR_DRIVER_DOOR_OPEN_AVAIL & (0x01U)) << 2) | ((_m->REAR_PASS_DOOR_OPEN_AVAIL & (0x01U)) << 3) | ((_m->HOOD_OPEN_AVAIL & (0x01U)) << 4) | ((_m->TRUNK_OPEN_AVAIL & (0x01U)) << 5) | ((_m->FUEL_DOOR_OPEN_AVAIL & (0x01U)) << 6);

  *_len = DOOR_RPT_DLC;
  *_ide = DOOR_RPT_IDE;
  return DOOR_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_REAR_LIGHTS_RPT_pacmod12(REAR_LIGHTS_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->BRAKE_LIGHTS_ON = (_d[0] & (0x01U));
  _m->REVERSE_LIGHTS_ON = ((_d[0] >> 1) & (0x01U));
  _m->BRAKE_LIGHTS_ON_AVAIL = (_d[1] & (0x01U));
  _m->REVERSE_LIGHTS_ON_AVAIL = ((_d[1] >> 1) & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < REAR_LIGHTS_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_REAR_LIGHTS_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return REAR_LIGHTS_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_REAR_LIGHTS_RPT_pacmod12(REAR_LIGHTS_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < REAR_LIGHTS_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->BRAKE_LIGHTS_ON & (0x01U)) | ((_m->REVERSE_LIGHTS_ON & (0x01U)) << 1);
  cframe->Data[1] |= (_m->BRAKE_LIGHTS_ON_AVAIL & (0x01U)) | ((_m->REVERSE_LIGHTS_ON_AVAIL & (0x01U)) << 1);

  cframe->MsgId = REAR_LIGHTS_RPT_CANID;
  cframe->DLC = REAR_LIGHTS_RPT_DLC;
  cframe->IDE = REAR_LIGHTS_RPT_IDE;
  return REAR_LIGHTS_RPT_CANID;
}

#else

uint32_t Pack_REAR_LIGHTS_RPT_pacmod12(REAR_LIGHTS_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < REAR_LIGHTS_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->BRAKE_LIGHTS_ON & (0x01U)) | ((_m->REVERSE_LIGHTS_ON & (0x01U)) << 1);
  _d[1] |= (_m->BRAKE_LIGHTS_ON_AVAIL & (0x01U)) | ((_m->REVERSE_LIGHTS_ON_AVAIL & (0x01U)) << 1);

  *_len = REAR_LIGHTS_RPT_DLC;
  *_ide = REAR_LIGHTS_RPT_IDE;
  return REAR_LIGHTS_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_LINEAR_ACCEL_RPT_pacmod12(LINEAR_ACCEL_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->LATERAL_NEW_DATA_RX = (_d[0] & (0x01U));
  _m->LONGITUDINAL_NEW_DATA_RX = ((_d[0] >> 1) & (0x01U));
  _m->VERTICAL_NEW_DATA_RX = ((_d[0] >> 2) & (0x01U));
  _m->LATERAL_VALID = ((_d[0] >> 3) & (0x01U));
  _m->LONGITUDINAL_VALID = ((_d[0] >> 4) & (0x01U));
  _m->VERTICAL_VALID = ((_d[0] >> 5) & (0x01U));
  _m->LATERAL_ACCEL_ro = ((_d[1] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->LATERAL_ACCEL_phys = (sigfloat_t)(PACMOD12_LATERAL_ACCEL_ro_fromS(_m->LATERAL_ACCEL_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->LONGITUDINAL_ACCEL_ro = ((_d[3] & (0xFFU)) << 8) | (_d[4] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->LONGITUDINAL_ACCEL_phys = (sigfloat_t)(PACMOD12_LONGITUDINAL_ACCEL_ro_fromS(_m->LONGITUDINAL_ACCEL_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->VERTICAL_ACCEL_ro = ((_d[5] & (0xFFU)) << 8) | (_d[6] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->VERTICAL_ACCEL_phys = (sigfloat_t)(PACMOD12_VERTICAL_ACCEL_ro_fromS(_m->VERTICAL_ACCEL_ro));
#endif // PACMOD12_USE_SIGFLOAT

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < LINEAR_ACCEL_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_LINEAR_ACCEL_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return LINEAR_ACCEL_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_LINEAR_ACCEL_RPT_pacmod12(LINEAR_ACCEL_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < LINEAR_ACCEL_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->LATERAL_ACCEL_ro = PACMOD12_LATERAL_ACCEL_ro_toS(_m->LATERAL_ACCEL_phys);
  _m->LONGITUDINAL_ACCEL_ro = PACMOD12_LONGITUDINAL_ACCEL_ro_toS(_m->LONGITUDINAL_ACCEL_phys);
  _m->VERTICAL_ACCEL_ro = PACMOD12_VERTICAL_ACCEL_ro_toS(_m->VERTICAL_ACCEL_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[0] |= (_m->LATERAL_NEW_DATA_RX & (0x01U)) | ((_m->LONGITUDINAL_NEW_DATA_RX & (0x01U)) << 1) | ((_m->VERTICAL_NEW_DATA_RX & (0x01U)) << 2) | ((_m->LATERAL_VALID & (0x01U)) << 3) | ((_m->LONGITUDINAL_VALID & (0x01U)) << 4) | ((_m->VERTICAL_VALID & (0x01U)) << 5);
  cframe->Data[1] |= ((_m->LATERAL_ACCEL_ro >> 8) & (0xFFU));
  cframe->Data[2] |= (_m->LATERAL_ACCEL_ro & (0xFFU));
  cframe->Data[3] |= ((_m->LONGITUDINAL_ACCEL_ro >> 8) & (0xFFU));
  cframe->Data[4] |= (_m->LONGITUDINAL_ACCEL_ro & (0xFFU));
  cframe->Data[5] |= ((_m->VERTICAL_ACCEL_ro >> 8) & (0xFFU));
  cframe->Data[6] |= (_m->VERTICAL_ACCEL_ro & (0xFFU));

  cframe->MsgId = LINEAR_ACCEL_RPT_CANID;
  cframe->DLC = LINEAR_ACCEL_RPT_DLC;
  cframe->IDE = LINEAR_ACCEL_RPT_IDE;
  return LINEAR_ACCEL_RPT_CANID;
}

#else

uint32_t Pack_LINEAR_ACCEL_RPT_pacmod12(LINEAR_ACCEL_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < LINEAR_ACCEL_RPT_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->LATERAL_ACCEL_ro = PACMOD12_LATERAL_ACCEL_ro_toS(_m->LATERAL_ACCEL_phys);
  _m->LONGITUDINAL_ACCEL_ro = PACMOD12_LONGITUDINAL_ACCEL_ro_toS(_m->LONGITUDINAL_ACCEL_phys);
  _m->VERTICAL_ACCEL_ro = PACMOD12_VERTICAL_ACCEL_ro_toS(_m->VERTICAL_ACCEL_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[0] |= (_m->LATERAL_NEW_DATA_RX & (0x01U)) | ((_m->LONGITUDINAL_NEW_DATA_RX & (0x01U)) << 1) | ((_m->VERTICAL_NEW_DATA_RX & (0x01U)) << 2) | ((_m->LATERAL_VALID & (0x01U)) << 3) | ((_m->LONGITUDINAL_VALID & (0x01U)) << 4) | ((_m->VERTICAL_VALID & (0x01U)) << 5);
  _d[1] |= ((_m->LATERAL_ACCEL_ro >> 8) & (0xFFU));
  _d[2] |= (_m->LATERAL_ACCEL_ro & (0xFFU));
  _d[3] |= ((_m->LONGITUDINAL_ACCEL_ro >> 8) & (0xFFU));
  _d[4] |= (_m->LONGITUDINAL_ACCEL_ro & (0xFFU));
  _d[5] |= ((_m->VERTICAL_ACCEL_ro >> 8) & (0xFFU));
  _d[6] |= (_m->VERTICAL_ACCEL_ro & (0xFFU));

  *_len = LINEAR_ACCEL_RPT_DLC;
  *_ide = LINEAR_ACCEL_RPT_IDE;
  return LINEAR_ACCEL_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_ANG_VEL_RPT_pacmod12(ANG_VEL_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->PITCH_NEW_DATA_RX = (_d[0] & (0x01U));
  _m->ROLL_NEW_DATA_RX = ((_d[0] >> 1) & (0x01U));
  _m->YAW_NEW_DATA_RX = ((_d[0] >> 2) & (0x01U));
  _m->PITCH_VALID = ((_d[0] >> 3) & (0x01U));
  _m->ROLL_VALID = ((_d[0] >> 4) & (0x01U));
  _m->YAW_VALID = ((_d[0] >> 5) & (0x01U));
  _m->PITCH_VEL_ro = ((_d[1] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->PITCH_VEL_phys = (sigfloat_t)(PACMOD12_PITCH_VEL_ro_fromS(_m->PITCH_VEL_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->ROLL_VEL_ro = ((_d[3] & (0xFFU)) << 8) | (_d[4] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->ROLL_VEL_phys = (sigfloat_t)(PACMOD12_ROLL_VEL_ro_fromS(_m->ROLL_VEL_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->YAW_VEL_ro = ((_d[5] & (0xFFU)) << 8) | (_d[6] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->YAW_VEL_phys = (sigfloat_t)(PACMOD12_YAW_VEL_ro_fromS(_m->YAW_VEL_ro));
#endif // PACMOD12_USE_SIGFLOAT

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < ANG_VEL_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_ANG_VEL_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return ANG_VEL_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_ANG_VEL_RPT_pacmod12(ANG_VEL_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < ANG_VEL_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->PITCH_VEL_ro = PACMOD12_PITCH_VEL_ro_toS(_m->PITCH_VEL_phys);
  _m->ROLL_VEL_ro = PACMOD12_ROLL_VEL_ro_toS(_m->ROLL_VEL_phys);
  _m->YAW_VEL_ro = PACMOD12_YAW_VEL_ro_toS(_m->YAW_VEL_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[0] |= (_m->PITCH_NEW_DATA_RX & (0x01U)) | ((_m->ROLL_NEW_DATA_RX & (0x01U)) << 1) | ((_m->YAW_NEW_DATA_RX & (0x01U)) << 2) | ((_m->PITCH_VALID & (0x01U)) << 3) | ((_m->ROLL_VALID & (0x01U)) << 4) | ((_m->YAW_VALID & (0x01U)) << 5);
  cframe->Data[1] |= ((_m->PITCH_VEL_ro >> 8) & (0xFFU));
  cframe->Data[2] |= (_m->PITCH_VEL_ro & (0xFFU));
  cframe->Data[3] |= ((_m->ROLL_VEL_ro >> 8) & (0xFFU));
  cframe->Data[4] |= (_m->ROLL_VEL_ro & (0xFFU));
  cframe->Data[5] |= ((_m->YAW_VEL_ro >> 8) & (0xFFU));
  cframe->Data[6] |= (_m->YAW_VEL_ro & (0xFFU));

  cframe->MsgId = ANG_VEL_RPT_CANID;
  cframe->DLC = ANG_VEL_RPT_DLC;
  cframe->IDE = ANG_VEL_RPT_IDE;
  return ANG_VEL_RPT_CANID;
}

#else

uint32_t Pack_ANG_VEL_RPT_pacmod12(ANG_VEL_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < ANG_VEL_RPT_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->PITCH_VEL_ro = PACMOD12_PITCH_VEL_ro_toS(_m->PITCH_VEL_phys);
  _m->ROLL_VEL_ro = PACMOD12_ROLL_VEL_ro_toS(_m->ROLL_VEL_phys);
  _m->YAW_VEL_ro = PACMOD12_YAW_VEL_ro_toS(_m->YAW_VEL_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[0] |= (_m->PITCH_NEW_DATA_RX & (0x01U)) | ((_m->ROLL_NEW_DATA_RX & (0x01U)) << 1) | ((_m->YAW_NEW_DATA_RX & (0x01U)) << 2) | ((_m->PITCH_VALID & (0x01U)) << 3) | ((_m->ROLL_VALID & (0x01U)) << 4) | ((_m->YAW_VALID & (0x01U)) << 5);
  _d[1] |= ((_m->PITCH_VEL_ro >> 8) & (0xFFU));
  _d[2] |= (_m->PITCH_VEL_ro & (0xFFU));
  _d[3] |= ((_m->ROLL_VEL_ro >> 8) & (0xFFU));
  _d[4] |= (_m->ROLL_VEL_ro & (0xFFU));
  _d[5] |= ((_m->YAW_VEL_ro >> 8) & (0xFFU));
  _d[6] |= (_m->YAW_VEL_ro & (0xFFU));

  *_len = ANG_VEL_RPT_DLC;
  *_ide = ANG_VEL_RPT_IDE;
  return ANG_VEL_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_NOTIFICATION_CMD_pacmod12(NOTIFICATION_CMD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->BUZZER_MUTE = (_d[0] & (0x01U));
  _m->UNDERDASH_LIGHTS_WHITE = ((_d[0] >> 1) & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < NOTIFICATION_CMD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_NOTIFICATION_CMD_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return NOTIFICATION_CMD_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_NOTIFICATION_CMD_pacmod12(NOTIFICATION_CMD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < NOTIFICATION_CMD_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->BUZZER_MUTE & (0x01U)) | ((_m->UNDERDASH_LIGHTS_WHITE & (0x01U)) << 1);

  cframe->MsgId = NOTIFICATION_CMD_CANID;
  cframe->DLC = NOTIFICATION_CMD_DLC;
  cframe->IDE = NOTIFICATION_CMD_IDE;
  return NOTIFICATION_CMD_CANID;
}

#else

uint32_t Pack_NOTIFICATION_CMD_pacmod12(NOTIFICATION_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < NOTIFICATION_CMD_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->BUZZER_MUTE & (0x01U)) | ((_m->UNDERDASH_LIGHTS_WHITE & (0x01U)) << 1);

  *_len = NOTIFICATION_CMD_DLC;
  *_ide = NOTIFICATION_CMD_IDE;
  return NOTIFICATION_CMD_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_ESTOP_RPT_pacmod12(ESTOP_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ESTOP = (_d[0] & (0x01U));
  _m->ESTOP_FAULT = ((_d[0] >> 1) & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < ESTOP_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_ESTOP_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return ESTOP_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_ESTOP_RPT_pacmod12(ESTOP_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < ESTOP_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ESTOP & (0x01U)) | ((_m->ESTOP_FAULT & (0x01U)) << 1);

  cframe->MsgId = ESTOP_RPT_CANID;
  cframe->DLC = ESTOP_RPT_DLC;
  cframe->IDE = ESTOP_RPT_IDE;
  return ESTOP_RPT_CANID;
}

#else

uint32_t Pack_ESTOP_RPT_pacmod12(ESTOP_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < ESTOP_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ESTOP & (0x01U)) | ((_m->ESTOP_FAULT & (0x01U)) << 1);

  *_len = ESTOP_RPT_DLC;
  *_ide = ESTOP_RPT_IDE;
  return ESTOP_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_USER_NOTIFICATION_CMD_pacmod12(USER_NOTIFICATION_CMD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->BUZZER_MUTE = (_d[0] & (0x01U));
  _m->LIGHT_COMMAND = ((_d[0] >> 1) & (0x0FU));
  _m->BUZZER_ON = ((_d[0] >> 5) & (0x01U));
  _m->BUZZER_MUTE_INDICATOR = ((_d[0] >> 6) & (0x03U));
  _m->LED_BRIGHTNESS = (_d[1] & (0x0FU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < USER_NOTIFICATION_CMD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_USER_NOTIFICATION_CMD_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return USER_NOTIFICATION_CMD_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_USER_NOTIFICATION_CMD_pacmod12(USER_NOTIFICATION_CMD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < USER_NOTIFICATION_CMD_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->BUZZER_MUTE & (0x01U)) | ((_m->LIGHT_COMMAND & (0x0FU)) << 1) | ((_m->BUZZER_ON & (0x01U)) << 5) | ((_m->BUZZER_MUTE_INDICATOR & (0x03U)) << 6);
  cframe->Data[1] |= (_m->LED_BRIGHTNESS & (0x0FU));

  cframe->MsgId = USER_NOTIFICATION_CMD_CANID;
  cframe->DLC = USER_NOTIFICATION_CMD_DLC;
  cframe->IDE = USER_NOTIFICATION_CMD_IDE;
  return USER_NOTIFICATION_CMD_CANID;
}

#else

uint32_t Pack_USER_NOTIFICATION_CMD_pacmod12(USER_NOTIFICATION_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < USER_NOTIFICATION_CMD_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->BUZZER_MUTE & (0x01U)) | ((_m->LIGHT_COMMAND & (0x0FU)) << 1) | ((_m->BUZZER_ON & (0x01U)) << 5) | ((_m->BUZZER_MUTE_INDICATOR & (0x03U)) << 6);
  _d[1] |= (_m->LED_BRIGHTNESS & (0x0FU));

  *_len = USER_NOTIFICATION_CMD_DLC;
  *_ide = USER_NOTIFICATION_CMD_IDE;
  return USER_NOTIFICATION_CMD_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_TIRE_PRESSURE_RPT_pacmod12(TIRE_PRESSURE_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->FRONT_LEFT_TIRE_PRESSURE_ro = (_d[0] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->FRONT_LEFT_TIRE_PRESSURE_phys = PACMOD12_FRONT_LEFT_TIRE_PRESSURE_ro_fromS(_m->FRONT_LEFT_TIRE_PRESSURE_ro);
#endif // PACMOD12_USE_SIGFLOAT

  _m->FRONT_RIGHT_TIRE_PRESSURE_ro = (_d[1] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->FRONT_RIGHT_TIRE_PRESSURE_phys = PACMOD12_FRONT_RIGHT_TIRE_PRESSURE_ro_fromS(_m->FRONT_RIGHT_TIRE_PRESSURE_ro);
#endif // PACMOD12_USE_SIGFLOAT

  _m->REAR_LEFT_TIRE_PRESSURE_ro = (_d[2] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->REAR_LEFT_TIRE_PRESSURE_phys = PACMOD12_REAR_LEFT_TIRE_PRESSURE_ro_fromS(_m->REAR_LEFT_TIRE_PRESSURE_ro);
#endif // PACMOD12_USE_SIGFLOAT

  _m->REAR_RIGHT_TIRE_PRESSURE_ro = (_d[3] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->REAR_RIGHT_TIRE_PRESSURE_phys = PACMOD12_REAR_RIGHT_TIRE_PRESSURE_ro_fromS(_m->REAR_RIGHT_TIRE_PRESSURE_ro);
#endif // PACMOD12_USE_SIGFLOAT

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < TIRE_PRESSURE_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_TIRE_PRESSURE_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return TIRE_PRESSURE_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_TIRE_PRESSURE_RPT_pacmod12(TIRE_PRESSURE_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < TIRE_PRESSURE_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->FRONT_LEFT_TIRE_PRESSURE_ro = PACMOD12_FRONT_LEFT_TIRE_PRESSURE_ro_toS(_m->FRONT_LEFT_TIRE_PRESSURE_phys);
  _m->FRONT_RIGHT_TIRE_PRESSURE_ro = PACMOD12_FRONT_RIGHT_TIRE_PRESSURE_ro_toS(_m->FRONT_RIGHT_TIRE_PRESSURE_phys);
  _m->REAR_LEFT_TIRE_PRESSURE_ro = PACMOD12_REAR_LEFT_TIRE_PRESSURE_ro_toS(_m->REAR_LEFT_TIRE_PRESSURE_phys);
  _m->REAR_RIGHT_TIRE_PRESSURE_ro = PACMOD12_REAR_RIGHT_TIRE_PRESSURE_ro_toS(_m->REAR_RIGHT_TIRE_PRESSURE_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[0] |= (_m->FRONT_LEFT_TIRE_PRESSURE_ro & (0xFFU));
  cframe->Data[1] |= (_m->FRONT_RIGHT_TIRE_PRESSURE_ro & (0xFFU));
  cframe->Data[2] |= (_m->REAR_LEFT_TIRE_PRESSURE_ro & (0xFFU));
  cframe->Data[3] |= (_m->REAR_RIGHT_TIRE_PRESSURE_ro & (0xFFU));

  cframe->MsgId = TIRE_PRESSURE_RPT_CANID;
  cframe->DLC = TIRE_PRESSURE_RPT_DLC;
  cframe->IDE = TIRE_PRESSURE_RPT_IDE;
  return TIRE_PRESSURE_RPT_CANID;
}

#else

uint32_t Pack_TIRE_PRESSURE_RPT_pacmod12(TIRE_PRESSURE_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < TIRE_PRESSURE_RPT_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->FRONT_LEFT_TIRE_PRESSURE_ro = PACMOD12_FRONT_LEFT_TIRE_PRESSURE_ro_toS(_m->FRONT_LEFT_TIRE_PRESSURE_phys);
  _m->FRONT_RIGHT_TIRE_PRESSURE_ro = PACMOD12_FRONT_RIGHT_TIRE_PRESSURE_ro_toS(_m->FRONT_RIGHT_TIRE_PRESSURE_phys);
  _m->REAR_LEFT_TIRE_PRESSURE_ro = PACMOD12_REAR_LEFT_TIRE_PRESSURE_ro_toS(_m->REAR_LEFT_TIRE_PRESSURE_phys);
  _m->REAR_RIGHT_TIRE_PRESSURE_ro = PACMOD12_REAR_RIGHT_TIRE_PRESSURE_ro_toS(_m->REAR_RIGHT_TIRE_PRESSURE_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[0] |= (_m->FRONT_LEFT_TIRE_PRESSURE_ro & (0xFFU));
  _d[1] |= (_m->FRONT_RIGHT_TIRE_PRESSURE_ro & (0xFFU));
  _d[2] |= (_m->REAR_LEFT_TIRE_PRESSURE_ro & (0xFFU));
  _d[3] |= (_m->REAR_RIGHT_TIRE_PRESSURE_ro & (0xFFU));

  *_len = TIRE_PRESSURE_RPT_DLC;
  *_ide = TIRE_PRESSURE_RPT_IDE;
  return TIRE_PRESSURE_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_DRIVE_TRAIN_FEATURE_RPT_pacmod12(DRIVE_TRAIN_FEATURE_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ANTILOCK_BRAKE_ACTIVE = (_d[0] & (0x01U));
  _m->TRACTION_CONTROL_ACTIVE = ((_d[0] >> 1) & (0x01U));
  _m->FOUR_WHEEL_DRIVE_ACTIVE = ((_d[0] >> 2) & (0x01U));
  _m->ANTILOCK_BRAKE_ACTIVE_AVAIL = ((_d[0] >> 4) & (0x01U));
  _m->TRACTION_CONTROL_ACTIVE_AVAIL = ((_d[0] >> 5) & (0x01U));
  _m->FOUR_WHEEL_DRIVE_ACTIVE_AVAIL = ((_d[0] >> 6) & (0x01U));
  _m->DRIVE_MODE = (_d[1] & (0x0FU));
  _m->DRIVE_MODE_AVAIL = ((_d[1] >> 4) & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < DRIVE_TRAIN_FEATURE_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_DRIVE_TRAIN_FEATURE_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return DRIVE_TRAIN_FEATURE_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_DRIVE_TRAIN_FEATURE_RPT_pacmod12(DRIVE_TRAIN_FEATURE_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < DRIVE_TRAIN_FEATURE_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ANTILOCK_BRAKE_ACTIVE & (0x01U)) | ((_m->TRACTION_CONTROL_ACTIVE & (0x01U)) << 1) | ((_m->FOUR_WHEEL_DRIVE_ACTIVE & (0x01U)) << 2) | ((_m->ANTILOCK_BRAKE_ACTIVE_AVAIL & (0x01U)) << 4) | ((_m->TRACTION_CONTROL_ACTIVE_AVAIL & (0x01U)) << 5) | ((_m->FOUR_WHEEL_DRIVE_ACTIVE_AVAIL & (0x01U)) << 6);
  cframe->Data[1] |= (_m->DRIVE_MODE & (0x0FU)) | ((_m->DRIVE_MODE_AVAIL & (0x01U)) << 4);

  cframe->MsgId = DRIVE_TRAIN_FEATURE_RPT_CANID;
  cframe->DLC = DRIVE_TRAIN_FEATURE_RPT_DLC;
  cframe->IDE = DRIVE_TRAIN_FEATURE_RPT_IDE;
  return DRIVE_TRAIN_FEATURE_RPT_CANID;
}

#else

uint32_t Pack_DRIVE_TRAIN_FEATURE_RPT_pacmod12(DRIVE_TRAIN_FEATURE_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < DRIVE_TRAIN_FEATURE_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ANTILOCK_BRAKE_ACTIVE & (0x01U)) | ((_m->TRACTION_CONTROL_ACTIVE & (0x01U)) << 1) | ((_m->FOUR_WHEEL_DRIVE_ACTIVE & (0x01U)) << 2) | ((_m->ANTILOCK_BRAKE_ACTIVE_AVAIL & (0x01U)) << 4) | ((_m->TRACTION_CONTROL_ACTIVE_AVAIL & (0x01U)) << 5) | ((_m->FOUR_WHEEL_DRIVE_ACTIVE_AVAIL & (0x01U)) << 6);
  _d[1] |= (_m->DRIVE_MODE & (0x0FU)) | ((_m->DRIVE_MODE_AVAIL & (0x01U)) << 4);

  *_len = DRIVE_TRAIN_FEATURE_RPT_DLC;
  *_ide = DRIVE_TRAIN_FEATURE_RPT_IDE;
  return DRIVE_TRAIN_FEATURE_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_SAFETY_FUNC_CRITICAL_STOP_RPT_pacmod12(SAFETY_FUNC_CRITICAL_STOP_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->AUTOMSMAN_OPCTRL_FAULT = (_d[0] & (0x01U));
  _m->REMOTE_STOP_FAULT = ((_d[0] >> 1) & (0x01U));
  _m->SAFETY_BRAKE_OPCTRL_OFF = ((_d[0] >> 2) & (0x01U));
  _m->SAFETY_BRAKE_CMD_TIMEOUT = ((_d[0] >> 3) & (0x01U));
  _m->SAFETY_FUNC_CMD_TIMEOUT = ((_d[0] >> 4) & (0x01U));
  _m->SAFETY_FUNC_CRITICAL_STOP_1_CMD = ((_d[0] >> 5) & (0x01U));
  _m->SAFETY_FUNC_CRITICAL_STOP_2_CMD = ((_d[0] >> 6) & (0x01U));
  _m->SAFETY_FUNC_NONE_CMD = ((_d[0] >> 7) & (0x01U));
  _m->PACMOD_SYSTEM_TIMEOUT = (_d[1] & (0x01U));
  _m->PACMOD_SYSTEM_FAULT = ((_d[1] >> 1) & (0x01U));
  _m->PACMOD_SYSTEM_NOT_ACTIVE = ((_d[1] >> 2) & (0x01U));
  _m->VEHICLE_REPORT_TIMEOUT = ((_d[1] >> 3) & (0x01U));
  _m->VEHICLE_REPORT_FAULT = ((_d[1] >> 4) & (0x01U));
  _m->LOW_ENGINE_RPM = ((_d[1] >> 5) & (0x01U));
  _m->PRI_SAFETY_BRAKE_SIGNAL_1_FAULT = ((_d[1] >> 6) & (0x01U));
  _m->PRI_SAFETY_BRAKE_SIGNAL_2_FAULT = ((_d[1] >> 7) & (0x01U));
  _m->SEC_SAFETY_BRAKE_SIGNAL_1_FAULT = (_d[2] & (0x01U));
  _m->SEC_SAFETY_BRAKE_SIGNAL_2_FAULT = ((_d[2] >> 1) & (0x01U));
  _m->PRIMARY_PROCESSOR_FAULT = ((_d[2] >> 2) & (0x01U));
  _m->SECONDARY_PROCESSOR_FAULT = ((_d[2] >> 3) & (0x01U));
  _m->REMOTE_STOP_CMD = ((_d[2] >> 4) & (0x01U));
  _m->PRI_SAFETY_BRAKE_PRESSURE_FAULT = ((_d[2] >> 5) & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < SAFETY_FUNC_CRITICAL_STOP_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_SAFETY_FUNC_CRITICAL_STOP_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return SAFETY_FUNC_CRITICAL_STOP_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_SAFETY_FUNC_CRITICAL_STOP_RPT_pacmod12(SAFETY_FUNC_CRITICAL_STOP_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < SAFETY_FUNC_CRITICAL_STOP_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->AUTOMSMAN_OPCTRL_FAULT & (0x01U)) | ((_m->REMOTE_STOP_FAULT & (0x01U)) << 1) | ((_m->SAFETY_BRAKE_OPCTRL_OFF & (0x01U)) << 2) | ((_m->SAFETY_BRAKE_CMD_TIMEOUT & (0x01U)) << 3) | ((_m->SAFETY_FUNC_CMD_TIMEOUT & (0x01U)) << 4) | ((_m->SAFETY_FUNC_CRITICAL_STOP_1_CMD & (0x01U)) << 5) | ((_m->SAFETY_FUNC_CRITICAL_STOP_2_CMD & (0x01U)) << 6) | ((_m->SAFETY_FUNC_NONE_CMD & (0x01U)) << 7);
  cframe->Data[1] |= (_m->PACMOD_SYSTEM_TIMEOUT & (0x01U)) | ((_m->PACMOD_SYSTEM_FAULT & (0x01U)) << 1) | ((_m->PACMOD_SYSTEM_NOT_ACTIVE & (0x01U)) << 2) | ((_m->VEHICLE_REPORT_TIMEOUT & (0x01U)) << 3) | ((_m->VEHICLE_REPORT_FAULT & (0x01U)) << 4) | ((_m->LOW_ENGINE_RPM & (0x01U)) << 5) | ((_m->PRI_SAFETY_BRAKE_SIGNAL_1_FAULT & (0x01U)) << 6) | ((_m->PRI_SAFETY_BRAKE_SIGNAL_2_FAULT & (0x01U)) << 7);
  cframe->Data[2] |= (_m->SEC_SAFETY_BRAKE_SIGNAL_1_FAULT & (0x01U)) | ((_m->SEC_SAFETY_BRAKE_SIGNAL_2_FAULT & (0x01U)) << 1) | ((_m->PRIMARY_PROCESSOR_FAULT & (0x01U)) << 2) | ((_m->SECONDARY_PROCESSOR_FAULT & (0x01U)) << 3) | ((_m->REMOTE_STOP_CMD & (0x01U)) << 4) | ((_m->PRI_SAFETY_BRAKE_PRESSURE_FAULT & (0x01U)) << 5);

  cframe->MsgId = SAFETY_FUNC_CRITICAL_STOP_RPT_CANID;
  cframe->DLC = SAFETY_FUNC_CRITICAL_STOP_RPT_DLC;
  cframe->IDE = SAFETY_FUNC_CRITICAL_STOP_RPT_IDE;
  return SAFETY_FUNC_CRITICAL_STOP_RPT_CANID;
}

#else

uint32_t Pack_SAFETY_FUNC_CRITICAL_STOP_RPT_pacmod12(SAFETY_FUNC_CRITICAL_STOP_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < SAFETY_FUNC_CRITICAL_STOP_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->AUTOMSMAN_OPCTRL_FAULT & (0x01U)) | ((_m->REMOTE_STOP_FAULT & (0x01U)) << 1) | ((_m->SAFETY_BRAKE_OPCTRL_OFF & (0x01U)) << 2) | ((_m->SAFETY_BRAKE_CMD_TIMEOUT & (0x01U)) << 3) | ((_m->SAFETY_FUNC_CMD_TIMEOUT & (0x01U)) << 4) | ((_m->SAFETY_FUNC_CRITICAL_STOP_1_CMD & (0x01U)) << 5) | ((_m->SAFETY_FUNC_CRITICAL_STOP_2_CMD & (0x01U)) << 6) | ((_m->SAFETY_FUNC_NONE_CMD & (0x01U)) << 7);
  _d[1] |= (_m->PACMOD_SYSTEM_TIMEOUT & (0x01U)) | ((_m->PACMOD_SYSTEM_FAULT & (0x01U)) << 1) | ((_m->PACMOD_SYSTEM_NOT_ACTIVE & (0x01U)) << 2) | ((_m->VEHICLE_REPORT_TIMEOUT & (0x01U)) << 3) | ((_m->VEHICLE_REPORT_FAULT & (0x01U)) << 4) | ((_m->LOW_ENGINE_RPM & (0x01U)) << 5) | ((_m->PRI_SAFETY_BRAKE_SIGNAL_1_FAULT & (0x01U)) << 6) | ((_m->PRI_SAFETY_BRAKE_SIGNAL_2_FAULT & (0x01U)) << 7);
  _d[2] |= (_m->SEC_SAFETY_BRAKE_SIGNAL_1_FAULT & (0x01U)) | ((_m->SEC_SAFETY_BRAKE_SIGNAL_2_FAULT & (0x01U)) << 1) | ((_m->PRIMARY_PROCESSOR_FAULT & (0x01U)) << 2) | ((_m->SECONDARY_PROCESSOR_FAULT & (0x01U)) << 3) | ((_m->REMOTE_STOP_CMD & (0x01U)) << 4) | ((_m->PRI_SAFETY_BRAKE_PRESSURE_FAULT & (0x01U)) << 5);

  *_len = SAFETY_FUNC_CRITICAL_STOP_RPT_DLC;
  *_ide = SAFETY_FUNC_CRITICAL_STOP_RPT_IDE;
  return SAFETY_FUNC_CRITICAL_STOP_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_WATCHDOG_RPT_2_pacmod12(WATCHDOG_RPT_2_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ACCEL_RPT_TIMEOUT = (_d[0] & (0x01U));
  _m->BRAKE_RPT_TIMEOUT = ((_d[0] >> 1) & (0x01U));
  _m->BRAKE_DECEL_RPT_TIMEOUT = ((_d[0] >> 2) & (0x01U));
  _m->CABIN_CLIMATE_RPT_TIMEOUT = ((_d[0] >> 3) & (0x01U));
  _m->CABIN_FAN_SPEED_RPT_TIMEOUT = ((_d[0] >> 4) & (0x01U));
  _m->CABIN_TEMP_RPT_TIMEOUT = ((_d[0] >> 5) & (0x01U));
  _m->CRUISE_CONTROL_RPT_TIMEOUT = ((_d[0] >> 6) & (0x01U));
  _m->DASH_LEFT_RPT_TIMEOUT = ((_d[0] >> 7) & (0x01U));
  _m->DASH_RIGHT_RPT_TIMEOUT = (_d[1] & (0x01U));
  _m->ENGINE_BRAKE_RPT_TIMEOUT = ((_d[1] >> 1) & (0x01U));
  _m->HAZARD_LIGHTS_RPT_TIMEOUT = ((_d[1] >> 2) & (0x01U));
  _m->HEADLIGHT_RPT_TIMEOUT = ((_d[1] >> 3) & (0x01U));
  _m->HORN_RPT_TIMEOUT = ((_d[1] >> 4) & (0x01U));
  _m->MARKER_LAMP_RPT_TIMEOUT = ((_d[1] >> 5) & (0x01U));
  _m->MEDIA_CONTROLS_RPT_TIMEOUT = ((_d[1] >> 6) & (0x01U));
  _m->PARKING_BRAKE_RPT_TIMEOUT = ((_d[1] >> 7) & (0x01U));
  _m->REAR_PASS_DOOR_RPT_TIMEOUT = (_d[2] & (0x01U));
  _m->SHIFT_RPT_TIMEOUT = ((_d[2] >> 1) & (0x01U));
  _m->SPRAYER_RPT_TIMEOUT = ((_d[2] >> 2) & (0x01U));
  _m->STEERING_RPT_TIMEOUT = ((_d[2] >> 3) & (0x01U));
  _m->TURN_RPT_TIMEOUT = ((_d[2] >> 4) & (0x01U));
  _m->WIPER_RPT_TIMEOUT = ((_d[2] >> 5) & (0x01U));
  _m->PACMOD1_SANITY_FAULT = ((_d[2] >> 6) & (0x01U));
  _m->PACMOD2_SANITY_FAULT = ((_d[2] >> 7) & (0x01U));
  _m->PACMOD3_SANITY_FAULT = (_d[3] & (0x01U));
  _m->PACMINI1_SANITY_FAULT = ((_d[3] >> 1) & (0x01U));
  _m->PACMINI2_SANITY_FAULT = ((_d[3] >> 2) & (0x01U));
  _m->PACMINI3_SANITY_FAULT = ((_d[3] >> 3) & (0x01U));
  _m->PACMOD1_COMPONENT_RPT_TIMEOUT = ((_d[3] >> 4) & (0x01U));
  _m->PACMOD2_COMPONENT_RPT_TIMEOUT = ((_d[3] >> 5) & (0x01U));
  _m->PACMOD3_COMPONENT_RPT_TIMEOUT = ((_d[3] >> 6) & (0x01U));
  _m->PACMINI1_COMPONENT_RPT_TIMEOUT = ((_d[3] >> 7) & (0x01U));
  _m->PACMINI2_COMPONENT_RPT_TIMEOUT = (_d[4] & (0x01U));
  _m->PACMINI3_COMPONENT_RPT_TIMEOUT = ((_d[4] >> 1) & (0x01U));
  _m->PACMOD1_SYSTEM_PRESENT_FAULT = ((_d[4] >> 2) & (0x01U));
  _m->PACMOD2_SYSTEM_PRESENT_FAULT = ((_d[4] >> 3) & (0x01U));
  _m->PACMOD3_SYSTEM_PRESENT_FAULT = ((_d[4] >> 4) & (0x01U));
  _m->PACMINI1_SYSTEM_PRESENT_FAULT = ((_d[4] >> 5) & (0x01U));
  _m->PACMINI2_SYSTEM_PRESENT_FAULT = ((_d[4] >> 6) & (0x01U));
  _m->PACMINI3_SYSTEM_PRESENT_FAULT = ((_d[4] >> 7) & (0x01U));
  _m->DRIVE_MODE_INVALID = (_d[5] & (0x01U));
  _m->GLOBAL_CMD_SANITY_FAULT = ((_d[5] >> 1) & (0x01U));
  _m->GLOBAL_CMD_TIMEOUT = ((_d[5] >> 2) & (0x01U));
  _m->EXHAUST_BRAKE_RPT_TIMEOUT = ((_d[5] >> 3) & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < WATCHDOG_RPT_2_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_WATCHDOG_RPT_2_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return WATCHDOG_RPT_2_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_WATCHDOG_RPT_2_pacmod12(WATCHDOG_RPT_2_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < WATCHDOG_RPT_2_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->ACCEL_RPT_TIMEOUT & (0x01U)) | ((_m->BRAKE_RPT_TIMEOUT & (0x01U)) << 1) | ((_m->BRAKE_DECEL_RPT_TIMEOUT & (0x01U)) << 2) | ((_m->CABIN_CLIMATE_RPT_TIMEOUT & (0x01U)) << 3) | ((_m->CABIN_FAN_SPEED_RPT_TIMEOUT & (0x01U)) << 4) | ((_m->CABIN_TEMP_RPT_TIMEOUT & (0x01U)) << 5) | ((_m->CRUISE_CONTROL_RPT_TIMEOUT & (0x01U)) << 6) | ((_m->DASH_LEFT_RPT_TIMEOUT & (0x01U)) << 7);
  cframe->Data[1] |= (_m->DASH_RIGHT_RPT_TIMEOUT & (0x01U)) | ((_m->ENGINE_BRAKE_RPT_TIMEOUT & (0x01U)) << 1) | ((_m->HAZARD_LIGHTS_RPT_TIMEOUT & (0x01U)) << 2) | ((_m->HEADLIGHT_RPT_TIMEOUT & (0x01U)) << 3) | ((_m->HORN_RPT_TIMEOUT & (0x01U)) << 4) | ((_m->MARKER_LAMP_RPT_TIMEOUT & (0x01U)) << 5) | ((_m->MEDIA_CONTROLS_RPT_TIMEOUT & (0x01U)) << 6) | ((_m->PARKING_BRAKE_RPT_TIMEOUT & (0x01U)) << 7);
  cframe->Data[2] |= (_m->REAR_PASS_DOOR_RPT_TIMEOUT & (0x01U)) | ((_m->SHIFT_RPT_TIMEOUT & (0x01U)) << 1) | ((_m->SPRAYER_RPT_TIMEOUT & (0x01U)) << 2) | ((_m->STEERING_RPT_TIMEOUT & (0x01U)) << 3) | ((_m->TURN_RPT_TIMEOUT & (0x01U)) << 4) | ((_m->WIPER_RPT_TIMEOUT & (0x01U)) << 5) | ((_m->PACMOD1_SANITY_FAULT & (0x01U)) << 6) | ((_m->PACMOD2_SANITY_FAULT & (0x01U)) << 7);
  cframe->Data[3] |= (_m->PACMOD3_SANITY_FAULT & (0x01U)) | ((_m->PACMINI1_SANITY_FAULT & (0x01U)) << 1) | ((_m->PACMINI2_SANITY_FAULT & (0x01U)) << 2) | ((_m->PACMINI3_SANITY_FAULT & (0x01U)) << 3) | ((_m->PACMOD1_COMPONENT_RPT_TIMEOUT & (0x01U)) << 4) | ((_m->PACMOD2_COMPONENT_RPT_TIMEOUT & (0x01U)) << 5) | ((_m->PACMOD3_COMPONENT_RPT_TIMEOUT & (0x01U)) << 6) | ((_m->PACMINI1_COMPONENT_RPT_TIMEOUT & (0x01U)) << 7);
  cframe->Data[4] |= (_m->PACMINI2_COMPONENT_RPT_TIMEOUT & (0x01U)) | ((_m->PACMINI3_COMPONENT_RPT_TIMEOUT & (0x01U)) << 1) | ((_m->PACMOD1_SYSTEM_PRESENT_FAULT & (0x01U)) << 2) | ((_m->PACMOD2_SYSTEM_PRESENT_FAULT & (0x01U)) << 3) | ((_m->PACMOD3_SYSTEM_PRESENT_FAULT & (0x01U)) << 4) | ((_m->PACMINI1_SYSTEM_PRESENT_FAULT & (0x01U)) << 5) | ((_m->PACMINI2_SYSTEM_PRESENT_FAULT & (0x01U)) << 6) | ((_m->PACMINI3_SYSTEM_PRESENT_FAULT & (0x01U)) << 7);
  cframe->Data[5] |= (_m->DRIVE_MODE_INVALID & (0x01U)) | ((_m->GLOBAL_CMD_SANITY_FAULT & (0x01U)) << 1) | ((_m->GLOBAL_CMD_TIMEOUT & (0x01U)) << 2) | ((_m->EXHAUST_BRAKE_RPT_TIMEOUT & (0x01U)) << 3);

  cframe->MsgId = WATCHDOG_RPT_2_CANID;
  cframe->DLC = WATCHDOG_RPT_2_DLC;
  cframe->IDE = WATCHDOG_RPT_2_IDE;
  return WATCHDOG_RPT_2_CANID;
}

#else

uint32_t Pack_WATCHDOG_RPT_2_pacmod12(WATCHDOG_RPT_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < WATCHDOG_RPT_2_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->ACCEL_RPT_TIMEOUT & (0x01U)) | ((_m->BRAKE_RPT_TIMEOUT & (0x01U)) << 1) | ((_m->BRAKE_DECEL_RPT_TIMEOUT & (0x01U)) << 2) | ((_m->CABIN_CLIMATE_RPT_TIMEOUT & (0x01U)) << 3) | ((_m->CABIN_FAN_SPEED_RPT_TIMEOUT & (0x01U)) << 4) | ((_m->CABIN_TEMP_RPT_TIMEOUT & (0x01U)) << 5) | ((_m->CRUISE_CONTROL_RPT_TIMEOUT & (0x01U)) << 6) | ((_m->DASH_LEFT_RPT_TIMEOUT & (0x01U)) << 7);
  _d[1] |= (_m->DASH_RIGHT_RPT_TIMEOUT & (0x01U)) | ((_m->ENGINE_BRAKE_RPT_TIMEOUT & (0x01U)) << 1) | ((_m->HAZARD_LIGHTS_RPT_TIMEOUT & (0x01U)) << 2) | ((_m->HEADLIGHT_RPT_TIMEOUT & (0x01U)) << 3) | ((_m->HORN_RPT_TIMEOUT & (0x01U)) << 4) | ((_m->MARKER_LAMP_RPT_TIMEOUT & (0x01U)) << 5) | ((_m->MEDIA_CONTROLS_RPT_TIMEOUT & (0x01U)) << 6) | ((_m->PARKING_BRAKE_RPT_TIMEOUT & (0x01U)) << 7);
  _d[2] |= (_m->REAR_PASS_DOOR_RPT_TIMEOUT & (0x01U)) | ((_m->SHIFT_RPT_TIMEOUT & (0x01U)) << 1) | ((_m->SPRAYER_RPT_TIMEOUT & (0x01U)) << 2) | ((_m->STEERING_RPT_TIMEOUT & (0x01U)) << 3) | ((_m->TURN_RPT_TIMEOUT & (0x01U)) << 4) | ((_m->WIPER_RPT_TIMEOUT & (0x01U)) << 5) | ((_m->PACMOD1_SANITY_FAULT & (0x01U)) << 6) | ((_m->PACMOD2_SANITY_FAULT & (0x01U)) << 7);
  _d[3] |= (_m->PACMOD3_SANITY_FAULT & (0x01U)) | ((_m->PACMINI1_SANITY_FAULT & (0x01U)) << 1) | ((_m->PACMINI2_SANITY_FAULT & (0x01U)) << 2) | ((_m->PACMINI3_SANITY_FAULT & (0x01U)) << 3) | ((_m->PACMOD1_COMPONENT_RPT_TIMEOUT & (0x01U)) << 4) | ((_m->PACMOD2_COMPONENT_RPT_TIMEOUT & (0x01U)) << 5) | ((_m->PACMOD3_COMPONENT_RPT_TIMEOUT & (0x01U)) << 6) | ((_m->PACMINI1_COMPONENT_RPT_TIMEOUT & (0x01U)) << 7);
  _d[4] |= (_m->PACMINI2_COMPONENT_RPT_TIMEOUT & (0x01U)) | ((_m->PACMINI3_COMPONENT_RPT_TIMEOUT & (0x01U)) << 1) | ((_m->PACMOD1_SYSTEM_PRESENT_FAULT & (0x01U)) << 2) | ((_m->PACMOD2_SYSTEM_PRESENT_FAULT & (0x01U)) << 3) | ((_m->PACMOD3_SYSTEM_PRESENT_FAULT & (0x01U)) << 4) | ((_m->PACMINI1_SYSTEM_PRESENT_FAULT & (0x01U)) << 5) | ((_m->PACMINI2_SYSTEM_PRESENT_FAULT & (0x01U)) << 6) | ((_m->PACMINI3_SYSTEM_PRESENT_FAULT & (0x01U)) << 7);
  _d[5] |= (_m->DRIVE_MODE_INVALID & (0x01U)) | ((_m->GLOBAL_CMD_SANITY_FAULT & (0x01U)) << 1) | ((_m->GLOBAL_CMD_TIMEOUT & (0x01U)) << 2) | ((_m->EXHAUST_BRAKE_RPT_TIMEOUT & (0x01U)) << 3);

  *_len = WATCHDOG_RPT_2_DLC;
  *_ide = WATCHDOG_RPT_2_IDE;
  return WATCHDOG_RPT_2_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_VIN_RPT_2_pacmod12(VIN_RPT_2_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->VEHICLE_IDENTIFIER_SECTION_10 = (_d[0] & (0xFFU));
  _m->VEHICLE_IDENTIFIER_SECTION_11 = (_d[1] & (0xFFU));
  _m->VEHICLE_IDENTIFIER_SECTION_12 = (_d[2] & (0xFFU));
  _m->VEHICLE_IDENTIFIER_SECTION_13 = (_d[3] & (0xFFU));
  _m->VEHICLE_IDENTIFIER_SECTION_14 = (_d[4] & (0xFFU));
  _m->VEHICLE_IDENTIFIER_SECTION_15 = (_d[5] & (0xFFU));
  _m->VEHICLE_IDENTIFIER_SECTION_16 = (_d[6] & (0xFFU));
  _m->VEHICLE_IDENTIFIER_SECTION_17 = (_d[7] & (0xFFU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < VIN_RPT_2_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_VIN_RPT_2_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return VIN_RPT_2_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_VIN_RPT_2_pacmod12(VIN_RPT_2_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < VIN_RPT_2_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->VEHICLE_IDENTIFIER_SECTION_10 & (0xFFU));
  cframe->Data[1] |= (_m->VEHICLE_IDENTIFIER_SECTION_11 & (0xFFU));
  cframe->Data[2] |= (_m->VEHICLE_IDENTIFIER_SECTION_12 & (0xFFU));
  cframe->Data[3] |= (_m->VEHICLE_IDENTIFIER_SECTION_13 & (0xFFU));
  cframe->Data[4] |= (_m->VEHICLE_IDENTIFIER_SECTION_14 & (0xFFU));
  cframe->Data[5] |= (_m->VEHICLE_IDENTIFIER_SECTION_15 & (0xFFU));
  cframe->Data[6] |= (_m->VEHICLE_IDENTIFIER_SECTION_16 & (0xFFU));
  cframe->Data[7] |= (_m->VEHICLE_IDENTIFIER_SECTION_17 & (0xFFU));

  cframe->MsgId = VIN_RPT_2_CANID;
  cframe->DLC = VIN_RPT_2_DLC;
  cframe->IDE = VIN_RPT_2_IDE;
  return VIN_RPT_2_CANID;
}

#else

uint32_t Pack_VIN_RPT_2_pacmod12(VIN_RPT_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < VIN_RPT_2_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->VEHICLE_IDENTIFIER_SECTION_10 & (0xFFU));
  _d[1] |= (_m->VEHICLE_IDENTIFIER_SECTION_11 & (0xFFU));
  _d[2] |= (_m->VEHICLE_IDENTIFIER_SECTION_12 & (0xFFU));
  _d[3] |= (_m->VEHICLE_IDENTIFIER_SECTION_13 & (0xFFU));
  _d[4] |= (_m->VEHICLE_IDENTIFIER_SECTION_14 & (0xFFU));
  _d[5] |= (_m->VEHICLE_IDENTIFIER_SECTION_15 & (0xFFU));
  _d[6] |= (_m->VEHICLE_IDENTIFIER_SECTION_16 & (0xFFU));
  _d[7] |= (_m->VEHICLE_IDENTIFIER_SECTION_17 & (0xFFU));

  *_len = VIN_RPT_2_DLC;
  *_ide = VIN_RPT_2_IDE;
  return VIN_RPT_2_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_NOTIFICATION_RPT_pacmod12(NOTIFICATION_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->BUZZER_STATUS = (_d[0] & (0x03U));
  _m->LIGHT_STATUS = ((_d[0] >> 2) & (0x03U));
  _m->LIGHT_COLOR = ((_d[0] >> 4) & (0x0FU));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < NOTIFICATION_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_NOTIFICATION_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return NOTIFICATION_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_NOTIFICATION_RPT_pacmod12(NOTIFICATION_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < NOTIFICATION_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->BUZZER_STATUS & (0x03U)) | ((_m->LIGHT_STATUS & (0x03U)) << 2) | ((_m->LIGHT_COLOR & (0x0FU)) << 4);

  cframe->MsgId = NOTIFICATION_RPT_CANID;
  cframe->DLC = NOTIFICATION_RPT_DLC;
  cframe->IDE = NOTIFICATION_RPT_IDE;
  return NOTIFICATION_RPT_CANID;
}

#else

uint32_t Pack_NOTIFICATION_RPT_pacmod12(NOTIFICATION_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < NOTIFICATION_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->BUZZER_STATUS & (0x03U)) | ((_m->LIGHT_STATUS & (0x03U)) << 2) | ((_m->LIGHT_COLOR & (0x0FU)) << 4);

  *_len = NOTIFICATION_RPT_DLC;
  *_ide = NOTIFICATION_RPT_IDE;
  return NOTIFICATION_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_AIR_PRESSURE_RPT_pacmod12(AIR_PRESSURE_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->PNEUMATIC_SUPPLY_PRESSURE_ro = (_d[0] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->PNEUMATIC_SUPPLY_PRESSURE_phys = PACMOD12_PNEUMATIC_SUPPLY_PRESSURE_ro_fromS(_m->PNEUMATIC_SUPPLY_PRESSURE_ro);
#endif // PACMOD12_USE_SIGFLOAT

  _m->BRAKE_CIRCUIT_1_PRESSURE_ro = (_d[1] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->BRAKE_CIRCUIT_1_PRESSURE_phys = PACMOD12_BRAKE_CIRCUIT_1_PRESSURE_ro_fromS(_m->BRAKE_CIRCUIT_1_PRESSURE_ro);
#endif // PACMOD12_USE_SIGFLOAT

  _m->BRAKE_CIRCUIT_2_PRESSURE_ro = (_d[2] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->BRAKE_CIRCUIT_2_PRESSURE_phys = PACMOD12_BRAKE_CIRCUIT_2_PRESSURE_ro_fromS(_m->BRAKE_CIRCUIT_2_PRESSURE_ro);
#endif // PACMOD12_USE_SIGFLOAT

  _m->PARK_TRAILER_AIR_PRESSURE_ro = (_d[3] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->PARK_TRAILER_AIR_PRESSURE_phys = PACMOD12_PARK_TRAILER_AIR_PRESSURE_ro_fromS(_m->PARK_TRAILER_AIR_PRESSURE_ro);
#endif // PACMOD12_USE_SIGFLOAT

  _m->POWERTRAIN_AIR_PRESSURE_ro = (_d[4] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->POWERTRAIN_AIR_PRESSURE_phys = PACMOD12_POWERTRAIN_AIR_PRESSURE_ro_fromS(_m->POWERTRAIN_AIR_PRESSURE_ro);
#endif // PACMOD12_USE_SIGFLOAT

  _m->AIR_COMPRESSOR_STATUS = (_d[5] & (0x03U));
  _m->PNEUMATIC_SUPPLY_PRESSURE_AVAIL = (_d[6] & (0x01U));
  _m->BRAKE_CIRCUIT_1_PRESSURE_AVAIL = ((_d[6] >> 1) & (0x01U));
  _m->BRAKE_CIRCUIT_2_PRESSURE_AVAIL = ((_d[6] >> 2) & (0x01U));
  _m->PARK_TRAILER_AIR_PRESSURE_AVAIL = ((_d[6] >> 3) & (0x01U));
  _m->POWERTRAIN_AIR_PRESSURE_AVAIL = ((_d[6] >> 4) & (0x01U));
  _m->AIR_COMPRESSOR_STATUS_AVAIL = ((_d[6] >> 5) & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < AIR_PRESSURE_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_AIR_PRESSURE_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return AIR_PRESSURE_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_AIR_PRESSURE_RPT_pacmod12(AIR_PRESSURE_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < AIR_PRESSURE_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->PNEUMATIC_SUPPLY_PRESSURE_ro = PACMOD12_PNEUMATIC_SUPPLY_PRESSURE_ro_toS(_m->PNEUMATIC_SUPPLY_PRESSURE_phys);
  _m->BRAKE_CIRCUIT_1_PRESSURE_ro = PACMOD12_BRAKE_CIRCUIT_1_PRESSURE_ro_toS(_m->BRAKE_CIRCUIT_1_PRESSURE_phys);
  _m->BRAKE_CIRCUIT_2_PRESSURE_ro = PACMOD12_BRAKE_CIRCUIT_2_PRESSURE_ro_toS(_m->BRAKE_CIRCUIT_2_PRESSURE_phys);
  _m->PARK_TRAILER_AIR_PRESSURE_ro = PACMOD12_PARK_TRAILER_AIR_PRESSURE_ro_toS(_m->PARK_TRAILER_AIR_PRESSURE_phys);
  _m->POWERTRAIN_AIR_PRESSURE_ro = PACMOD12_POWERTRAIN_AIR_PRESSURE_ro_toS(_m->POWERTRAIN_AIR_PRESSURE_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[0] |= (_m->PNEUMATIC_SUPPLY_PRESSURE_ro & (0xFFU));
  cframe->Data[1] |= (_m->BRAKE_CIRCUIT_1_PRESSURE_ro & (0xFFU));
  cframe->Data[2] |= (_m->BRAKE_CIRCUIT_2_PRESSURE_ro & (0xFFU));
  cframe->Data[3] |= (_m->PARK_TRAILER_AIR_PRESSURE_ro & (0xFFU));
  cframe->Data[4] |= (_m->POWERTRAIN_AIR_PRESSURE_ro & (0xFFU));
  cframe->Data[5] |= (_m->AIR_COMPRESSOR_STATUS & (0x03U));
  cframe->Data[6] |= (_m->PNEUMATIC_SUPPLY_PRESSURE_AVAIL & (0x01U)) | ((_m->BRAKE_CIRCUIT_1_PRESSURE_AVAIL & (0x01U)) << 1) | ((_m->BRAKE_CIRCUIT_2_PRESSURE_AVAIL & (0x01U)) << 2) | ((_m->PARK_TRAILER_AIR_PRESSURE_AVAIL & (0x01U)) << 3) | ((_m->POWERTRAIN_AIR_PRESSURE_AVAIL & (0x01U)) << 4) | ((_m->AIR_COMPRESSOR_STATUS_AVAIL & (0x01U)) << 5);

  cframe->MsgId = AIR_PRESSURE_RPT_CANID;
  cframe->DLC = AIR_PRESSURE_RPT_DLC;
  cframe->IDE = AIR_PRESSURE_RPT_IDE;
  return AIR_PRESSURE_RPT_CANID;
}

#else

uint32_t Pack_AIR_PRESSURE_RPT_pacmod12(AIR_PRESSURE_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < AIR_PRESSURE_RPT_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->PNEUMATIC_SUPPLY_PRESSURE_ro = PACMOD12_PNEUMATIC_SUPPLY_PRESSURE_ro_toS(_m->PNEUMATIC_SUPPLY_PRESSURE_phys);
  _m->BRAKE_CIRCUIT_1_PRESSURE_ro = PACMOD12_BRAKE_CIRCUIT_1_PRESSURE_ro_toS(_m->BRAKE_CIRCUIT_1_PRESSURE_phys);
  _m->BRAKE_CIRCUIT_2_PRESSURE_ro = PACMOD12_BRAKE_CIRCUIT_2_PRESSURE_ro_toS(_m->BRAKE_CIRCUIT_2_PRESSURE_phys);
  _m->PARK_TRAILER_AIR_PRESSURE_ro = PACMOD12_PARK_TRAILER_AIR_PRESSURE_ro_toS(_m->PARK_TRAILER_AIR_PRESSURE_phys);
  _m->POWERTRAIN_AIR_PRESSURE_ro = PACMOD12_POWERTRAIN_AIR_PRESSURE_ro_toS(_m->POWERTRAIN_AIR_PRESSURE_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[0] |= (_m->PNEUMATIC_SUPPLY_PRESSURE_ro & (0xFFU));
  _d[1] |= (_m->BRAKE_CIRCUIT_1_PRESSURE_ro & (0xFFU));
  _d[2] |= (_m->BRAKE_CIRCUIT_2_PRESSURE_ro & (0xFFU));
  _d[3] |= (_m->PARK_TRAILER_AIR_PRESSURE_ro & (0xFFU));
  _d[4] |= (_m->POWERTRAIN_AIR_PRESSURE_ro & (0xFFU));
  _d[5] |= (_m->AIR_COMPRESSOR_STATUS & (0x03U));
  _d[6] |= (_m->PNEUMATIC_SUPPLY_PRESSURE_AVAIL & (0x01U)) | ((_m->BRAKE_CIRCUIT_1_PRESSURE_AVAIL & (0x01U)) << 1) | ((_m->BRAKE_CIRCUIT_2_PRESSURE_AVAIL & (0x01U)) << 2) | ((_m->PARK_TRAILER_AIR_PRESSURE_AVAIL & (0x01U)) << 3) | ((_m->POWERTRAIN_AIR_PRESSURE_AVAIL & (0x01U)) << 4) | ((_m->AIR_COMPRESSOR_STATUS_AVAIL & (0x01U)) << 5);

  *_len = AIR_PRESSURE_RPT_DLC;
  *_ide = AIR_PRESSURE_RPT_IDE;
  return AIR_PRESSURE_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_ENGINE_RPT_2_pacmod12(ENGINE_RPT_2_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->FUEL_RATE_ro = ((_d[0] & (0xFFU)) << 8) | (_d[1] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->FUEL_RATE_phys = (sigfloat_t)(PACMOD12_FUEL_RATE_ro_fromS(_m->FUEL_RATE_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->FUEL_RATE_AVAIL = (_d[2] & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < ENGINE_RPT_2_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_ENGINE_RPT_2_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return ENGINE_RPT_2_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_ENGINE_RPT_2_pacmod12(ENGINE_RPT_2_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < ENGINE_RPT_2_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->FUEL_RATE_ro = PACMOD12_FUEL_RATE_ro_toS(_m->FUEL_RATE_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[0] |= ((_m->FUEL_RATE_ro >> 8) & (0xFFU));
  cframe->Data[1] |= (_m->FUEL_RATE_ro & (0xFFU));
  cframe->Data[2] |= (_m->FUEL_RATE_AVAIL & (0x01U));

  cframe->MsgId = ENGINE_RPT_2_CANID;
  cframe->DLC = ENGINE_RPT_2_DLC;
  cframe->IDE = ENGINE_RPT_2_IDE;
  return ENGINE_RPT_2_CANID;
}

#else

uint32_t Pack_ENGINE_RPT_2_pacmod12(ENGINE_RPT_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < ENGINE_RPT_2_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->FUEL_RATE_ro = PACMOD12_FUEL_RATE_ro_toS(_m->FUEL_RATE_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[0] |= ((_m->FUEL_RATE_ro >> 8) & (0xFFU));
  _d[1] |= (_m->FUEL_RATE_ro & (0xFFU));
  _d[2] |= (_m->FUEL_RATE_AVAIL & (0x01U));

  *_len = ENGINE_RPT_2_DLC;
  *_ide = ENGINE_RPT_2_IDE;
  return ENGINE_RPT_2_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_WHEEL_SPEED_RPT_2_pacmod12(WHEEL_SPEED_RPT_2_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->WHEEL_SPD_AXLE_3_LEFT_ro = ((_d[0] & (0xFFU)) << 8) | (_d[1] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->WHEEL_SPD_AXLE_3_LEFT_phys = (sigfloat_t)(PACMOD12_WHEEL_SPD_AXLE_3_LEFT_ro_fromS(_m->WHEEL_SPD_AXLE_3_LEFT_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->WHEEL_SPD_AXLE_3_RIGHT_ro = ((_d[2] & (0xFFU)) << 8) | (_d[3] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->WHEEL_SPD_AXLE_3_RIGHT_phys = (sigfloat_t)(PACMOD12_WHEEL_SPD_AXLE_3_RIGHT_ro_fromS(_m->WHEEL_SPD_AXLE_3_RIGHT_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->WHEEL_SPD_AXLE_4_LEFT_ro = ((_d[4] & (0xFFU)) << 8) | (_d[5] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->WHEEL_SPD_AXLE_4_LEFT_phys = (sigfloat_t)(PACMOD12_WHEEL_SPD_AXLE_4_LEFT_ro_fromS(_m->WHEEL_SPD_AXLE_4_LEFT_ro));
#endif // PACMOD12_USE_SIGFLOAT

  _m->WHEEL_SPD_AXLE_4_RIGHT_ro = ((_d[6] & (0xFFU)) << 8) | (_d[7] & (0xFFU));
#ifdef PACMOD12_USE_SIGFLOAT
  _m->WHEEL_SPD_AXLE_4_RIGHT_phys = (sigfloat_t)(PACMOD12_WHEEL_SPD_AXLE_4_RIGHT_ro_fromS(_m->WHEEL_SPD_AXLE_4_RIGHT_ro));
#endif // PACMOD12_USE_SIGFLOAT

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < WHEEL_SPEED_RPT_2_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_WHEEL_SPEED_RPT_2_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return WHEEL_SPEED_RPT_2_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_WHEEL_SPEED_RPT_2_pacmod12(WHEEL_SPEED_RPT_2_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < WHEEL_SPEED_RPT_2_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->WHEEL_SPD_AXLE_3_LEFT_ro = PACMOD12_WHEEL_SPD_AXLE_3_LEFT_ro_toS(_m->WHEEL_SPD_AXLE_3_LEFT_phys);
  _m->WHEEL_SPD_AXLE_3_RIGHT_ro = PACMOD12_WHEEL_SPD_AXLE_3_RIGHT_ro_toS(_m->WHEEL_SPD_AXLE_3_RIGHT_phys);
  _m->WHEEL_SPD_AXLE_4_LEFT_ro = PACMOD12_WHEEL_SPD_AXLE_4_LEFT_ro_toS(_m->WHEEL_SPD_AXLE_4_LEFT_phys);
  _m->WHEEL_SPD_AXLE_4_RIGHT_ro = PACMOD12_WHEEL_SPD_AXLE_4_RIGHT_ro_toS(_m->WHEEL_SPD_AXLE_4_RIGHT_phys);
#endif // PACMOD12_USE_SIGFLOAT

  cframe->Data[0] |= ((_m->WHEEL_SPD_AXLE_3_LEFT_ro >> 8) & (0xFFU));
  cframe->Data[1] |= (_m->WHEEL_SPD_AXLE_3_LEFT_ro & (0xFFU));
  cframe->Data[2] |= ((_m->WHEEL_SPD_AXLE_3_RIGHT_ro >> 8) & (0xFFU));
  cframe->Data[3] |= (_m->WHEEL_SPD_AXLE_3_RIGHT_ro & (0xFFU));
  cframe->Data[4] |= ((_m->WHEEL_SPD_AXLE_4_LEFT_ro >> 8) & (0xFFU));
  cframe->Data[5] |= (_m->WHEEL_SPD_AXLE_4_LEFT_ro & (0xFFU));
  cframe->Data[6] |= ((_m->WHEEL_SPD_AXLE_4_RIGHT_ro >> 8) & (0xFFU));
  cframe->Data[7] |= (_m->WHEEL_SPD_AXLE_4_RIGHT_ro & (0xFFU));

  cframe->MsgId = WHEEL_SPEED_RPT_2_CANID;
  cframe->DLC = WHEEL_SPEED_RPT_2_DLC;
  cframe->IDE = WHEEL_SPEED_RPT_2_IDE;
  return WHEEL_SPEED_RPT_2_CANID;
}

#else

uint32_t Pack_WHEEL_SPEED_RPT_2_pacmod12(WHEEL_SPEED_RPT_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < WHEEL_SPEED_RPT_2_DLC) && (i < 8); _d[i++] = 0);

#ifdef PACMOD12_USE_SIGFLOAT
  _m->WHEEL_SPD_AXLE_3_LEFT_ro = PACMOD12_WHEEL_SPD_AXLE_3_LEFT_ro_toS(_m->WHEEL_SPD_AXLE_3_LEFT_phys);
  _m->WHEEL_SPD_AXLE_3_RIGHT_ro = PACMOD12_WHEEL_SPD_AXLE_3_RIGHT_ro_toS(_m->WHEEL_SPD_AXLE_3_RIGHT_phys);
  _m->WHEEL_SPD_AXLE_4_LEFT_ro = PACMOD12_WHEEL_SPD_AXLE_4_LEFT_ro_toS(_m->WHEEL_SPD_AXLE_4_LEFT_phys);
  _m->WHEEL_SPD_AXLE_4_RIGHT_ro = PACMOD12_WHEEL_SPD_AXLE_4_RIGHT_ro_toS(_m->WHEEL_SPD_AXLE_4_RIGHT_phys);
#endif // PACMOD12_USE_SIGFLOAT

  _d[0] |= ((_m->WHEEL_SPD_AXLE_3_LEFT_ro >> 8) & (0xFFU));
  _d[1] |= (_m->WHEEL_SPD_AXLE_3_LEFT_ro & (0xFFU));
  _d[2] |= ((_m->WHEEL_SPD_AXLE_3_RIGHT_ro >> 8) & (0xFFU));
  _d[3] |= (_m->WHEEL_SPD_AXLE_3_RIGHT_ro & (0xFFU));
  _d[4] |= ((_m->WHEEL_SPD_AXLE_4_LEFT_ro >> 8) & (0xFFU));
  _d[5] |= (_m->WHEEL_SPD_AXLE_4_LEFT_ro & (0xFFU));
  _d[6] |= ((_m->WHEEL_SPD_AXLE_4_RIGHT_ro >> 8) & (0xFFU));
  _d[7] |= (_m->WHEEL_SPD_AXLE_4_RIGHT_ro & (0xFFU));

  *_len = WHEEL_SPEED_RPT_2_DLC;
  *_ide = WHEEL_SPEED_RPT_2_IDE;
  return WHEEL_SPEED_RPT_2_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_WATCHDOG_RPT_pacmod12(WATCHDOG_RPT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->GLOBAL_ENABLE_FLAG = (_d[0] & (0x01U));
  _m->GLOBAL_OVERRIDE_ACTIVE = ((_d[0] >> 1) & (0x01U));
  _m->GLOBAL_COMMAND_TIMEOUT_ERROR = ((_d[0] >> 2) & (0x01U));
  _m->GLOBAL_PACMOD_SUBSYSTEM_TIMEOUT = ((_d[0] >> 3) & (0x01U));
  _m->GLOBAL_VEHICLE_CAN_TIMEOUT = ((_d[0] >> 4) & (0x01U));
  _m->GLOBAL_PACMOD_SYS_FAULT_ACTIVE = ((_d[0] >> 5) & (0x01U));
  _m->GLOBAL_CONFIG_FAULT_ACTIVE = ((_d[0] >> 6) & (0x01U));
  _m->GLOBAL_TIMEOUT = ((_d[0] >> 7) & (0x01U));
  _m->ACCEL_ENABLED = (_d[1] & (0x01U));
  _m->ACCEL_OVERRIDE_ACTIVE = ((_d[1] >> 1) & (0x01U));
  _m->ACCEL_COMMAND_OUTPUT_FAULT = ((_d[1] >> 2) & (0x01U));
  _m->ACCEL_INPUT_OUTPUT_FAULT = ((_d[1] >> 3) & (0x01U));
  _m->ACCEL_OUTPUT_REPORTED_FAULT = ((_d[1] >> 4) & (0x01U));
  _m->ACCEL_PACMOD_FAULT = ((_d[1] >> 5) & (0x01U));
  _m->ACCEL_VEHICLE_FAULT = ((_d[1] >> 6) & (0x01U));
  _m->ACCEL_TIMEOUT = ((_d[1] >> 7) & (0x01U));
  _m->BRAKE_ENABLED = (_d[2] & (0x01U));
  _m->BRAKE_OVERRIDE_ACTIVE = ((_d[2] >> 1) & (0x01U));
  _m->BRAKE_COMMAND_OUTPUT_FAULT = ((_d[2] >> 2) & (0x01U));
  _m->BRAKE_INPUT_OUTPUT_FAULT = ((_d[2] >> 3) & (0x01U));
  _m->BRAKE_OUTPUT_REPORTED_FAULT = ((_d[2] >> 4) & (0x01U));
  _m->BRAKE_PACMOD_FAULT = ((_d[2] >> 5) & (0x01U));
  _m->BRAKE_VEHICLE_FAULT = ((_d[2] >> 6) & (0x01U));
  _m->BRAKE_TIMEOUT = ((_d[2] >> 7) & (0x01U));
  _m->SHIFT_ENABLED = (_d[3] & (0x01U));
  _m->SHIFT_OVERRIDE_ACTIVE = ((_d[3] >> 1) & (0x01U));
  _m->SHIFT_COMMAND_OUTPUT_FAULT = ((_d[3] >> 2) & (0x01U));
  _m->SHIFT_INPUT_OUTPUT_FAULT = ((_d[3] >> 3) & (0x01U));
  _m->SHIFT_OUTPUT_REPORTED_FAULT = ((_d[3] >> 4) & (0x01U));
  _m->SHIFT_PACMOD_FAULT = ((_d[3] >> 5) & (0x01U));
  _m->SHIFT_VEHICLE_FAULT = ((_d[3] >> 6) & (0x01U));
  _m->SHIFT_TIMEOUT = ((_d[3] >> 7) & (0x01U));
  _m->STEER_ENABLED = (_d[4] & (0x01U));
  _m->STEER_OVERRIDE_ACTIVE = ((_d[4] >> 1) & (0x01U));
  _m->STEER_COMMAND_OUTPUT_FAULT = ((_d[4] >> 2) & (0x01U));
  _m->STEER_INPUT_OUTPUT_FAULT = ((_d[4] >> 3) & (0x01U));
  _m->STEER_OUTPUT_REPORTED_FAULT = ((_d[4] >> 4) & (0x01U));
  _m->STEER_PACMOD_FAULT = ((_d[4] >> 5) & (0x01U));
  _m->STEER_VEHICLE_FAULT = ((_d[4] >> 6) & (0x01U));
  _m->STEER_TIMEOUT = ((_d[4] >> 7) & (0x01U));
  _m->PACMOD1_CONFIG_FAULT = (_d[5] & (0x01U));
  _m->PACMOD1_CAN_TIMEOUT = ((_d[5] >> 1) & (0x01U));
  _m->PACMOD1_COUNTER_FAULT = ((_d[5] >> 2) & (0x01U));
  _m->PACMOD2_CONFIG_FAULT = ((_d[5] >> 3) & (0x01U));
  _m->PACMOD2_CAN_TIMEOUT = ((_d[5] >> 4) & (0x01U));
  _m->PACMOD2_COUNTER_FAULT = ((_d[5] >> 5) & (0x01U));
  _m->PACMOD3_CONFIG_FAULT = ((_d[5] >> 6) & (0x01U));
  _m->PACMOD3_CAN_TIMEOUT = ((_d[5] >> 7) & (0x01U));
  _m->PACMOD3_COUNTER_FAULT = (_d[6] & (0x01U));
  _m->PACMINI1_RPT_TIMEOUT = ((_d[6] >> 1) & (0x01U));
  _m->PACMINI1_CONFIG_FAULT = ((_d[6] >> 2) & (0x01U));
  _m->PACMINI1_CAN_TIMEOUT = ((_d[6] >> 3) & (0x01U));
  _m->PACMINI1_COUNTER_FAULT = ((_d[6] >> 4) & (0x01U));
  _m->PACMINI2_RPT_TIMEOUT = ((_d[6] >> 5) & (0x01U));
  _m->PACMINI2_CONFIG_FAULT = ((_d[6] >> 6) & (0x01U));
  _m->PACMINI2_CAN_TIMEOUT = ((_d[6] >> 7) & (0x01U));
  _m->PACMINI2_COUNTER_FAULT = (_d[7] & (0x01U));
  _m->PACMINI3_RPT_TIMEOUT = ((_d[7] >> 1) & (0x01U));
  _m->PACMINI3_CONFIG_FAULT = ((_d[7] >> 2) & (0x01U));
  _m->PACMINI3_CAN_TIMEOUT = ((_d[7] >> 3) & (0x01U));
  _m->PACMINI3_COUNTER_FAULT = ((_d[7] >> 4) & (0x01U));
  _m->PACMOD_SYSTEM_PRESENT_FAULT = ((_d[7] >> 5) & (0x01U));
  _m->PACMINI_SYSTEM_PRESENT_FAULT = ((_d[7] >> 6) & (0x01U));
  _m->GLOBAL_INT_POWER_SUPPLY_FAULT = ((_d[7] >> 7) & (0x01U));

#ifdef PACMOD12_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < WATCHDOG_RPT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_WATCHDOG_RPT_pacmod12(&_m->mon1);
#endif // PACMOD12_USE_DIAG_MONITORS

 return WATCHDOG_RPT_CANID;
}

#ifdef PACMOD12_USE_CANSTRUCT

uint32_t Pack_WATCHDOG_RPT_pacmod12(WATCHDOG_RPT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < WATCHDOG_RPT_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->GLOBAL_ENABLE_FLAG & (0x01U)) | ((_m->GLOBAL_OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->GLOBAL_COMMAND_TIMEOUT_ERROR & (0x01U)) << 2) | ((_m->GLOBAL_PACMOD_SUBSYSTEM_TIMEOUT & (0x01U)) << 3) | ((_m->GLOBAL_VEHICLE_CAN_TIMEOUT & (0x01U)) << 4) | ((_m->GLOBAL_PACMOD_SYS_FAULT_ACTIVE & (0x01U)) << 5) | ((_m->GLOBAL_CONFIG_FAULT_ACTIVE & (0x01U)) << 6) | ((_m->GLOBAL_TIMEOUT & (0x01U)) << 7);
  cframe->Data[1] |= (_m->ACCEL_ENABLED & (0x01U)) | ((_m->ACCEL_OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->ACCEL_COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->ACCEL_INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->ACCEL_OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->ACCEL_PACMOD_FAULT & (0x01U)) << 5) | ((_m->ACCEL_VEHICLE_FAULT & (0x01U)) << 6) | ((_m->ACCEL_TIMEOUT & (0x01U)) << 7);
  cframe->Data[2] |= (_m->BRAKE_ENABLED & (0x01U)) | ((_m->BRAKE_OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->BRAKE_COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->BRAKE_INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->BRAKE_OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->BRAKE_PACMOD_FAULT & (0x01U)) << 5) | ((_m->BRAKE_VEHICLE_FAULT & (0x01U)) << 6) | ((_m->BRAKE_TIMEOUT & (0x01U)) << 7);
  cframe->Data[3] |= (_m->SHIFT_ENABLED & (0x01U)) | ((_m->SHIFT_OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->SHIFT_COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->SHIFT_INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->SHIFT_OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->SHIFT_PACMOD_FAULT & (0x01U)) << 5) | ((_m->SHIFT_VEHICLE_FAULT & (0x01U)) << 6) | ((_m->SHIFT_TIMEOUT & (0x01U)) << 7);
  cframe->Data[4] |= (_m->STEER_ENABLED & (0x01U)) | ((_m->STEER_OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->STEER_COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->STEER_INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->STEER_OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->STEER_PACMOD_FAULT & (0x01U)) << 5) | ((_m->STEER_VEHICLE_FAULT & (0x01U)) << 6) | ((_m->STEER_TIMEOUT & (0x01U)) << 7);
  cframe->Data[5] |= (_m->PACMOD1_CONFIG_FAULT & (0x01U)) | ((_m->PACMOD1_CAN_TIMEOUT & (0x01U)) << 1) | ((_m->PACMOD1_COUNTER_FAULT & (0x01U)) << 2) | ((_m->PACMOD2_CONFIG_FAULT & (0x01U)) << 3) | ((_m->PACMOD2_CAN_TIMEOUT & (0x01U)) << 4) | ((_m->PACMOD2_COUNTER_FAULT & (0x01U)) << 5) | ((_m->PACMOD3_CONFIG_FAULT & (0x01U)) << 6) | ((_m->PACMOD3_CAN_TIMEOUT & (0x01U)) << 7);
  cframe->Data[6] |= (_m->PACMOD3_COUNTER_FAULT & (0x01U)) | ((_m->PACMINI1_RPT_TIMEOUT & (0x01U)) << 1) | ((_m->PACMINI1_CONFIG_FAULT & (0x01U)) << 2) | ((_m->PACMINI1_CAN_TIMEOUT & (0x01U)) << 3) | ((_m->PACMINI1_COUNTER_FAULT & (0x01U)) << 4) | ((_m->PACMINI2_RPT_TIMEOUT & (0x01U)) << 5) | ((_m->PACMINI2_CONFIG_FAULT & (0x01U)) << 6) | ((_m->PACMINI2_CAN_TIMEOUT & (0x01U)) << 7);
  cframe->Data[7] |= (_m->PACMINI2_COUNTER_FAULT & (0x01U)) | ((_m->PACMINI3_RPT_TIMEOUT & (0x01U)) << 1) | ((_m->PACMINI3_CONFIG_FAULT & (0x01U)) << 2) | ((_m->PACMINI3_CAN_TIMEOUT & (0x01U)) << 3) | ((_m->PACMINI3_COUNTER_FAULT & (0x01U)) << 4) | ((_m->PACMOD_SYSTEM_PRESENT_FAULT & (0x01U)) << 5) | ((_m->PACMINI_SYSTEM_PRESENT_FAULT & (0x01U)) << 6) | ((_m->GLOBAL_INT_POWER_SUPPLY_FAULT & (0x01U)) << 7);

  cframe->MsgId = WATCHDOG_RPT_CANID;
  cframe->DLC = WATCHDOG_RPT_DLC;
  cframe->IDE = WATCHDOG_RPT_IDE;
  return WATCHDOG_RPT_CANID;
}

#else

uint32_t Pack_WATCHDOG_RPT_pacmod12(WATCHDOG_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < WATCHDOG_RPT_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->GLOBAL_ENABLE_FLAG & (0x01U)) | ((_m->GLOBAL_OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->GLOBAL_COMMAND_TIMEOUT_ERROR & (0x01U)) << 2) | ((_m->GLOBAL_PACMOD_SUBSYSTEM_TIMEOUT & (0x01U)) << 3) | ((_m->GLOBAL_VEHICLE_CAN_TIMEOUT & (0x01U)) << 4) | ((_m->GLOBAL_PACMOD_SYS_FAULT_ACTIVE & (0x01U)) << 5) | ((_m->GLOBAL_CONFIG_FAULT_ACTIVE & (0x01U)) << 6) | ((_m->GLOBAL_TIMEOUT & (0x01U)) << 7);
  _d[1] |= (_m->ACCEL_ENABLED & (0x01U)) | ((_m->ACCEL_OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->ACCEL_COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->ACCEL_INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->ACCEL_OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->ACCEL_PACMOD_FAULT & (0x01U)) << 5) | ((_m->ACCEL_VEHICLE_FAULT & (0x01U)) << 6) | ((_m->ACCEL_TIMEOUT & (0x01U)) << 7);
  _d[2] |= (_m->BRAKE_ENABLED & (0x01U)) | ((_m->BRAKE_OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->BRAKE_COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->BRAKE_INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->BRAKE_OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->BRAKE_PACMOD_FAULT & (0x01U)) << 5) | ((_m->BRAKE_VEHICLE_FAULT & (0x01U)) << 6) | ((_m->BRAKE_TIMEOUT & (0x01U)) << 7);
  _d[3] |= (_m->SHIFT_ENABLED & (0x01U)) | ((_m->SHIFT_OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->SHIFT_COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->SHIFT_INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->SHIFT_OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->SHIFT_PACMOD_FAULT & (0x01U)) << 5) | ((_m->SHIFT_VEHICLE_FAULT & (0x01U)) << 6) | ((_m->SHIFT_TIMEOUT & (0x01U)) << 7);
  _d[4] |= (_m->STEER_ENABLED & (0x01U)) | ((_m->STEER_OVERRIDE_ACTIVE & (0x01U)) << 1) | ((_m->STEER_COMMAND_OUTPUT_FAULT & (0x01U)) << 2) | ((_m->STEER_INPUT_OUTPUT_FAULT & (0x01U)) << 3) | ((_m->STEER_OUTPUT_REPORTED_FAULT & (0x01U)) << 4) | ((_m->STEER_PACMOD_FAULT & (0x01U)) << 5) | ((_m->STEER_VEHICLE_FAULT & (0x01U)) << 6) | ((_m->STEER_TIMEOUT & (0x01U)) << 7);
  _d[5] |= (_m->PACMOD1_CONFIG_FAULT & (0x01U)) | ((_m->PACMOD1_CAN_TIMEOUT & (0x01U)) << 1) | ((_m->PACMOD1_COUNTER_FAULT & (0x01U)) << 2) | ((_m->PACMOD2_CONFIG_FAULT & (0x01U)) << 3) | ((_m->PACMOD2_CAN_TIMEOUT & (0x01U)) << 4) | ((_m->PACMOD2_COUNTER_FAULT & (0x01U)) << 5) | ((_m->PACMOD3_CONFIG_FAULT & (0x01U)) << 6) | ((_m->PACMOD3_CAN_TIMEOUT & (0x01U)) << 7);
  _d[6] |= (_m->PACMOD3_COUNTER_FAULT & (0x01U)) | ((_m->PACMINI1_RPT_TIMEOUT & (0x01U)) << 1) | ((_m->PACMINI1_CONFIG_FAULT & (0x01U)) << 2) | ((_m->PACMINI1_CAN_TIMEOUT & (0x01U)) << 3) | ((_m->PACMINI1_COUNTER_FAULT & (0x01U)) << 4) | ((_m->PACMINI2_RPT_TIMEOUT & (0x01U)) << 5) | ((_m->PACMINI2_CONFIG_FAULT & (0x01U)) << 6) | ((_m->PACMINI2_CAN_TIMEOUT & (0x01U)) << 7);
  _d[7] |= (_m->PACMINI2_COUNTER_FAULT & (0x01U)) | ((_m->PACMINI3_RPT_TIMEOUT & (0x01U)) << 1) | ((_m->PACMINI3_CONFIG_FAULT & (0x01U)) << 2) | ((_m->PACMINI3_CAN_TIMEOUT & (0x01U)) << 3) | ((_m->PACMINI3_COUNTER_FAULT & (0x01U)) << 4) | ((_m->PACMOD_SYSTEM_PRESENT_FAULT & (0x01U)) << 5) | ((_m->PACMINI_SYSTEM_PRESENT_FAULT & (0x01U)) << 6) | ((_m->GLOBAL_INT_POWER_SUPPLY_FAULT & (0x01U)) << 7);

  *_len = WATCHDOG_RPT_DLC;
  *_ide = WATCHDOG_RPT_IDE;
  return WATCHDOG_RPT_CANID;
}

#endif // PACMOD12_USE_CANSTRUCT

