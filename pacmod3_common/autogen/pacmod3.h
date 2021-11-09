#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// DBC file version
#define VER_PACMOD3_MAJ (0U)
#define VER_PACMOD3_MIN (0U)

// include current dbc-driver compilation config
#include "pacmod3-config.h"

#ifdef PACMOD3_USE_DIAG_MONITORS
// This file must define:
// base monitor struct
// function signature for HASH calculation: (@GetFrameHash)
// function signature for getting system tick value: (@GetSystemTick)
#include "canmonitorutil.h"

#endif // PACMOD3_USE_DIAG_MONITORS


// def @GLOBAL_RPT CAN Message (16   0x10)
#define GLOBAL_RPT_IDE (0U)
#define GLOBAL_RPT_DLC (8U)
#define GLOBAL_RPT_CANID (0x10)

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  //  0 : "Control Disabled"
  //  1 : "Control Enabled"
  // 
  uint8_t PACMOD_SYSTEM_ENABLED : 1;           //      Bits= 1

  //  0 : "Not Overridden"
  //  1 : "Overridden"
  // 
  uint8_t PACMOD_SYSTEM_OVERRIDE_ACTIVE : 1;   //      Bits= 1

  uint8_t USR_CAN_TIMEOUT : 1;                 //      Bits= 1

  uint8_t STR_CAN_TIMEOUT : 1;                 //      Bits= 1

  //  0 : "No Active CAN Timeout"
  //  1 : "Active CAN Timeout"
  // 
  uint8_t BRK_CAN_TIMEOUT : 1;                 //      Bits= 1

  uint8_t PACMOD_SUBSYSTEM_TIMEOUT : 1;        //      Bits= 1

  uint8_t VEH_CAN_TIMEOUT : 1;                 //      Bits= 1

  uint8_t PACMOD_SYSTEM_FAULT_ACTIVE : 1;      //      Bits= 1

  uint8_t CONFIG_FAULT_ACTIVE : 1;             //      Bits= 1

  uint16_t USR_CAN_READ_ERRORS;                //      Bits=16

#else

  //  0 : "Control Disabled"
  //  1 : "Control Enabled"
  // 
  uint8_t PACMOD_SYSTEM_ENABLED;               //      Bits= 1

  //  0 : "Not Overridden"
  //  1 : "Overridden"
  // 
  uint8_t PACMOD_SYSTEM_OVERRIDE_ACTIVE;       //      Bits= 1

  uint8_t USR_CAN_TIMEOUT;                     //      Bits= 1

  uint8_t STR_CAN_TIMEOUT;                     //      Bits= 1

  //  0 : "No Active CAN Timeout"
  //  1 : "Active CAN Timeout"
  // 
  uint8_t BRK_CAN_TIMEOUT;                     //      Bits= 1

  uint8_t PACMOD_SUBSYSTEM_TIMEOUT;            //      Bits= 1

  uint8_t VEH_CAN_TIMEOUT;                     //      Bits= 1

  uint8_t PACMOD_SYSTEM_FAULT_ACTIVE;          //      Bits= 1

  uint8_t CONFIG_FAULT_ACTIVE;                 //      Bits= 1

  uint16_t USR_CAN_READ_ERRORS;                //      Bits=16

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} GLOBAL_RPT_t;

// def @COMPONENT_RPT CAN Message (32   0x20)
#define COMPONENT_RPT_IDE (0U)
#define COMPONENT_RPT_DLC (4U)
#define COMPONENT_RPT_CANID (0x20)

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  //  0 : "PACMod"
  //  1 : "PACMini"
  //  2 : "PACMicro"
  // 
  uint8_t COMPONENT_TYPE;                    //      Bits= 8

  //  0 : "PACMOD"
  //  1 : "STEERING_AND_STEERING_COLUMN"
  //  2 : "ACCELERATOR_AND_BRAKING"
  //  3 : "BRAKING"
  //  4 : "SHIFTING"
  //  5 : "STEERING"
  //  6 : "E_SHIFTER"
  //  7 : "WATCHDOG"
  // 
  uint8_t COMPONENT_FUNC;                    //      Bits= 8

  uint8_t COUNTER : 4;                       //      Bits= 4

  uint8_t COMPLEMENT : 4;                    //      Bits= 4

  uint8_t CONFIG_FAULT : 1;                  //      Bits= 1

#else

  //  0 : "PACMod"
  //  1 : "PACMini"
  //  2 : "PACMicro"
  // 
  uint8_t COMPONENT_TYPE;                    //      Bits= 8

  //  0 : "PACMOD"
  //  1 : "STEERING_AND_STEERING_COLUMN"
  //  2 : "ACCELERATOR_AND_BRAKING"
  //  3 : "BRAKING"
  //  4 : "SHIFTING"
  //  5 : "STEERING"
  //  6 : "E_SHIFTER"
  //  7 : "WATCHDOG"
  // 
  uint8_t COMPONENT_FUNC;                    //      Bits= 8

  uint8_t COUNTER;                           //      Bits= 4

  uint8_t COMPLEMENT;                        //      Bits= 4

  uint8_t CONFIG_FAULT;                      //      Bits= 1

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} COMPONENT_RPT_t;

// def @ACCEL_CMD CAN Message (256  0x100)
#define ACCEL_CMD_IDE (0U)
#define ACCEL_CMD_DLC (8U)
#define ACCEL_CMD_CANID (0x100)
// signal: @ACCEL_CMD_ro
#define PACMOD3_ACCEL_CMD_ro_CovFactor (0.001000)
#define PACMOD3_ACCEL_CMD_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD3_ACCEL_CMD_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t ENABLE : 1;                        //      Bits= 1

  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  uint8_t CLEAR_FAULTS : 1;                  //      Bits= 1

  uint16_t ACCEL_CMD_ro;                     //      Bits=16 Factor= 0.001000       

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t ACCEL_CMD_phys;
#endif // PACMOD3_USE_SIGFLOAT

#else

  uint8_t ENABLE;                            //      Bits= 1

  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  uint8_t CLEAR_FAULTS;                      //      Bits= 1

  uint16_t ACCEL_CMD_ro;                     //      Bits=16 Factor= 0.001000       

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t ACCEL_CMD_phys;
#endif // PACMOD3_USE_SIGFLOAT

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} ACCEL_CMD_t;

// def @BRAKE_CMD CAN Message (260  0x104)
#define BRAKE_CMD_IDE (0U)
#define BRAKE_CMD_DLC (8U)
#define BRAKE_CMD_CANID (0x104)
// signal: @BRAKE_CMD_ro
#define PACMOD3_BRAKE_CMD_ro_CovFactor (0.001000)
#define PACMOD3_BRAKE_CMD_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD3_BRAKE_CMD_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t ENABLE : 1;                        //      Bits= 1

  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  uint8_t CLEAR_FAULTS : 1;                  //      Bits= 1

  uint16_t BRAKE_CMD_ro;                     //      Bits=16 Factor= 0.001000       

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t BRAKE_CMD_phys;
#endif // PACMOD3_USE_SIGFLOAT

#else

  uint8_t ENABLE;                            //      Bits= 1

  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  uint8_t CLEAR_FAULTS;                      //      Bits= 1

  uint16_t BRAKE_CMD_ro;                     //      Bits=16 Factor= 0.001000       

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t BRAKE_CMD_phys;
#endif // PACMOD3_USE_SIGFLOAT

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} BRAKE_CMD_t;

// def @CRUISE_CONTROL_BUTTONS_CMD CAN Message (264  0x108)
#define CRUISE_CONTROL_BUTTONS_CMD_IDE (0U)
#define CRUISE_CONTROL_BUTTONS_CMD_DLC (8U)
#define CRUISE_CONTROL_BUTTONS_CMD_CANID (0x108)

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t ENABLE : 1;                        //      Bits= 1

  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  uint8_t CLEAR_FAULTS : 1;                  //      Bits= 1

  //  6 : "CRUISE_CONTROL_ON_OFF"
  //  5 : "CRUISE_CONTROL_RES_INC"
  //  4 : "CRUISE_CONTROL_SET_DEC"
  //  3 : "CRUISE_CONTROL_ACC_CLOSER"
  //  2 : "CRUISE_CONTROL_ACC_FURTHER"
  //  1 : "CRUISE_CONTROL_CNCL"
  //  0 : "CRUISE_CONTROL_NONE"
  // 
  uint8_t CRUISE_CONTROL_BUTTON;             //      Bits= 8

#else

  uint8_t ENABLE;                            //      Bits= 1

  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  uint8_t CLEAR_FAULTS;                      //      Bits= 1

  //  6 : "CRUISE_CONTROL_ON_OFF"
  //  5 : "CRUISE_CONTROL_RES_INC"
  //  4 : "CRUISE_CONTROL_SET_DEC"
  //  3 : "CRUISE_CONTROL_ACC_CLOSER"
  //  2 : "CRUISE_CONTROL_ACC_FURTHER"
  //  1 : "CRUISE_CONTROL_CNCL"
  //  0 : "CRUISE_CONTROL_NONE"
  // 
  uint8_t CRUISE_CONTROL_BUTTON;             //      Bits= 8

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} CRUISE_CONTROL_BUTTONS_CMD_t;

// def @DASH_CONTROLS_LEFT_CMD CAN Message (268  0x10c)
#define DASH_CONTROLS_LEFT_CMD_IDE (0U)
#define DASH_CONTROLS_LEFT_CMD_DLC (8U)
#define DASH_CONTROLS_LEFT_CMD_CANID (0x10c)

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t ENABLE : 1;                        //      Bits= 1

  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  uint8_t CLEAR_FAULTS : 1;                  //      Bits= 1

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  // 
  uint8_t DASH_CONTROLS_BUTTON;              //      Bits= 8

#else

  uint8_t ENABLE;                            //      Bits= 1

  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  uint8_t CLEAR_FAULTS;                      //      Bits= 1

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  // 
  uint8_t DASH_CONTROLS_BUTTON;              //      Bits= 8

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} DASH_CONTROLS_LEFT_CMD_t;

// def @DASH_CONTROLS_RIGHT_CMD CAN Message (272  0x110)
#define DASH_CONTROLS_RIGHT_CMD_IDE (0U)
#define DASH_CONTROLS_RIGHT_CMD_DLC (8U)
#define DASH_CONTROLS_RIGHT_CMD_CANID (0x110)

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t ENABLE : 1;                        //      Bits= 1

  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  uint8_t CLEAR_FAULTS : 1;                  //      Bits= 1

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  // 
  uint8_t DASH_CONTROLS_BUTTON;              //      Bits= 8

#else

  uint8_t ENABLE;                            //      Bits= 1

  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  uint8_t CLEAR_FAULTS;                      //      Bits= 1

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  // 
  uint8_t DASH_CONTROLS_BUTTON;              //      Bits= 8

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} DASH_CONTROLS_RIGHT_CMD_t;

// def @HAZARD_LIGHTS_CMD CAN Message (276  0x114)
#define HAZARD_LIGHTS_CMD_IDE (0U)
#define HAZARD_LIGHTS_CMD_DLC (8U)
#define HAZARD_LIGHTS_CMD_CANID (0x114)

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t ENABLE : 1;                        //      Bits= 1

  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  uint8_t CLEAR_FAULTS : 1;                  //      Bits= 1

  uint8_t HAZARD_LIGHTS_CMD : 1;             //      Bits= 1

#else

  uint8_t ENABLE;                            //      Bits= 1

  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  uint8_t CLEAR_FAULTS;                      //      Bits= 1

  uint8_t HAZARD_LIGHTS_CMD;                 //      Bits= 1

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} HAZARD_LIGHTS_CMD_t;

// def @HEADLIGHT_CMD CAN Message (280  0x118)
#define HEADLIGHT_CMD_IDE (0U)
#define HEADLIGHT_CMD_DLC (8U)
#define HEADLIGHT_CMD_CANID (0x118)

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t ENABLE : 1;                        //      Bits= 1

  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  uint8_t CLEAR_FAULTS : 1;                  //      Bits= 1

  //  2 : "High Beams"
  //  1 : "Low Beams"
  //  0 : "Headlights Off"
  // 
  uint8_t HEADLIGHT_CMD;                     //      Bits= 8

#else

  uint8_t ENABLE;                            //      Bits= 1

  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  uint8_t CLEAR_FAULTS;                      //      Bits= 1

  //  2 : "High Beams"
  //  1 : "Low Beams"
  //  0 : "Headlights Off"
  // 
  uint8_t HEADLIGHT_CMD;                     //      Bits= 8

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} HEADLIGHT_CMD_t;

// def @HORN_CMD CAN Message (284  0x11c)
#define HORN_CMD_IDE (0U)
#define HORN_CMD_DLC (8U)
#define HORN_CMD_CANID (0x11c)

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t ENABLE : 1;                        //      Bits= 1

  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  uint8_t CLEAR_FAULTS : 1;                  //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t HORN_CMD : 1;                      //      Bits= 1

#else

  uint8_t ENABLE;                            //      Bits= 1

  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  uint8_t CLEAR_FAULTS;                      //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t HORN_CMD;                          //      Bits= 1

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} HORN_CMD_t;

// def @MEDIA_CONTROLS_CMD CAN Message (288  0x120)
#define MEDIA_CONTROLS_CMD_IDE (0U)
#define MEDIA_CONTROLS_CMD_DLC (8U)
#define MEDIA_CONTROLS_CMD_CANID (0x120)

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t ENABLE : 1;                        //      Bits= 1

  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  uint8_t CLEAR_FAULTS : 1;                  //      Bits= 1

  //  6 : "MEDIA_CONTROL_VOL_DOWN"
  //  5 : "MEDIA_CONTROL_VOL_UP"
  //  4 : "MEDIA_CONTROL_NEXT_TRACK_HANG_UP"
  //  3 : "MEDIA_CONTROL_PREV_TRACK_ANSWER"
  //  2 : "MEDIA_CONTROL_MUTE"
  //  1 : "MEDIA_CONTROL_VOICE_COMMAND"
  //  0 : "MEDIA_CONTROL_NONE"
  // 
  uint8_t MEDIA_CONTROLS_CMD;                //      Bits= 8

#else

  uint8_t ENABLE;                            //      Bits= 1

  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  uint8_t CLEAR_FAULTS;                      //      Bits= 1

  //  6 : "MEDIA_CONTROL_VOL_DOWN"
  //  5 : "MEDIA_CONTROL_VOL_UP"
  //  4 : "MEDIA_CONTROL_NEXT_TRACK_HANG_UP"
  //  3 : "MEDIA_CONTROL_PREV_TRACK_ANSWER"
  //  2 : "MEDIA_CONTROL_MUTE"
  //  1 : "MEDIA_CONTROL_VOICE_COMMAND"
  //  0 : "MEDIA_CONTROL_NONE"
  // 
  uint8_t MEDIA_CONTROLS_CMD;                //      Bits= 8

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} MEDIA_CONTROLS_CMD_t;

// def @PARKING_BRAKE_CMD CAN Message (292  0x124)
#define PARKING_BRAKE_CMD_IDE (0U)
#define PARKING_BRAKE_CMD_DLC (8U)
#define PARKING_BRAKE_CMD_CANID (0x124)

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t ENABLE : 1;                        //      Bits= 1

  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  uint8_t CLEAR_FAULTS : 1;                  //      Bits= 1

  uint8_t PARKING_BRAKE_CMD : 1;             //      Bits= 1

#else

  uint8_t ENABLE;                            //      Bits= 1

  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  uint8_t CLEAR_FAULTS;                      //      Bits= 1

  uint8_t PARKING_BRAKE_CMD;                 //      Bits= 1

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} PARKING_BRAKE_CMD_t;

// def @SHIFT_CMD CAN Message (296  0x128)
#define SHIFT_CMD_IDE (0U)
#define SHIFT_CMD_DLC (8U)
#define SHIFT_CMD_CANID (0x128)

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t ENABLE : 1;                        //      Bits= 1

  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  uint8_t CLEAR_FAULTS : 1;                  //      Bits= 1

  // FORWARD is also HIGH on vehicles with LOW/HIGH, PARK and LOW only available on certain Vehicles.
  //  0 : "PARK"
  //  1 : "REVERSE"
  //  2 : "NEUTRAL"
  //  3 : "FORWARD/HIGH"
  //  4 : "LOW"
  //  7 : "NONE"
  // 
  uint8_t SHIFT_CMD;                         //      Bits= 8

#else

  uint8_t ENABLE;                            //      Bits= 1

  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  uint8_t CLEAR_FAULTS;                      //      Bits= 1

  // FORWARD is also HIGH on vehicles with LOW/HIGH, PARK and LOW only available on certain Vehicles.
  //  0 : "PARK"
  //  1 : "REVERSE"
  //  2 : "NEUTRAL"
  //  3 : "FORWARD/HIGH"
  //  4 : "LOW"
  //  7 : "NONE"
  // 
  uint8_t SHIFT_CMD;                         //      Bits= 8

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} SHIFT_CMD_t;

// def @STEERING_CMD CAN Message (300  0x12c)
#define STEERING_CMD_IDE (0U)
#define STEERING_CMD_DLC (8U)
#define STEERING_CMD_CANID (0x12c)
// signal: @POSITION_ro
#define PACMOD3_POSITION_ro_CovFactor (0.001000)
#define PACMOD3_POSITION_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD3_POSITION_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @ROTATION_RATE_ro
#define PACMOD3_ROTATION_RATE_ro_CovFactor (0.001000)
#define PACMOD3_ROTATION_RATE_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD3_ROTATION_RATE_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t ENABLE : 1;                        //      Bits= 1

  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  uint8_t CLEAR_FAULTS : 1;                  //      Bits= 1

  int16_t POSITION_ro;                       //  [-] Bits=16 Factor= 0.001000        Unit:'rad'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t POSITION_phys;
#endif // PACMOD3_USE_SIGFLOAT

  uint16_t ROTATION_RATE_ro;                 //      Bits=16 Factor= 0.001000        Unit:'rad/s'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t ROTATION_RATE_phys;
#endif // PACMOD3_USE_SIGFLOAT

#else

  uint8_t ENABLE;                            //      Bits= 1

  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  uint8_t CLEAR_FAULTS;                      //      Bits= 1

  int16_t POSITION_ro;                       //  [-] Bits=16 Factor= 0.001000        Unit:'rad'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t POSITION_phys;
#endif // PACMOD3_USE_SIGFLOAT

  uint16_t ROTATION_RATE_ro;                 //      Bits=16 Factor= 0.001000        Unit:'rad/s'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t ROTATION_RATE_phys;
#endif // PACMOD3_USE_SIGFLOAT

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} STEERING_CMD_t;

// def @TURN_CMD CAN Message (304  0x130)
#define TURN_CMD_IDE (0U)
#define TURN_CMD_DLC (8U)
#define TURN_CMD_CANID (0x130)

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t ENABLE : 1;                        //      Bits= 1

  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  uint8_t CLEAR_FAULTS : 1;                  //      Bits= 1

  //  0 : "RIGHT"
  //  1 : "NONE"
  //  2 : "LEFT"
  //  3 : "HAZARD"
  // 
  uint8_t TURN_SIGNAL_CMD;                   //      Bits= 8

#else

  uint8_t ENABLE;                            //      Bits= 1

  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  uint8_t CLEAR_FAULTS;                      //      Bits= 1

  //  0 : "RIGHT"
  //  1 : "NONE"
  //  2 : "LEFT"
  //  3 : "HAZARD"
  // 
  uint8_t TURN_SIGNAL_CMD;                   //      Bits= 8

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} TURN_CMD_t;

// def @WIPER_CMD CAN Message (308  0x134)
#define WIPER_CMD_IDE (0U)
#define WIPER_CMD_DLC (8U)
#define WIPER_CMD_CANID (0x134)

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t ENABLE : 1;                        //      Bits= 1

  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  uint8_t CLEAR_FAULTS : 1;                  //      Bits= 1

  //  7 : "High"
  //  6 : "Low"
  //  5 : "Intermittent 5"
  //  4 : "Intermittent 4"
  //  3 : "Intermittent 3"
  //  2 : "Intermittent 2"
  //  1 : "Intermittent 1"
  //  0 : "Wipers Off"
  // 
  uint8_t WIPER_CMD;                         //      Bits= 8

#else

  uint8_t ENABLE;                            //      Bits= 1

  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  uint8_t CLEAR_FAULTS;                      //      Bits= 1

  //  7 : "High"
  //  6 : "Low"
  //  5 : "Intermittent 5"
  //  4 : "Intermittent 4"
  //  3 : "Intermittent 3"
  //  2 : "Intermittent 2"
  //  1 : "Intermittent 1"
  //  0 : "Wipers Off"
  // 
  uint8_t WIPER_CMD;                         //      Bits= 8

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} WIPER_CMD_t;

// def @ACCEL_RPT CAN Message (512  0x200)
#define ACCEL_RPT_IDE (0U)
#define ACCEL_RPT_DLC (8U)
#define ACCEL_RPT_CANID (0x200)
// signal: @MANUAL_INPUT_ro
#define PACMOD3_MANUAL_INPUT_ro_CovFactor (0.001000)
#define PACMOD3_MANUAL_INPUT_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD3_MANUAL_INPUT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @COMMANDED_VALUE_ro
#define PACMOD3_COMMANDED_VALUE_ro_CovFactor (0.001000)
#define PACMOD3_COMMANDED_VALUE_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD3_COMMANDED_VALUE_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @OUTPUT_VALUE_ro
#define PACMOD3_OUTPUT_VALUE_ro_CovFactor (0.001000)
#define PACMOD3_OUTPUT_VALUE_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD3_OUTPUT_VALUE_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t ENABLED : 1;                       //      Bits= 1

  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  uint16_t MANUAL_INPUT_ro;                  //      Bits=16 Factor= 0.001000       

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t MANUAL_INPUT_phys;
#endif // PACMOD3_USE_SIGFLOAT

  uint16_t COMMANDED_VALUE_ro;               //      Bits=16 Factor= 0.001000       

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t COMMANDED_VALUE_phys;
#endif // PACMOD3_USE_SIGFLOAT

  uint16_t OUTPUT_VALUE_ro;                  //      Bits=16 Factor= 0.001000       

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t OUTPUT_VALUE_phys;
#endif // PACMOD3_USE_SIGFLOAT

#else

  uint8_t ENABLED;                           //      Bits= 1

  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  uint8_t PACMOD_FAULT;                      //      Bits= 1

  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  uint16_t MANUAL_INPUT_ro;                  //      Bits=16 Factor= 0.001000       

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t MANUAL_INPUT_phys;
#endif // PACMOD3_USE_SIGFLOAT

  uint16_t COMMANDED_VALUE_ro;               //      Bits=16 Factor= 0.001000       

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t COMMANDED_VALUE_phys;
#endif // PACMOD3_USE_SIGFLOAT

  uint16_t OUTPUT_VALUE_ro;                  //      Bits=16 Factor= 0.001000       

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t OUTPUT_VALUE_phys;
#endif // PACMOD3_USE_SIGFLOAT

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} ACCEL_RPT_t;

// def @BRAKE_RPT CAN Message (516  0x204)
#define BRAKE_RPT_IDE (0U)
#define BRAKE_RPT_DLC (8U)
#define BRAKE_RPT_CANID (0x204)
// signal: @MANUAL_INPUT_ro
#define PACMOD3_MANUAL_INPUT_ro_CovFactor (0.001000)
#define PACMOD3_MANUAL_INPUT_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD3_MANUAL_INPUT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @COMMANDED_VALUE_ro
#define PACMOD3_COMMANDED_VALUE_ro_CovFactor (0.001000)
#define PACMOD3_COMMANDED_VALUE_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD3_COMMANDED_VALUE_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @OUTPUT_VALUE_ro
#define PACMOD3_OUTPUT_VALUE_ro_CovFactor (0.001000)
#define PACMOD3_OUTPUT_VALUE_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD3_OUTPUT_VALUE_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t ENABLED : 1;                       //      Bits= 1

  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  uint16_t MANUAL_INPUT_ro;                  //      Bits=16 Factor= 0.001000       

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t MANUAL_INPUT_phys;
#endif // PACMOD3_USE_SIGFLOAT

  uint16_t COMMANDED_VALUE_ro;               //      Bits=16 Factor= 0.001000       

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t COMMANDED_VALUE_phys;
#endif // PACMOD3_USE_SIGFLOAT

  uint16_t OUTPUT_VALUE_ro;                  //      Bits=16 Factor= 0.001000       

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t OUTPUT_VALUE_phys;
#endif // PACMOD3_USE_SIGFLOAT

#else

  uint8_t ENABLED;                           //      Bits= 1

  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  uint8_t PACMOD_FAULT;                      //      Bits= 1

  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  uint16_t MANUAL_INPUT_ro;                  //      Bits=16 Factor= 0.001000       

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t MANUAL_INPUT_phys;
#endif // PACMOD3_USE_SIGFLOAT

  uint16_t COMMANDED_VALUE_ro;               //      Bits=16 Factor= 0.001000       

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t COMMANDED_VALUE_phys;
#endif // PACMOD3_USE_SIGFLOAT

  uint16_t OUTPUT_VALUE_ro;                  //      Bits=16 Factor= 0.001000       

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t OUTPUT_VALUE_phys;
#endif // PACMOD3_USE_SIGFLOAT

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} BRAKE_RPT_t;

// def @CRUISE_CONTROL_BUTTONS_RPT CAN Message (520  0x208)
#define CRUISE_CONTROL_BUTTONS_RPT_IDE (0U)
#define CRUISE_CONTROL_BUTTONS_RPT_DLC (8U)
#define CRUISE_CONTROL_BUTTONS_RPT_CANID (0x208)

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t ENABLED : 1;                       //      Bits= 1

  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  6 : "CRUISE_CONTROL_ON_OFF"
  //  5 : "CRUISE_CONTROL_RES_INC"
  //  4 : "CRUISE_CONTROL_SET_DEC"
  //  3 : "CRUISE_CONTROL_ACC_CLOSER"
  //  2 : "CRUISE_CONTROL_ACC_FURTHER"
  //  1 : "CRUISE_CONTROL_CNCL"
  //  0 : "CRUISE_CONTROL_NONE"
  // 
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  6 : "CRUISE_CONTROL_ON_OFF"
  //  5 : "CRUISE_CONTROL_RES_INC"
  //  4 : "CRUISE_CONTROL_SET_DEC"
  //  3 : "CRUISE_CONTROL_ACC_CLOSER"
  //  2 : "CRUISE_CONTROL_ACC_FURTHER"
  //  1 : "CRUISE_CONTROL_CNCL"
  //  0 : "CRUISE_CONTROL_NONE"
  // 
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  6 : "CRUISE_CONTROL_ON_OFF"
  //  5 : "CRUISE_CONTROL_RES_INC"
  //  4 : "CRUISE_CONTROL_SET_DEC"
  //  3 : "CRUISE_CONTROL_ACC_CLOSER"
  //  2 : "CRUISE_CONTROL_ACC_FURTHER"
  //  1 : "CRUISE_CONTROL_CNCL"
  //  0 : "CRUISE_CONTROL_NONE"
  // 
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#else

  uint8_t ENABLED;                           //      Bits= 1

  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  uint8_t PACMOD_FAULT;                      //      Bits= 1

  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  6 : "CRUISE_CONTROL_ON_OFF"
  //  5 : "CRUISE_CONTROL_RES_INC"
  //  4 : "CRUISE_CONTROL_SET_DEC"
  //  3 : "CRUISE_CONTROL_ACC_CLOSER"
  //  2 : "CRUISE_CONTROL_ACC_FURTHER"
  //  1 : "CRUISE_CONTROL_CNCL"
  //  0 : "CRUISE_CONTROL_NONE"
  // 
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  6 : "CRUISE_CONTROL_ON_OFF"
  //  5 : "CRUISE_CONTROL_RES_INC"
  //  4 : "CRUISE_CONTROL_SET_DEC"
  //  3 : "CRUISE_CONTROL_ACC_CLOSER"
  //  2 : "CRUISE_CONTROL_ACC_FURTHER"
  //  1 : "CRUISE_CONTROL_CNCL"
  //  0 : "CRUISE_CONTROL_NONE"
  // 
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  6 : "CRUISE_CONTROL_ON_OFF"
  //  5 : "CRUISE_CONTROL_RES_INC"
  //  4 : "CRUISE_CONTROL_SET_DEC"
  //  3 : "CRUISE_CONTROL_ACC_CLOSER"
  //  2 : "CRUISE_CONTROL_ACC_FURTHER"
  //  1 : "CRUISE_CONTROL_CNCL"
  //  0 : "CRUISE_CONTROL_NONE"
  // 
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} CRUISE_CONTROL_BUTTONS_RPT_t;

// def @DASH_CONTROLS_LEFT_RPT CAN Message (524  0x20c)
#define DASH_CONTROLS_LEFT_RPT_IDE (0U)
#define DASH_CONTROLS_LEFT_RPT_DLC (8U)
#define DASH_CONTROLS_LEFT_RPT_CANID (0x20c)

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t ENABLED : 1;                       //      Bits= 1

  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  // 
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  // 
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  // 
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#else

  uint8_t ENABLED;                           //      Bits= 1

  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  uint8_t PACMOD_FAULT;                      //      Bits= 1

  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  // 
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  // 
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  // 
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} DASH_CONTROLS_LEFT_RPT_t;

// def @DASH_CONTROLS_RIGHT_RPT CAN Message (528  0x210)
#define DASH_CONTROLS_RIGHT_RPT_IDE (0U)
#define DASH_CONTROLS_RIGHT_RPT_DLC (8U)
#define DASH_CONTROLS_RIGHT_RPT_CANID (0x210)

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t ENABLED : 1;                       //      Bits= 1

  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  // 
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  // 
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  // 
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#else

  uint8_t ENABLED;                           //      Bits= 1

  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  uint8_t PACMOD_FAULT;                      //      Bits= 1

  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  // 
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  // 
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  // 
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} DASH_CONTROLS_RIGHT_RPT_t;

// def @HAZARD_LIGHTS_RPT CAN Message (532  0x214)
#define HAZARD_LIGHTS_RPT_IDE (0U)
#define HAZARD_LIGHTS_RPT_DLC (8U)
#define HAZARD_LIGHTS_RPT_CANID (0x214)

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t ENABLED : 1;                       //      Bits= 1

  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  uint8_t MANUAL_INPUT : 1;                  //      Bits= 1

  uint8_t COMMANDED_VALUE : 1;               //      Bits= 1

  uint8_t OUTPUT_VALUE : 1;                  //      Bits= 1

#else

  uint8_t ENABLED;                           //      Bits= 1

  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  uint8_t PACMOD_FAULT;                      //      Bits= 1

  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  uint8_t MANUAL_INPUT;                      //      Bits= 1

  uint8_t COMMANDED_VALUE;                   //      Bits= 1

  uint8_t OUTPUT_VALUE;                      //      Bits= 1

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} HAZARD_LIGHTS_RPT_t;

// def @HEADLIGHT_RPT CAN Message (536  0x218)
#define HEADLIGHT_RPT_IDE (0U)
#define HEADLIGHT_RPT_DLC (8U)
#define HEADLIGHT_RPT_CANID (0x218)

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t ENABLED : 1;                       //      Bits= 1

  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  2 : "High Beams"
  //  1 : "Low Beams"
  //  0 : "Headlights Off"
  // 
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  2 : "High Beams"
  //  1 : "Low Beams"
  //  0 : "Headlights Off"
  // 
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  2 : "High Beams"
  //  1 : "Low Beams"
  //  0 : "Headlights Off"
  // 
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#else

  uint8_t ENABLED;                           //      Bits= 1

  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  uint8_t PACMOD_FAULT;                      //      Bits= 1

  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  2 : "High Beams"
  //  1 : "Low Beams"
  //  0 : "Headlights Off"
  // 
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  2 : "High Beams"
  //  1 : "Low Beams"
  //  0 : "Headlights Off"
  // 
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  2 : "High Beams"
  //  1 : "Low Beams"
  //  0 : "Headlights Off"
  // 
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} HEADLIGHT_RPT_t;

// def @HORN_RPT CAN Message (540  0x21c)
#define HORN_RPT_IDE (0U)
#define HORN_RPT_DLC (8U)
#define HORN_RPT_CANID (0x21c)

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t ENABLED : 1;                       //      Bits= 1

  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#else

  uint8_t ENABLED;                           //      Bits= 1

  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  uint8_t PACMOD_FAULT;                      //      Bits= 1

  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} HORN_RPT_t;

// def @MEDIA_CONTROLS_RPT CAN Message (544  0x220)
#define MEDIA_CONTROLS_RPT_IDE (0U)
#define MEDIA_CONTROLS_RPT_DLC (8U)
#define MEDIA_CONTROLS_RPT_CANID (0x220)

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t ENABLED : 1;                       //      Bits= 1

  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  6 : "MEDIA_CONTROL_VOL_DOWN"
  //  5 : "MEDIA_CONTROL_VOL_UP"
  //  4 : "MEDIA_CONTROL_NEXT_TRACK_HANG_UP"
  //  3 : "MEDIA_CONTROL_PREV_TRACK_ANSWER"
  //  2 : "MEDIA_CONTROL_MUTE"
  //  1 : "MEDIA_CONTROL_VOICE_COMMAND"
  //  0 : "MEDIA_CONTROL_NONE"
  // 
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  6 : "MEDIA_CONTROL_VOL_DOWN"
  //  5 : "MEDIA_CONTROL_VOL_UP"
  //  4 : "MEDIA_CONTROL_NEXT_TRACK_HANG_UP"
  //  3 : "MEDIA_CONTROL_PREV_TRACK_ANSWER"
  //  2 : "MEDIA_CONTROL_MUTE"
  //  1 : "MEDIA_CONTROL_VOICE_COMMAND"
  //  0 : "MEDIA_CONTROL_NONE"
  // 
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  6 : "MEDIA_CONTROL_VOL_DOWN"
  //  5 : "MEDIA_CONTROL_VOL_UP"
  //  4 : "MEDIA_CONTROL_NEXT_TRACK_HANG_UP"
  //  3 : "MEDIA_CONTROL_PREV_TRACK_ANSWER"
  //  2 : "MEDIA_CONTROL_MUTE"
  //  1 : "MEDIA_CONTROL_VOICE_COMMAND"
  //  0 : "MEDIA_CONTROL_NONE"
  // 
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#else

  uint8_t ENABLED;                           //      Bits= 1

  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  uint8_t PACMOD_FAULT;                      //      Bits= 1

  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  6 : "MEDIA_CONTROL_VOL_DOWN"
  //  5 : "MEDIA_CONTROL_VOL_UP"
  //  4 : "MEDIA_CONTROL_NEXT_TRACK_HANG_UP"
  //  3 : "MEDIA_CONTROL_PREV_TRACK_ANSWER"
  //  2 : "MEDIA_CONTROL_MUTE"
  //  1 : "MEDIA_CONTROL_VOICE_COMMAND"
  //  0 : "MEDIA_CONTROL_NONE"
  // 
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  6 : "MEDIA_CONTROL_VOL_DOWN"
  //  5 : "MEDIA_CONTROL_VOL_UP"
  //  4 : "MEDIA_CONTROL_NEXT_TRACK_HANG_UP"
  //  3 : "MEDIA_CONTROL_PREV_TRACK_ANSWER"
  //  2 : "MEDIA_CONTROL_MUTE"
  //  1 : "MEDIA_CONTROL_VOICE_COMMAND"
  //  0 : "MEDIA_CONTROL_NONE"
  // 
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  6 : "MEDIA_CONTROL_VOL_DOWN"
  //  5 : "MEDIA_CONTROL_VOL_UP"
  //  4 : "MEDIA_CONTROL_NEXT_TRACK_HANG_UP"
  //  3 : "MEDIA_CONTROL_PREV_TRACK_ANSWER"
  //  2 : "MEDIA_CONTROL_MUTE"
  //  1 : "MEDIA_CONTROL_VOICE_COMMAND"
  //  0 : "MEDIA_CONTROL_NONE"
  // 
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} MEDIA_CONTROLS_RPT_t;

// def @PARKING_BRAKE_RPT CAN Message (548  0x224)
#define PARKING_BRAKE_RPT_IDE (0U)
#define PARKING_BRAKE_RPT_DLC (8U)
#define PARKING_BRAKE_RPT_CANID (0x224)

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t ENABLED : 1;                       //      Bits= 1

  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  uint8_t MANUAL_INPUT : 1;                  //      Bits= 1

  uint8_t COMMANDED_VALUE : 1;               //      Bits= 1

  uint8_t OUTPUT_VALUE : 1;                  //      Bits= 1

#else

  uint8_t ENABLED;                           //      Bits= 1

  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  uint8_t PACMOD_FAULT;                      //      Bits= 1

  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  uint8_t MANUAL_INPUT;                      //      Bits= 1

  uint8_t COMMANDED_VALUE;                   //      Bits= 1

  uint8_t OUTPUT_VALUE;                      //      Bits= 1

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} PARKING_BRAKE_RPT_t;

// def @SHIFT_RPT CAN Message (552  0x228)
#define SHIFT_RPT_IDE (0U)
#define SHIFT_RPT_DLC (8U)
#define SHIFT_RPT_CANID (0x228)

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t ENABLED : 1;                       //      Bits= 1

  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "PARK"
  //  1 : "REVERSE"
  //  2 : "NEUTRAL"
  //  3 : "FORWARD/HIGH"
  //  4 : "LOW"
  //  5 : "BETWEEN_GEARS"
  //  6 : "ERROR"
  //  7 : "NONE"
  // 
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  0 : "PARK"
  //  1 : "REVERSE"
  //  2 : "NEUTRAL"
  //  3 : "FORWARD/HIGH"
  //  4 : "LOW"
  //  7 : "NONE"
  // 
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  0 : "PARK"
  //  1 : "REVERSE"
  //  2 : "NEUTRAL"
  //  3 : "FORWARD/HIGH"
  //  4 : "LOW"
  //  5 : "BETWEEN_GEARS"
  //  6 : "ERROR"
  //  7 : "NONE"
  // 
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#else

  uint8_t ENABLED;                           //      Bits= 1

  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  uint8_t PACMOD_FAULT;                      //      Bits= 1

  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "PARK"
  //  1 : "REVERSE"
  //  2 : "NEUTRAL"
  //  3 : "FORWARD/HIGH"
  //  4 : "LOW"
  //  5 : "BETWEEN_GEARS"
  //  6 : "ERROR"
  //  7 : "NONE"
  // 
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  0 : "PARK"
  //  1 : "REVERSE"
  //  2 : "NEUTRAL"
  //  3 : "FORWARD/HIGH"
  //  4 : "LOW"
  //  7 : "NONE"
  // 
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  0 : "PARK"
  //  1 : "REVERSE"
  //  2 : "NEUTRAL"
  //  3 : "FORWARD/HIGH"
  //  4 : "LOW"
  //  5 : "BETWEEN_GEARS"
  //  6 : "ERROR"
  //  7 : "NONE"
  // 
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} SHIFT_RPT_t;

// def @STEERING_RPT CAN Message (556  0x22c)
#define STEERING_RPT_IDE (0U)
#define STEERING_RPT_DLC (8U)
#define STEERING_RPT_CANID (0x22c)
// signal: @MANUAL_INPUT_ro
#define PACMOD3_MANUAL_INPUT_ro_CovFactor (0.001000)
#define PACMOD3_MANUAL_INPUT_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD3_MANUAL_INPUT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @COMMANDED_VALUE_ro
#define PACMOD3_COMMANDED_VALUE_ro_CovFactor (0.001000)
#define PACMOD3_COMMANDED_VALUE_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD3_COMMANDED_VALUE_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @OUTPUT_VALUE_ro
#define PACMOD3_OUTPUT_VALUE_ro_CovFactor (0.001000)
#define PACMOD3_OUTPUT_VALUE_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD3_OUTPUT_VALUE_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t ENABLED : 1;                       //      Bits= 1

  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  int16_t MANUAL_INPUT_ro;                   //  [-] Bits=16 Factor= 0.001000        Unit:'rad'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t MANUAL_INPUT_phys;
#endif // PACMOD3_USE_SIGFLOAT

  int16_t COMMANDED_VALUE_ro;                //  [-] Bits=16 Factor= 0.001000        Unit:'rad'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t COMMANDED_VALUE_phys;
#endif // PACMOD3_USE_SIGFLOAT

  int16_t OUTPUT_VALUE_ro;                   //  [-] Bits=16 Factor= 0.001000        Unit:'rad'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t OUTPUT_VALUE_phys;
#endif // PACMOD3_USE_SIGFLOAT

#else

  uint8_t ENABLED;                           //      Bits= 1

  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  uint8_t PACMOD_FAULT;                      //      Bits= 1

  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  int16_t MANUAL_INPUT_ro;                   //  [-] Bits=16 Factor= 0.001000        Unit:'rad'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t MANUAL_INPUT_phys;
#endif // PACMOD3_USE_SIGFLOAT

  int16_t COMMANDED_VALUE_ro;                //  [-] Bits=16 Factor= 0.001000        Unit:'rad'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t COMMANDED_VALUE_phys;
#endif // PACMOD3_USE_SIGFLOAT

  int16_t OUTPUT_VALUE_ro;                   //  [-] Bits=16 Factor= 0.001000        Unit:'rad'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t OUTPUT_VALUE_phys;
#endif // PACMOD3_USE_SIGFLOAT

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} STEERING_RPT_t;

// def @TURN_RPT CAN Message (560  0x230)
#define TURN_RPT_IDE (0U)
#define TURN_RPT_DLC (8U)
#define TURN_RPT_CANID (0x230)

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t ENABLED : 1;                       //      Bits= 1

  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "RIGHT"
  //  1 : "NONE"
  //  2 : "LEFT"
  //  3 : "HAZARD"
  // 
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  0 : "RIGHT"
  //  1 : "NONE"
  //  2 : "LEFT"
  //  3 : "HAZARD"
  // 
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  0 : "RIGHT"
  //  1 : "NONE"
  //  2 : "LEFT"
  //  3 : "HAZARD"
  // 
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#else

  uint8_t ENABLED;                           //      Bits= 1

  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  uint8_t PACMOD_FAULT;                      //      Bits= 1

  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "RIGHT"
  //  1 : "NONE"
  //  2 : "LEFT"
  //  3 : "HAZARD"
  // 
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  0 : "RIGHT"
  //  1 : "NONE"
  //  2 : "LEFT"
  //  3 : "HAZARD"
  // 
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  0 : "RIGHT"
  //  1 : "NONE"
  //  2 : "LEFT"
  //  3 : "HAZARD"
  // 
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} TURN_RPT_t;

// def @WIPER_RPT CAN Message (564  0x234)
#define WIPER_RPT_IDE (0U)
#define WIPER_RPT_DLC (8U)
#define WIPER_RPT_CANID (0x234)

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t ENABLED : 1;                       //      Bits= 1

  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  7 : "High"
  //  6 : "Low"
  //  5 : "Intermittent 5"
  //  4 : "Intermittent 4"
  //  3 : "Intermittent 3"
  //  2 : "Intermittent 2"
  //  1 : "Intermittent 1"
  //  0 : "Wipers Off"
  // 
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  7 : "High"
  //  6 : "Low"
  //  5 : "Intermittent 5"
  //  4 : "Intermittent 4"
  //  3 : "Intermittent 3"
  //  2 : "Intermittent 2"
  //  1 : "Intermittent 1"
  //  0 : "Wipers Off"
  // 
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  7 : "High"
  //  6 : "Low"
  //  5 : "Intermittent 5"
  //  4 : "Intermittent 4"
  //  3 : "Intermittent 3"
  //  2 : "Intermittent 2"
  //  1 : "Intermittent 1"
  //  0 : "Wipers Off"
  // 
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#else

  uint8_t ENABLED;                           //      Bits= 1

  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  uint8_t PACMOD_FAULT;                      //      Bits= 1

  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  7 : "High"
  //  6 : "Low"
  //  5 : "Intermittent 5"
  //  4 : "Intermittent 4"
  //  3 : "Intermittent 3"
  //  2 : "Intermittent 2"
  //  1 : "Intermittent 1"
  //  0 : "Wipers Off"
  // 
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  7 : "High"
  //  6 : "Low"
  //  5 : "Intermittent 5"
  //  4 : "Intermittent 4"
  //  3 : "Intermittent 3"
  //  2 : "Intermittent 2"
  //  1 : "Intermittent 1"
  //  0 : "Wipers Off"
  // 
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  7 : "High"
  //  6 : "Low"
  //  5 : "Intermittent 5"
  //  4 : "Intermittent 4"
  //  3 : "Intermittent 3"
  //  2 : "Intermittent 2"
  //  1 : "Intermittent 1"
  //  0 : "Wipers Off"
  // 
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} WIPER_RPT_t;

// def @ACCEL_AUX_RPT CAN Message (768  0x300)
#define ACCEL_AUX_RPT_IDE (0U)
#define ACCEL_AUX_RPT_DLC (8U)
#define ACCEL_AUX_RPT_CANID (0x300)
// signal: @RAW_PEDAL_POS_ro
#define PACMOD3_RAW_PEDAL_POS_ro_CovFactor (0.001000)
#define PACMOD3_RAW_PEDAL_POS_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD3_RAW_PEDAL_POS_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @RAW_PEDAL_FORCE_ro
#define PACMOD3_RAW_PEDAL_FORCE_ro_CovFactor (0.001000)
#define PACMOD3_RAW_PEDAL_FORCE_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD3_RAW_PEDAL_FORCE_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  int16_t RAW_PEDAL_POS_ro;                  //  [-] Bits=16 Factor= 0.001000       

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t RAW_PEDAL_POS_phys;
#endif // PACMOD3_USE_SIGFLOAT

  int16_t RAW_PEDAL_FORCE_ro;                //  [-] Bits=16 Factor= 0.001000       

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t RAW_PEDAL_FORCE_phys;
#endif // PACMOD3_USE_SIGFLOAT

  uint8_t USER_INTERACTION : 1;              //      Bits= 1

  uint8_t RAW_PEDAL_POS_IS_VALID : 1;        //      Bits= 1

  uint8_t RAW_PEDAL_FORCE_IS_VALID : 1;      //      Bits= 1

  uint8_t USER_INTERACTION_IS_VALID : 1;     //      Bits= 1

#else

  int16_t RAW_PEDAL_POS_ro;                  //  [-] Bits=16 Factor= 0.001000       

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t RAW_PEDAL_POS_phys;
#endif // PACMOD3_USE_SIGFLOAT

  int16_t RAW_PEDAL_FORCE_ro;                //  [-] Bits=16 Factor= 0.001000       

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t RAW_PEDAL_FORCE_phys;
#endif // PACMOD3_USE_SIGFLOAT

  uint8_t USER_INTERACTION;                  //      Bits= 1

  uint8_t RAW_PEDAL_POS_IS_VALID;            //      Bits= 1

  uint8_t RAW_PEDAL_FORCE_IS_VALID;          //      Bits= 1

  uint8_t USER_INTERACTION_IS_VALID;         //      Bits= 1

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} ACCEL_AUX_RPT_t;

// def @BRAKE_AUX_RPT CAN Message (772  0x304)
#define BRAKE_AUX_RPT_IDE (0U)
#define BRAKE_AUX_RPT_DLC (8U)
#define BRAKE_AUX_RPT_CANID (0x304)

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  int16_t RAW_PEDAL_POS;                     //  [-] Bits=16

  int16_t RAW_PEDAL_FORCE;                   //  [-] Bits=16

  int16_t RAW_BRAKE_PRESSURE;                //  [-] Bits=16

  uint8_t USER_INTERACTION : 1;              //      Bits= 1

  uint8_t BRAKE_ON_OFF : 1;                  //      Bits= 1

  uint8_t RAW_PEDAL_POS_IS_VALID : 1;        //      Bits= 1

  uint8_t RAW_PEDAL_FORCE_IS_VALID : 1;      //      Bits= 1

  uint8_t RAW_BRAKE_PRESSURE_IS_VALID : 1;   //      Bits= 1

  uint8_t USER_INTERACTION_IS_VALID : 1;     //      Bits= 1

  uint8_t BRAKE_ON_OFF_IS_VALID : 1;         //      Bits= 1

#else

  int16_t RAW_PEDAL_POS;                     //  [-] Bits=16

  int16_t RAW_PEDAL_FORCE;                   //  [-] Bits=16

  int16_t RAW_BRAKE_PRESSURE;                //  [-] Bits=16

  uint8_t USER_INTERACTION;                  //      Bits= 1

  uint8_t BRAKE_ON_OFF;                      //      Bits= 1

  uint8_t RAW_PEDAL_POS_IS_VALID;            //      Bits= 1

  uint8_t RAW_PEDAL_FORCE_IS_VALID;          //      Bits= 1

  uint8_t RAW_BRAKE_PRESSURE_IS_VALID;       //      Bits= 1

  uint8_t USER_INTERACTION_IS_VALID;         //      Bits= 1

  uint8_t BRAKE_ON_OFF_IS_VALID;             //      Bits= 1

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} BRAKE_AUX_RPT_t;

// def @HEADLIGHT_AUX_RPT CAN Message (792  0x318)
#define HEADLIGHT_AUX_RPT_IDE (0U)
#define HEADLIGHT_AUX_RPT_DLC (8U)
#define HEADLIGHT_AUX_RPT_CANID (0x318)

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t HEADLIGHTS_ON : 1;                   //      Bits= 1

  uint8_t HEADLIGHTS_ON_BRIGHT : 1;            //      Bits= 1

  uint8_t FOG_LIGHTS_ON : 1;                   //      Bits= 1

  //  3 : "Headlights On - Auto Mode"
  //  2 : "Headlights On - Manual Mode"
  //  1 : "Parking Lights Only"
  //  0 : "Headlights Off"
  // 
  uint8_t HEADLIGHTS_MODE;                     //      Bits= 8

  uint8_t HEADLIGHTS_ON_IS_VALID : 1;          //      Bits= 1

  uint8_t HEADLIGHTS_ON_BRIGHT_IS_VALID : 1;   //      Bits= 1

  uint8_t FOG_LIGHTS_ON_IS_VALID : 1;          //      Bits= 1

  uint8_t HEADLIGHTS_MODE_IS_VALID : 1;        //      Bits= 1

#else

  uint8_t HEADLIGHTS_ON;                       //      Bits= 1

  uint8_t HEADLIGHTS_ON_BRIGHT;                //      Bits= 1

  uint8_t FOG_LIGHTS_ON;                       //      Bits= 1

  //  3 : "Headlights On - Auto Mode"
  //  2 : "Headlights On - Manual Mode"
  //  1 : "Parking Lights Only"
  //  0 : "Headlights Off"
  // 
  uint8_t HEADLIGHTS_MODE;                     //      Bits= 8

  uint8_t HEADLIGHTS_ON_IS_VALID;              //      Bits= 1

  uint8_t HEADLIGHTS_ON_BRIGHT_IS_VALID;       //      Bits= 1

  uint8_t FOG_LIGHTS_ON_IS_VALID;              //      Bits= 1

  uint8_t HEADLIGHTS_MODE_IS_VALID;            //      Bits= 1

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} HEADLIGHT_AUX_RPT_t;

// def @SHIFT_AUX_RPT CAN Message (808  0x328)
#define SHIFT_AUX_RPT_IDE (0U)
#define SHIFT_AUX_RPT_DLC (8U)
#define SHIFT_AUX_RPT_CANID (0x328)

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t BETWEEN_GEARS : 1;                     //      Bits= 1

  uint8_t STAY_IN_NEUTRAL_MODE : 1;              //      Bits= 1

  uint8_t BRAKE_INTERLOCK_ACTIVE : 1;            //      Bits= 1

  uint8_t SPEED_INTERLOCK_ACTIVE : 1;            //      Bits= 1

  uint8_t BETWEEN_GEARS_IS_VALID : 1;            //      Bits= 1

  uint8_t STAY_IN_NEUTRAL_MODE_IS_VALID : 1;     //      Bits= 1

  uint8_t BRAKE_INTERLOCK_ACTIVE_IS_VALID : 1;   //      Bits= 1

  uint8_t SPEED_INTERLOCK_ACTIVE_IS_VALID : 1;   //      Bits= 1

#else

  uint8_t BETWEEN_GEARS;                         //      Bits= 1

  uint8_t STAY_IN_NEUTRAL_MODE;                  //      Bits= 1

  uint8_t BRAKE_INTERLOCK_ACTIVE;                //      Bits= 1

  uint8_t SPEED_INTERLOCK_ACTIVE;                //      Bits= 1

  uint8_t BETWEEN_GEARS_IS_VALID;                //      Bits= 1

  uint8_t STAY_IN_NEUTRAL_MODE_IS_VALID;         //      Bits= 1

  uint8_t BRAKE_INTERLOCK_ACTIVE_IS_VALID;       //      Bits= 1

  uint8_t SPEED_INTERLOCK_ACTIVE_IS_VALID;       //      Bits= 1

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} SHIFT_AUX_RPT_t;

// def @STEERING_AUX_RPT CAN Message (812  0x32c)
#define STEERING_AUX_RPT_IDE (0U)
#define STEERING_AUX_RPT_DLC (8U)
#define STEERING_AUX_RPT_CANID (0x32c)
// signal: @RAW_POSITION_ro
#define PACMOD3_RAW_POSITION_ro_CovFactor (0.001000)
#define PACMOD3_RAW_POSITION_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD3_RAW_POSITION_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @RAW_TORQUE_ro
#define PACMOD3_RAW_TORQUE_ro_CovFactor (0.001000)
#define PACMOD3_RAW_TORQUE_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD3_RAW_TORQUE_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @ROTATION_RATE_ro
#define PACMOD3_ROTATION_RATE_ro_CovFactor (0.001000)
#define PACMOD3_ROTATION_RATE_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD3_ROTATION_RATE_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  int16_t RAW_POSITION_ro;                   //  [-] Bits=16 Factor= 0.001000       

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t RAW_POSITION_phys;
#endif // PACMOD3_USE_SIGFLOAT

  int16_t RAW_TORQUE_ro;                     //  [-] Bits=16 Factor= 0.001000       

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t RAW_TORQUE_phys;
#endif // PACMOD3_USE_SIGFLOAT

  uint16_t ROTATION_RATE_ro;                 //      Bits=16 Factor= 0.001000        Unit:'rad/s'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t ROTATION_RATE_phys;
#endif // PACMOD3_USE_SIGFLOAT

  uint8_t USER_INTERACTION : 1;              //      Bits= 1

  uint8_t RAW_POSITION_IS_VALID : 1;         //      Bits= 1

  uint8_t RAW_TORQUE_IS_VALID : 1;           //      Bits= 1

  uint8_t ROTATION_RATE_IS_VALID : 1;        //      Bits= 1

  uint8_t USER_INTERACTION_IS_VALID : 1;     //      Bits= 1

#else

  int16_t RAW_POSITION_ro;                   //  [-] Bits=16 Factor= 0.001000       

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t RAW_POSITION_phys;
#endif // PACMOD3_USE_SIGFLOAT

  int16_t RAW_TORQUE_ro;                     //  [-] Bits=16 Factor= 0.001000       

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t RAW_TORQUE_phys;
#endif // PACMOD3_USE_SIGFLOAT

  uint16_t ROTATION_RATE_ro;                 //      Bits=16 Factor= 0.001000        Unit:'rad/s'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t ROTATION_RATE_phys;
#endif // PACMOD3_USE_SIGFLOAT

  uint8_t USER_INTERACTION;                  //      Bits= 1

  uint8_t RAW_POSITION_IS_VALID;             //      Bits= 1

  uint8_t RAW_TORQUE_IS_VALID;               //      Bits= 1

  uint8_t ROTATION_RATE_IS_VALID;            //      Bits= 1

  uint8_t USER_INTERACTION_IS_VALID;         //      Bits= 1

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} STEERING_AUX_RPT_t;

// def @TURN_AUX_RPT CAN Message (816  0x330)
#define TURN_AUX_RPT_IDE (0U)
#define TURN_AUX_RPT_DLC (8U)
#define TURN_AUX_RPT_CANID (0x330)

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t DRIVER_BLINKER_BULB_ON : 1;            //      Bits= 1

  uint8_t PASS_BLINKER_BULB_ON : 1;              //      Bits= 1

  uint8_t DRIVER_BLINKER_BULB_ON_IS_VALID : 1;   //      Bits= 1

  uint8_t PASS_BLINKER_BULB_ON_IS_VALID : 1;     //      Bits= 1

#else

  uint8_t DRIVER_BLINKER_BULB_ON;                //      Bits= 1

  uint8_t PASS_BLINKER_BULB_ON;                  //      Bits= 1

  uint8_t DRIVER_BLINKER_BULB_ON_IS_VALID;       //      Bits= 1

  uint8_t PASS_BLINKER_BULB_ON_IS_VALID;         //      Bits= 1

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} TURN_AUX_RPT_t;

// def @WIPER_AUX_RPT CAN Message (820  0x334)
#define WIPER_AUX_RPT_IDE (0U)
#define WIPER_AUX_RPT_DLC (8U)
#define WIPER_AUX_RPT_CANID (0x334)

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t FRONT_WIPING : 1;                  //      Bits= 1

  uint8_t FRONT_SPRAYING : 1;                //      Bits= 1

  uint8_t REAR_WIPING : 1;                   //      Bits= 1

  uint8_t REAR_SPRAYING : 1;                 //      Bits= 1

  uint8_t SPRAY_NEAR_EMPTY : 1;              //      Bits= 1

  uint8_t SPRAY_EMPTY : 1;                   //      Bits= 1

  uint8_t FRONT_WIPING_IS_VALID : 1;         //      Bits= 1

  uint8_t FRONT_SPRAYING_IS_VALID : 1;       //      Bits= 1

  uint8_t REAR_WIPING_IS_VALID : 1;          //      Bits= 1

  uint8_t REAR_SPRAYING_IS_VALID : 1;        //      Bits= 1

  uint8_t SPRAY_NEAR_EMPTY_IS_VALID : 1;     //      Bits= 1

  uint8_t SPRAY_EMPTY_IS_VALID : 1;          //      Bits= 1

#else

  uint8_t FRONT_WIPING;                      //      Bits= 1

  uint8_t FRONT_SPRAYING;                    //      Bits= 1

  uint8_t REAR_WIPING;                       //      Bits= 1

  uint8_t REAR_SPRAYING;                     //      Bits= 1

  uint8_t SPRAY_NEAR_EMPTY;                  //      Bits= 1

  uint8_t SPRAY_EMPTY;                       //      Bits= 1

  uint8_t FRONT_WIPING_IS_VALID;             //      Bits= 1

  uint8_t FRONT_SPRAYING_IS_VALID;           //      Bits= 1

  uint8_t REAR_WIPING_IS_VALID;              //      Bits= 1

  uint8_t REAR_SPRAYING_IS_VALID;            //      Bits= 1

  uint8_t SPRAY_NEAR_EMPTY_IS_VALID;         //      Bits= 1

  uint8_t SPRAY_EMPTY_IS_VALID;              //      Bits= 1

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} WIPER_AUX_RPT_t;

// def @VEHICLE_SPEED_RPT CAN Message (1024 0x400)
#define VEHICLE_SPEED_RPT_IDE (0U)
#define VEHICLE_SPEED_RPT_DLC (8U)
#define VEHICLE_SPEED_RPT_CANID (0x400)
// signal: @VEHICLE_SPEED_ro
#define PACMOD3_VEHICLE_SPEED_ro_CovFactor (0.010000)
#define PACMOD3_VEHICLE_SPEED_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.010000)) )
#define PACMOD3_VEHICLE_SPEED_ro_fromS(x) ( (((x) * (0.010000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  int16_t VEHICLE_SPEED_ro;                  //  [-] Bits=16 Factor= 0.010000        Unit:'m/s'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t VEHICLE_SPEED_phys;
#endif // PACMOD3_USE_SIGFLOAT

  //  0 : "INVALID"
  //  1 : "VALID"
  // 
  uint8_t VEHICLE_SPEED_VALID : 1;           //      Bits= 1

#else

  int16_t VEHICLE_SPEED_ro;                  //  [-] Bits=16 Factor= 0.010000        Unit:'m/s'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t VEHICLE_SPEED_phys;
#endif // PACMOD3_USE_SIGFLOAT

  //  0 : "INVALID"
  //  1 : "VALID"
  // 
  uint8_t VEHICLE_SPEED_VALID;               //      Bits= 1

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} VEHICLE_SPEED_RPT_t;

// def @BRAKE_MOTOR_RPT_1 CAN Message (1025 0x401)
#define BRAKE_MOTOR_RPT_1_IDE (0U)
#define BRAKE_MOTOR_RPT_1_DLC (8U)
#define BRAKE_MOTOR_RPT_1_CANID (0x401)
// signal: @MOTOR_CURRENT_ro
#define PACMOD3_MOTOR_CURRENT_ro_CovFactor (0.001000)
#define PACMOD3_MOTOR_CURRENT_ro_toS(x) ( (int32_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD3_MOTOR_CURRENT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @SHAFT_POSITION_ro
#define PACMOD3_SHAFT_POSITION_ro_CovFactor (0.001000)
#define PACMOD3_SHAFT_POSITION_ro_toS(x) ( (int32_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD3_SHAFT_POSITION_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  int32_t MOTOR_CURRENT_ro;                  //  [-] Bits=32 Factor= 0.001000        Unit:'amps'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t MOTOR_CURRENT_phys;
#endif // PACMOD3_USE_SIGFLOAT

  int32_t SHAFT_POSITION_ro;                 //  [-] Bits=32 Factor= 0.001000        Unit:'radians'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t SHAFT_POSITION_phys;
#endif // PACMOD3_USE_SIGFLOAT

#else

  int32_t MOTOR_CURRENT_ro;                  //  [-] Bits=32 Factor= 0.001000        Unit:'amps'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t MOTOR_CURRENT_phys;
#endif // PACMOD3_USE_SIGFLOAT

  int32_t SHAFT_POSITION_ro;                 //  [-] Bits=32 Factor= 0.001000        Unit:'radians'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t SHAFT_POSITION_phys;
#endif // PACMOD3_USE_SIGFLOAT

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} BRAKE_MOTOR_RPT_1_t;

// def @BRAKE_MOTOR_RPT_2 CAN Message (1026 0x402)
#define BRAKE_MOTOR_RPT_2_IDE (0U)
#define BRAKE_MOTOR_RPT_2_DLC (8U)
#define BRAKE_MOTOR_RPT_2_CANID (0x402)
// signal: @ANGULAR_SPEED_ro
#define PACMOD3_ANGULAR_SPEED_ro_CovFactor (0.100000)
#define PACMOD3_ANGULAR_SPEED_ro_toS(x) ( (int32_t) (((x) - (0.000000)) / (0.100000)) )
#define PACMOD3_ANGULAR_SPEED_ro_fromS(x) ( (((x) * (0.100000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  int16_t ENCODER_TEMPERATURE;               //  [-] Bits=16 Unit:'deg_C'

  int16_t MOTOR_TEMPERATURE;                 //  [-] Bits=16 Unit:'deg_C'

  int32_t ANGULAR_SPEED_ro;                  //  [-] Bits=32 Factor= 0.100000        Unit:'rad/s'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t ANGULAR_SPEED_phys;
#endif // PACMOD3_USE_SIGFLOAT

#else

  int16_t ENCODER_TEMPERATURE;               //  [-] Bits=16 Unit:'deg_C'

  int16_t MOTOR_TEMPERATURE;                 //  [-] Bits=16 Unit:'deg_C'

  int32_t ANGULAR_SPEED_ro;                  //  [-] Bits=32 Factor= 0.100000        Unit:'rad/s'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t ANGULAR_SPEED_phys;
#endif // PACMOD3_USE_SIGFLOAT

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} BRAKE_MOTOR_RPT_2_t;

// def @BRAKE_MOTOR_RPT_3 CAN Message (1027 0x403)
#define BRAKE_MOTOR_RPT_3_IDE (0U)
#define BRAKE_MOTOR_RPT_3_DLC (8U)
#define BRAKE_MOTOR_RPT_3_CANID (0x403)
// signal: @TORQUE_OUTPUT_ro
#define PACMOD3_TORQUE_OUTPUT_ro_CovFactor (0.001000)
#define PACMOD3_TORQUE_OUTPUT_ro_toS(x) ( (int32_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD3_TORQUE_OUTPUT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @TORQUE_INPUT_ro
#define PACMOD3_TORQUE_INPUT_ro_CovFactor (0.001000)
#define PACMOD3_TORQUE_INPUT_ro_toS(x) ( (int32_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD3_TORQUE_INPUT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  int32_t TORQUE_OUTPUT_ro;                  //  [-] Bits=32 Factor= 0.001000        Unit:'N-m'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t TORQUE_OUTPUT_phys;
#endif // PACMOD3_USE_SIGFLOAT

  int32_t TORQUE_INPUT_ro;                   //  [-] Bits=32 Factor= 0.001000        Unit:'N-m'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t TORQUE_INPUT_phys;
#endif // PACMOD3_USE_SIGFLOAT

#else

  int32_t TORQUE_OUTPUT_ro;                  //  [-] Bits=32 Factor= 0.001000        Unit:'N-m'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t TORQUE_OUTPUT_phys;
#endif // PACMOD3_USE_SIGFLOAT

  int32_t TORQUE_INPUT_ro;                   //  [-] Bits=32 Factor= 0.001000        Unit:'N-m'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t TORQUE_INPUT_phys;
#endif // PACMOD3_USE_SIGFLOAT

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} BRAKE_MOTOR_RPT_3_t;

// def @STEERING_MOTOR_RPT_1 CAN Message (1028 0x404)
#define STEERING_MOTOR_RPT_1_IDE (0U)
#define STEERING_MOTOR_RPT_1_DLC (8U)
#define STEERING_MOTOR_RPT_1_CANID (0x404)
// signal: @MOTOR_CURRENT_ro
#define PACMOD3_MOTOR_CURRENT_ro_CovFactor (0.001000)
#define PACMOD3_MOTOR_CURRENT_ro_toS(x) ( (int32_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD3_MOTOR_CURRENT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @SHAFT_POSITION_ro
#define PACMOD3_SHAFT_POSITION_ro_CovFactor (0.001000)
#define PACMOD3_SHAFT_POSITION_ro_toS(x) ( (int32_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD3_SHAFT_POSITION_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  int32_t MOTOR_CURRENT_ro;                  //  [-] Bits=32 Factor= 0.001000        Unit:'amps'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t MOTOR_CURRENT_phys;
#endif // PACMOD3_USE_SIGFLOAT

  int32_t SHAFT_POSITION_ro;                 //  [-] Bits=32 Factor= 0.001000        Unit:'radians'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t SHAFT_POSITION_phys;
#endif // PACMOD3_USE_SIGFLOAT

#else

  int32_t MOTOR_CURRENT_ro;                  //  [-] Bits=32 Factor= 0.001000        Unit:'amps'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t MOTOR_CURRENT_phys;
#endif // PACMOD3_USE_SIGFLOAT

  int32_t SHAFT_POSITION_ro;                 //  [-] Bits=32 Factor= 0.001000        Unit:'radians'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t SHAFT_POSITION_phys;
#endif // PACMOD3_USE_SIGFLOAT

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} STEERING_MOTOR_RPT_1_t;

// def @STEERING_MOTOR_RPT_2 CAN Message (1029 0x405)
#define STEERING_MOTOR_RPT_2_IDE (0U)
#define STEERING_MOTOR_RPT_2_DLC (8U)
#define STEERING_MOTOR_RPT_2_CANID (0x405)
// signal: @ANGULAR_SPEED_ro
#define PACMOD3_ANGULAR_SPEED_ro_CovFactor (0.100000)
#define PACMOD3_ANGULAR_SPEED_ro_toS(x) ( (int32_t) (((x) - (0.000000)) / (0.100000)) )
#define PACMOD3_ANGULAR_SPEED_ro_fromS(x) ( (((x) * (0.100000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  int16_t ENCODER_TEMPERATURE;               //  [-] Bits=16 Unit:'deg_C'

  int16_t MOTOR_TEMPERATURE;                 //  [-] Bits=16 Unit:'deg_C'

  int32_t ANGULAR_SPEED_ro;                  //  [-] Bits=32 Factor= 0.100000        Unit:'rad/s'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t ANGULAR_SPEED_phys;
#endif // PACMOD3_USE_SIGFLOAT

#else

  int16_t ENCODER_TEMPERATURE;               //  [-] Bits=16 Unit:'deg_C'

  int16_t MOTOR_TEMPERATURE;                 //  [-] Bits=16 Unit:'deg_C'

  int32_t ANGULAR_SPEED_ro;                  //  [-] Bits=32 Factor= 0.100000        Unit:'rad/s'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t ANGULAR_SPEED_phys;
#endif // PACMOD3_USE_SIGFLOAT

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} STEERING_MOTOR_RPT_2_t;

// def @STEERING_MOTOR_RPT_3 CAN Message (1030 0x406)
#define STEERING_MOTOR_RPT_3_IDE (0U)
#define STEERING_MOTOR_RPT_3_DLC (8U)
#define STEERING_MOTOR_RPT_3_CANID (0x406)
// signal: @TORQUE_OUTPUT_ro
#define PACMOD3_TORQUE_OUTPUT_ro_CovFactor (0.001000)
#define PACMOD3_TORQUE_OUTPUT_ro_toS(x) ( (int32_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD3_TORQUE_OUTPUT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @TORQUE_INPUT_ro
#define PACMOD3_TORQUE_INPUT_ro_CovFactor (0.001000)
#define PACMOD3_TORQUE_INPUT_ro_toS(x) ( (int32_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD3_TORQUE_INPUT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  int32_t TORQUE_OUTPUT_ro;                  //  [-] Bits=32 Factor= 0.001000        Unit:'N-m'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t TORQUE_OUTPUT_phys;
#endif // PACMOD3_USE_SIGFLOAT

  int32_t TORQUE_INPUT_ro;                   //  [-] Bits=32 Factor= 0.001000        Unit:'N-m'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t TORQUE_INPUT_phys;
#endif // PACMOD3_USE_SIGFLOAT

#else

  int32_t TORQUE_OUTPUT_ro;                  //  [-] Bits=32 Factor= 0.001000        Unit:'N-m'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t TORQUE_OUTPUT_phys;
#endif // PACMOD3_USE_SIGFLOAT

  int32_t TORQUE_INPUT_ro;                   //  [-] Bits=32 Factor= 0.001000        Unit:'N-m'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t TORQUE_INPUT_phys;
#endif // PACMOD3_USE_SIGFLOAT

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} STEERING_MOTOR_RPT_3_t;

// def @WHEEL_SPEED_RPT CAN Message (1031 0x407)
#define WHEEL_SPEED_RPT_IDE (0U)
#define WHEEL_SPEED_RPT_DLC (8U)
#define WHEEL_SPEED_RPT_CANID (0x407)
// signal: @WHEEL_SPD_FRONT_LEFT_ro
#define PACMOD3_WHEEL_SPD_FRONT_LEFT_ro_CovFactor (0.010000)
#define PACMOD3_WHEEL_SPD_FRONT_LEFT_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.010000)) )
#define PACMOD3_WHEEL_SPD_FRONT_LEFT_ro_fromS(x) ( (((x) * (0.010000)) + (0.000000)) )
// signal: @WHEEL_SPD_FRONT_RIGHT_ro
#define PACMOD3_WHEEL_SPD_FRONT_RIGHT_ro_CovFactor (0.010000)
#define PACMOD3_WHEEL_SPD_FRONT_RIGHT_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.010000)) )
#define PACMOD3_WHEEL_SPD_FRONT_RIGHT_ro_fromS(x) ( (((x) * (0.010000)) + (0.000000)) )
// signal: @WHEEL_SPD_REAR_LEFT_ro
#define PACMOD3_WHEEL_SPD_REAR_LEFT_ro_CovFactor (0.010000)
#define PACMOD3_WHEEL_SPD_REAR_LEFT_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.010000)) )
#define PACMOD3_WHEEL_SPD_REAR_LEFT_ro_fromS(x) ( (((x) * (0.010000)) + (0.000000)) )
// signal: @WHEEL_SPD_REAR_RIGHT_ro
#define PACMOD3_WHEEL_SPD_REAR_RIGHT_ro_CovFactor (0.010000)
#define PACMOD3_WHEEL_SPD_REAR_RIGHT_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.010000)) )
#define PACMOD3_WHEEL_SPD_REAR_RIGHT_ro_fromS(x) ( (((x) * (0.010000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint16_t WHEEL_SPD_FRONT_LEFT_ro;          //      Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t WHEEL_SPD_FRONT_LEFT_phys;
#endif // PACMOD3_USE_SIGFLOAT

  int16_t WHEEL_SPD_FRONT_RIGHT_ro;          //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t WHEEL_SPD_FRONT_RIGHT_phys;
#endif // PACMOD3_USE_SIGFLOAT

  int16_t WHEEL_SPD_REAR_LEFT_ro;            //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t WHEEL_SPD_REAR_LEFT_phys;
#endif // PACMOD3_USE_SIGFLOAT

  int16_t WHEEL_SPD_REAR_RIGHT_ro;           //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t WHEEL_SPD_REAR_RIGHT_phys;
#endif // PACMOD3_USE_SIGFLOAT

#else

  uint16_t WHEEL_SPD_FRONT_LEFT_ro;          //      Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t WHEEL_SPD_FRONT_LEFT_phys;
#endif // PACMOD3_USE_SIGFLOAT

  int16_t WHEEL_SPD_FRONT_RIGHT_ro;          //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t WHEEL_SPD_FRONT_RIGHT_phys;
#endif // PACMOD3_USE_SIGFLOAT

  int16_t WHEEL_SPD_REAR_LEFT_ro;            //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t WHEEL_SPD_REAR_LEFT_phys;
#endif // PACMOD3_USE_SIGFLOAT

  int16_t WHEEL_SPD_REAR_RIGHT_ro;           //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t WHEEL_SPD_REAR_RIGHT_phys;
#endif // PACMOD3_USE_SIGFLOAT

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} WHEEL_SPEED_RPT_t;

// def @YAW_RATE_RPT CAN Message (1037 0x40d)
#define YAW_RATE_RPT_IDE (0U)
#define YAW_RATE_RPT_DLC (8U)
#define YAW_RATE_RPT_CANID (0x40d)
// signal: @YAW_RATE_ro
#define PACMOD3_YAW_RATE_ro_CovFactor (0.010000)
#define PACMOD3_YAW_RATE_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.010000)) )
#define PACMOD3_YAW_RATE_ro_fromS(x) ( (((x) * (0.010000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  int16_t YAW_RATE_ro;                       //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t YAW_RATE_phys;
#endif // PACMOD3_USE_SIGFLOAT

#else

  int16_t YAW_RATE_ro;                       //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t YAW_RATE_phys;
#endif // PACMOD3_USE_SIGFLOAT

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} YAW_RATE_RPT_t;

// def @LAT_LON_HEADING_RPT CAN Message (1038 0x40e)
#define LAT_LON_HEADING_RPT_IDE (0U)
#define LAT_LON_HEADING_RPT_DLC (8U)
#define LAT_LON_HEADING_RPT_CANID (0x40e)
// signal: @HEADING_ro
#define PACMOD3_HEADING_ro_CovFactor (0.010000)
#define PACMOD3_HEADING_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.010000)) )
#define PACMOD3_HEADING_ro_fromS(x) ( (((x) * (0.010000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  int8_t LATITUDE_DEGREES;                   //  [-] Bits= 8 Unit:'deg'

  int8_t LATITUDE_MINUTES;                   //  [-] Bits= 8 Unit:'min'

  int8_t LATITUDE_SECONDS;                   //  [-] Bits= 8 Unit:'sec'

  int8_t LONGITUDE_DEGREES;                  //  [-] Bits= 8 Unit:'deg'

  int8_t LONGITUDE_MINUTES;                  //  [-] Bits= 8 Unit:'min'

  int8_t LONGITUDE_SECONDS;                  //  [-] Bits= 8 Unit:'sec'

  int16_t HEADING_ro;                        //  [-] Bits=16 Factor= 0.010000        Unit:'deg'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t HEADING_phys;
#endif // PACMOD3_USE_SIGFLOAT

#else

  int8_t LATITUDE_DEGREES;                   //  [-] Bits= 8 Unit:'deg'

  int8_t LATITUDE_MINUTES;                   //  [-] Bits= 8 Unit:'min'

  int8_t LATITUDE_SECONDS;                   //  [-] Bits= 8 Unit:'sec'

  int8_t LONGITUDE_DEGREES;                  //  [-] Bits= 8 Unit:'deg'

  int8_t LONGITUDE_MINUTES;                  //  [-] Bits= 8 Unit:'min'

  int8_t LONGITUDE_SECONDS;                  //  [-] Bits= 8 Unit:'sec'

  int16_t HEADING_ro;                        //  [-] Bits=16 Factor= 0.010000        Unit:'deg'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t HEADING_phys;
#endif // PACMOD3_USE_SIGFLOAT

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} LAT_LON_HEADING_RPT_t;

// def @DATE_TIME_RPT CAN Message (1039 0x40f)
#define DATE_TIME_RPT_IDE (0U)
#define DATE_TIME_RPT_DLC (8U)
#define DATE_TIME_RPT_CANID (0x40f)
// signal: @DATE_YEAR_ro
#define PACMOD3_DATE_YEAR_ro_CovFactor (1)
#define PACMOD3_DATE_YEAR_ro_toS(x) ( (uint16_t) ((x) - (2000)) )
#define PACMOD3_DATE_YEAR_ro_fromS(x) ( ((x) + (2000)) )
// signal: @DATE_MONTH_ro
#define PACMOD3_DATE_MONTH_ro_CovFactor (1)
#define PACMOD3_DATE_MONTH_ro_toS(x) ( (uint16_t) ((x) - (1)) )
#define PACMOD3_DATE_MONTH_ro_fromS(x) ( ((x) + (1)) )
// signal: @DATE_DAY_ro
#define PACMOD3_DATE_DAY_ro_CovFactor (1)
#define PACMOD3_DATE_DAY_ro_toS(x) ( (uint16_t) ((x) - (1)) )
#define PACMOD3_DATE_DAY_ro_fromS(x) ( ((x) + (1)) )

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint16_t DATE_YEAR_ro;                     //      Bits= 8 Offset= 2000               Unit:'yr'

#ifdef PACMOD3_USE_SIGFLOAT
  uint16_t DATE_YEAR_phys;
#endif // PACMOD3_USE_SIGFLOAT

  uint16_t DATE_MONTH_ro;                    //      Bits= 8 Offset= 1                  Unit:'mon'

#ifdef PACMOD3_USE_SIGFLOAT
  uint16_t DATE_MONTH_phys;
#endif // PACMOD3_USE_SIGFLOAT

  uint16_t DATE_DAY_ro;                      //      Bits= 8 Offset= 1                  Unit:'dy'

#ifdef PACMOD3_USE_SIGFLOAT
  uint16_t DATE_DAY_phys;
#endif // PACMOD3_USE_SIGFLOAT

  uint8_t TIME_HOUR;                         //      Bits= 8 Unit:'hr'

  uint8_t TIME_MINUTE;                       //      Bits= 8 Unit:'min'

  uint8_t TIME_SECOND;                       //      Bits= 8 Unit:'sec'

#else

  uint16_t DATE_YEAR_ro;                     //      Bits= 8 Offset= 2000               Unit:'yr'

#ifdef PACMOD3_USE_SIGFLOAT
  uint16_t DATE_YEAR_phys;
#endif // PACMOD3_USE_SIGFLOAT

  uint16_t DATE_MONTH_ro;                    //      Bits= 8 Offset= 1                  Unit:'mon'

#ifdef PACMOD3_USE_SIGFLOAT
  uint16_t DATE_MONTH_phys;
#endif // PACMOD3_USE_SIGFLOAT

  uint16_t DATE_DAY_ro;                      //      Bits= 8 Offset= 1                  Unit:'dy'

#ifdef PACMOD3_USE_SIGFLOAT
  uint16_t DATE_DAY_phys;
#endif // PACMOD3_USE_SIGFLOAT

  uint8_t TIME_HOUR;                         //      Bits= 8 Unit:'hr'

  uint8_t TIME_MINUTE;                       //      Bits= 8 Unit:'min'

  uint8_t TIME_SECOND;                       //      Bits= 8 Unit:'sec'

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} DATE_TIME_RPT_t;

// def @DETECTED_OBJECT_RPT CAN Message (1041 0x411)
#define DETECTED_OBJECT_RPT_IDE (0U)
#define DETECTED_OBJECT_RPT_DLC (8U)
#define DETECTED_OBJECT_RPT_CANID (0x411)
// signal: @FRONT_OBJECT_DISTANCE_LOW_RES_ro
#define PACMOD3_FRONT_OBJECT_DISTANCE_LOW_RES_ro_CovFactor (0.001000)
#define PACMOD3_FRONT_OBJECT_DISTANCE_LOW_RES_ro_toS(x) ( (uint32_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD3_FRONT_OBJECT_DISTANCE_LOW_RES_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @FRONT_OBJECT_DISTANCE_HIGH_RES_ro
#define PACMOD3_FRONT_OBJECT_DISTANCE_HIGH_RES_ro_CovFactor (0.001000)
#define PACMOD3_FRONT_OBJECT_DISTANCE_HIGH_RES_ro_toS(x) ( (uint32_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD3_FRONT_OBJECT_DISTANCE_HIGH_RES_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint32_t FRONT_OBJECT_DISTANCE_LOW_RES_ro;       //      Bits=24 Factor= 0.001000        Unit:'m'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t FRONT_OBJECT_DISTANCE_LOW_RES_phys;
#endif // PACMOD3_USE_SIGFLOAT

  uint32_t FRONT_OBJECT_DISTANCE_HIGH_RES_ro;      //      Bits=24 Factor= 0.001000        Unit:'m'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t FRONT_OBJECT_DISTANCE_HIGH_RES_phys;
#endif // PACMOD3_USE_SIGFLOAT

#else

  uint32_t FRONT_OBJECT_DISTANCE_LOW_RES_ro;       //      Bits=24 Factor= 0.001000        Unit:'m'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t FRONT_OBJECT_DISTANCE_LOW_RES_phys;
#endif // PACMOD3_USE_SIGFLOAT

  uint32_t FRONT_OBJECT_DISTANCE_HIGH_RES_ro;      //      Bits=24 Factor= 0.001000        Unit:'m'

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t FRONT_OBJECT_DISTANCE_HIGH_RES_phys;
#endif // PACMOD3_USE_SIGFLOAT

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} DETECTED_OBJECT_RPT_t;

// def @VEH_SPECIFIC_RPT_1 CAN Message (1042 0x412)
#define VEH_SPECIFIC_RPT_1_IDE (0U)
#define VEH_SPECIFIC_RPT_1_DLC (8U)
#define VEH_SPECIFIC_RPT_1_CANID (0x412)

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t SHIFT_POS_1;                       //      Bits= 8

  uint8_t SHIFT_POS_2;                       //      Bits= 8

#else

  uint8_t SHIFT_POS_1;                       //      Bits= 8

  uint8_t SHIFT_POS_2;                       //      Bits= 8

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} VEH_SPECIFIC_RPT_1_t;

// def @VEH_DYNAMICS_RPT CAN Message (1043 0x413)
#define VEH_DYNAMICS_RPT_IDE (0U)
#define VEH_DYNAMICS_RPT_DLC (8U)
#define VEH_DYNAMICS_RPT_CANID (0x413)
// signal: @VEH_G_FORCES_ro
#define PACMOD3_VEH_G_FORCES_ro_CovFactor (0.001000)
#define PACMOD3_VEH_G_FORCES_ro_toS(x) ( (uint8_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD3_VEH_G_FORCES_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t VEH_G_FORCES_ro;                   //      Bits= 8 Factor= 0.001000       

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t VEH_G_FORCES_phys;
#endif // PACMOD3_USE_SIGFLOAT

#else

  uint8_t VEH_G_FORCES_ro;                   //      Bits= 8 Factor= 0.001000       

#ifdef PACMOD3_USE_SIGFLOAT
  sigfloat_t VEH_G_FORCES_phys;
#endif // PACMOD3_USE_SIGFLOAT

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} VEH_DYNAMICS_RPT_t;

// def @VIN_RPT CAN Message (1044 0x414)
#define VIN_RPT_IDE (0U)
#define VIN_RPT_DLC (8U)
#define VIN_RPT_CANID (0x414)

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint32_t VEH_MFG_CODE;                     //      Bits=24

  uint8_t VEH_MY_CODE;                       //      Bits= 8

  uint32_t VEH_SERIAL;                       //      Bits=24

#else

  uint32_t VEH_MFG_CODE;                     //      Bits=24

  uint8_t VEH_MY_CODE;                       //      Bits= 8

  uint32_t VEH_SERIAL;                       //      Bits=24

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} VIN_RPT_t;

// def @OCCUPANCY_RPT CAN Message (1045 0x415)
#define OCCUPANCY_RPT_IDE (0U)
#define OCCUPANCY_RPT_DLC (8U)
#define OCCUPANCY_RPT_CANID (0x415)

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t DRIVER_SEAT_OCCUPIED : 1;               //      Bits= 1

  uint8_t PASS_SEAT_OCCUPIED : 1;                 //      Bits= 1

  uint8_t REAR_SEAT_OCCUPIED : 1;                 //      Bits= 1

  uint8_t DRIVER_SEATBELT_BUCKLED : 1;            //      Bits= 1

  uint8_t PASS_SEATBELT_BUCKLED : 1;              //      Bits= 1

  uint8_t REAR_SEATBELT_BUCKLED : 1;              //      Bits= 1

  uint8_t DRIVER_SEAT_OCCUPIED_IS_VALID : 1;      //      Bits= 1

  uint8_t PASS_SEAT_OCCUPIED_IS_VALID : 1;        //      Bits= 1

  uint8_t REAR_SEAT_OCCUPIED_IS_VALID : 1;        //      Bits= 1

  uint8_t DRIVER_SEATBELT_BUCKLED_IS_VALID : 1;   //      Bits= 1

  uint8_t PASS_SEATBELT_BUCKLED_IS_VALID : 1;     //      Bits= 1

  uint8_t REAR_SEATBELT_BUCKLED_IS_VALID : 1;     //      Bits= 1

#else

  uint8_t DRIVER_SEAT_OCCUPIED;                   //      Bits= 1

  uint8_t PASS_SEAT_OCCUPIED;                     //      Bits= 1

  uint8_t REAR_SEAT_OCCUPIED;                     //      Bits= 1

  uint8_t DRIVER_SEATBELT_BUCKLED;                //      Bits= 1

  uint8_t PASS_SEATBELT_BUCKLED;                  //      Bits= 1

  uint8_t REAR_SEATBELT_BUCKLED;                  //      Bits= 1

  uint8_t DRIVER_SEAT_OCCUPIED_IS_VALID;          //      Bits= 1

  uint8_t PASS_SEAT_OCCUPIED_IS_VALID;            //      Bits= 1

  uint8_t REAR_SEAT_OCCUPIED_IS_VALID;            //      Bits= 1

  uint8_t DRIVER_SEATBELT_BUCKLED_IS_VALID;       //      Bits= 1

  uint8_t PASS_SEATBELT_BUCKLED_IS_VALID;         //      Bits= 1

  uint8_t REAR_SEATBELT_BUCKLED_IS_VALID;         //      Bits= 1

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} OCCUPANCY_RPT_t;

// def @INTERIOR_LIGHTS_RPT CAN Message (1046 0x416)
#define INTERIOR_LIGHTS_RPT_IDE (0U)
#define INTERIOR_LIGHTS_RPT_DLC (8U)
#define INTERIOR_LIGHTS_RPT_CANID (0x416)

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t FRONT_DOME_LIGHTS_ON : 1;            //      Bits= 1

  uint8_t REAR_DOME_LIGHTS_ON : 1;             //      Bits= 1

  uint8_t MOOD_LIGHTS_ON : 1;                  //      Bits= 1

  //  12 : "DIM_LEVEL_MAX"
  //  11 : "DIM_LEVEL_11"
  //  10 : "DIM_LEVEL_10"
  //  9 : "DIM_LEVEL_9"
  //  8 : "DIM_LEVEL_8"
  //  7 : "DIM_LEVEL_7"
  //  6 : "DIM_LEVEL_6"
  //  5 : "DIM_LEVEL_5"
  //  4 : "DIM_LEVEL_4"
  //  3 : "DIM_LEVEL_3"
  //  2 : "DIM_LEVEL_2"
  //  1 : "DIM_LEVEL_1"
  //  0 : "DIM_LEVEL_MIN"
  // 
  uint8_t DIM_LEVEL;                           //      Bits= 8

  uint8_t FRONT_DOME_LIGHTS_ON_IS_VALID : 1;   //      Bits= 1

  uint8_t REAR_DOME_LIGHTS_ON_IS_VALID : 1;    //      Bits= 1

  uint8_t MOOD_LIGHTS_ON_IS_VALID : 1;         //      Bits= 1

  uint8_t DIM_LEVEL_IS_VALID : 1;              //      Bits= 1

#else

  uint8_t FRONT_DOME_LIGHTS_ON;                //      Bits= 1

  uint8_t REAR_DOME_LIGHTS_ON;                 //      Bits= 1

  uint8_t MOOD_LIGHTS_ON;                      //      Bits= 1

  //  12 : "DIM_LEVEL_MAX"
  //  11 : "DIM_LEVEL_11"
  //  10 : "DIM_LEVEL_10"
  //  9 : "DIM_LEVEL_9"
  //  8 : "DIM_LEVEL_8"
  //  7 : "DIM_LEVEL_7"
  //  6 : "DIM_LEVEL_6"
  //  5 : "DIM_LEVEL_5"
  //  4 : "DIM_LEVEL_4"
  //  3 : "DIM_LEVEL_3"
  //  2 : "DIM_LEVEL_2"
  //  1 : "DIM_LEVEL_1"
  //  0 : "DIM_LEVEL_MIN"
  // 
  uint8_t DIM_LEVEL;                           //      Bits= 8

  uint8_t FRONT_DOME_LIGHTS_ON_IS_VALID;       //      Bits= 1

  uint8_t REAR_DOME_LIGHTS_ON_IS_VALID;        //      Bits= 1

  uint8_t MOOD_LIGHTS_ON_IS_VALID;             //      Bits= 1

  uint8_t DIM_LEVEL_IS_VALID;                  //      Bits= 1

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} INTERIOR_LIGHTS_RPT_t;

// def @DOOR_RPT CAN Message (1047 0x417)
#define DOOR_RPT_IDE (0U)
#define DOOR_RPT_DLC (8U)
#define DOOR_RPT_CANID (0x417)

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t DRIVER_DOOR_OPEN : 1;                 //      Bits= 1

  uint8_t PASS_DOOR_OPEN : 1;                   //      Bits= 1

  uint8_t REAR_DRIVER_DOOR_OPEN : 1;            //      Bits= 1

  uint8_t REAR_PASS_DOOR_OPEN : 1;              //      Bits= 1

  uint8_t HOOD_OPEN : 1;                        //      Bits= 1

  uint8_t TRUNK_OPEN : 1;                       //      Bits= 1

  uint8_t FUEL_DOOR_OPEN : 1;                   //      Bits= 1

  uint8_t DRIVER_DOOR_OPEN_IS_VALID : 1;        //      Bits= 1

  uint8_t PASS_DOOR_OPEN_IS_VALID : 1;          //      Bits= 1

  uint8_t REAR_DRIVER_DOOR_OPEN_IS_VALID : 1;   //      Bits= 1

  uint8_t REAR_PASS_DOOR_OPEN_IS_VALID : 1;     //      Bits= 1

  uint8_t HOOD_OPEN_IS_VALID : 1;               //      Bits= 1

  uint8_t TRUNK_OPEN_IS_VALID : 1;              //      Bits= 1

  uint8_t FUEL_DOOR_OPEN_IS_VALID : 1;          //      Bits= 1

#else

  uint8_t DRIVER_DOOR_OPEN;                     //      Bits= 1

  uint8_t PASS_DOOR_OPEN;                       //      Bits= 1

  uint8_t REAR_DRIVER_DOOR_OPEN;                //      Bits= 1

  uint8_t REAR_PASS_DOOR_OPEN;                  //      Bits= 1

  uint8_t HOOD_OPEN;                            //      Bits= 1

  uint8_t TRUNK_OPEN;                           //      Bits= 1

  uint8_t FUEL_DOOR_OPEN;                       //      Bits= 1

  uint8_t DRIVER_DOOR_OPEN_IS_VALID;            //      Bits= 1

  uint8_t PASS_DOOR_OPEN_IS_VALID;              //      Bits= 1

  uint8_t REAR_DRIVER_DOOR_OPEN_IS_VALID;       //      Bits= 1

  uint8_t REAR_PASS_DOOR_OPEN_IS_VALID;         //      Bits= 1

  uint8_t HOOD_OPEN_IS_VALID;                   //      Bits= 1

  uint8_t TRUNK_OPEN_IS_VALID;                  //      Bits= 1

  uint8_t FUEL_DOOR_OPEN_IS_VALID;              //      Bits= 1

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} DOOR_RPT_t;

// def @REAR_LIGHTS_RPT CAN Message (1048 0x418)
#define REAR_LIGHTS_RPT_IDE (0U)
#define REAR_LIGHTS_RPT_DLC (8U)
#define REAR_LIGHTS_RPT_CANID (0x418)

typedef struct
{
#ifdef PACMOD3_USE_BITS_SIGNAL

  uint8_t BRAKE_LIGHTS_ON : 1;               //      Bits= 1

  uint8_t REVERSE_LIGHTS_ON : 1;             //      Bits= 1

  uint8_t BRAKE_LIGHTS_ON_IS_VALID : 1;      //      Bits= 1

  uint8_t REVERSE_LIGHTS_ON_IS_VALID : 1;    //      Bits= 1

#else

  uint8_t BRAKE_LIGHTS_ON;                   //      Bits= 1

  uint8_t REVERSE_LIGHTS_ON;                 //      Bits= 1

  uint8_t BRAKE_LIGHTS_ON_IS_VALID;          //      Bits= 1

  uint8_t REVERSE_LIGHTS_ON_IS_VALID;        //      Bits= 1

#endif // PACMOD3_USE_BITS_SIGNAL

#ifdef PACMOD3_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD3_USE_DIAG_MONITORS

} REAR_LIGHTS_RPT_t;

// Function signatures

uint32_t Unpack_GLOBAL_RPT_pacmod3(GLOBAL_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_GLOBAL_RPT_pacmod3(GLOBAL_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_GLOBAL_RPT_pacmod3(GLOBAL_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_COMPONENT_RPT_pacmod3(COMPONENT_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_COMPONENT_RPT_pacmod3(COMPONENT_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_COMPONENT_RPT_pacmod3(COMPONENT_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_ACCEL_CMD_pacmod3(ACCEL_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_ACCEL_CMD_pacmod3(ACCEL_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_ACCEL_CMD_pacmod3(ACCEL_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_BRAKE_CMD_pacmod3(BRAKE_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_BRAKE_CMD_pacmod3(BRAKE_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BRAKE_CMD_pacmod3(BRAKE_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_CRUISE_CONTROL_BUTTONS_CMD_pacmod3(CRUISE_CONTROL_BUTTONS_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_CRUISE_CONTROL_BUTTONS_CMD_pacmod3(CRUISE_CONTROL_BUTTONS_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CRUISE_CONTROL_BUTTONS_CMD_pacmod3(CRUISE_CONTROL_BUTTONS_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_DASH_CONTROLS_LEFT_CMD_pacmod3(DASH_CONTROLS_LEFT_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_DASH_CONTROLS_LEFT_CMD_pacmod3(DASH_CONTROLS_LEFT_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DASH_CONTROLS_LEFT_CMD_pacmod3(DASH_CONTROLS_LEFT_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_DASH_CONTROLS_RIGHT_CMD_pacmod3(DASH_CONTROLS_RIGHT_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_DASH_CONTROLS_RIGHT_CMD_pacmod3(DASH_CONTROLS_RIGHT_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DASH_CONTROLS_RIGHT_CMD_pacmod3(DASH_CONTROLS_RIGHT_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_HAZARD_LIGHTS_CMD_pacmod3(HAZARD_LIGHTS_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_HAZARD_LIGHTS_CMD_pacmod3(HAZARD_LIGHTS_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_HAZARD_LIGHTS_CMD_pacmod3(HAZARD_LIGHTS_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_HEADLIGHT_CMD_pacmod3(HEADLIGHT_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_HEADLIGHT_CMD_pacmod3(HEADLIGHT_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_HEADLIGHT_CMD_pacmod3(HEADLIGHT_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_HORN_CMD_pacmod3(HORN_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_HORN_CMD_pacmod3(HORN_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_HORN_CMD_pacmod3(HORN_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_MEDIA_CONTROLS_CMD_pacmod3(MEDIA_CONTROLS_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_MEDIA_CONTROLS_CMD_pacmod3(MEDIA_CONTROLS_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_MEDIA_CONTROLS_CMD_pacmod3(MEDIA_CONTROLS_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_PARKING_BRAKE_CMD_pacmod3(PARKING_BRAKE_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_PARKING_BRAKE_CMD_pacmod3(PARKING_BRAKE_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_PARKING_BRAKE_CMD_pacmod3(PARKING_BRAKE_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_SHIFT_CMD_pacmod3(SHIFT_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_SHIFT_CMD_pacmod3(SHIFT_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_SHIFT_CMD_pacmod3(SHIFT_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_STEERING_CMD_pacmod3(STEERING_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_STEERING_CMD_pacmod3(STEERING_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_STEERING_CMD_pacmod3(STEERING_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_TURN_CMD_pacmod3(TURN_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_TURN_CMD_pacmod3(TURN_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_TURN_CMD_pacmod3(TURN_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_WIPER_CMD_pacmod3(WIPER_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_WIPER_CMD_pacmod3(WIPER_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_WIPER_CMD_pacmod3(WIPER_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_ACCEL_RPT_pacmod3(ACCEL_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_ACCEL_RPT_pacmod3(ACCEL_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_ACCEL_RPT_pacmod3(ACCEL_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_BRAKE_RPT_pacmod3(BRAKE_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_BRAKE_RPT_pacmod3(BRAKE_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BRAKE_RPT_pacmod3(BRAKE_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_CRUISE_CONTROL_BUTTONS_RPT_pacmod3(CRUISE_CONTROL_BUTTONS_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_CRUISE_CONTROL_BUTTONS_RPT_pacmod3(CRUISE_CONTROL_BUTTONS_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CRUISE_CONTROL_BUTTONS_RPT_pacmod3(CRUISE_CONTROL_BUTTONS_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_DASH_CONTROLS_LEFT_RPT_pacmod3(DASH_CONTROLS_LEFT_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_DASH_CONTROLS_LEFT_RPT_pacmod3(DASH_CONTROLS_LEFT_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DASH_CONTROLS_LEFT_RPT_pacmod3(DASH_CONTROLS_LEFT_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_DASH_CONTROLS_RIGHT_RPT_pacmod3(DASH_CONTROLS_RIGHT_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_DASH_CONTROLS_RIGHT_RPT_pacmod3(DASH_CONTROLS_RIGHT_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DASH_CONTROLS_RIGHT_RPT_pacmod3(DASH_CONTROLS_RIGHT_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_HAZARD_LIGHTS_RPT_pacmod3(HAZARD_LIGHTS_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_HAZARD_LIGHTS_RPT_pacmod3(HAZARD_LIGHTS_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_HAZARD_LIGHTS_RPT_pacmod3(HAZARD_LIGHTS_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_HEADLIGHT_RPT_pacmod3(HEADLIGHT_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_HEADLIGHT_RPT_pacmod3(HEADLIGHT_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_HEADLIGHT_RPT_pacmod3(HEADLIGHT_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_HORN_RPT_pacmod3(HORN_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_HORN_RPT_pacmod3(HORN_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_HORN_RPT_pacmod3(HORN_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_MEDIA_CONTROLS_RPT_pacmod3(MEDIA_CONTROLS_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_MEDIA_CONTROLS_RPT_pacmod3(MEDIA_CONTROLS_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_MEDIA_CONTROLS_RPT_pacmod3(MEDIA_CONTROLS_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_PARKING_BRAKE_RPT_pacmod3(PARKING_BRAKE_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_PARKING_BRAKE_RPT_pacmod3(PARKING_BRAKE_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_PARKING_BRAKE_RPT_pacmod3(PARKING_BRAKE_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_SHIFT_RPT_pacmod3(SHIFT_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_SHIFT_RPT_pacmod3(SHIFT_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_SHIFT_RPT_pacmod3(SHIFT_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_STEERING_RPT_pacmod3(STEERING_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_STEERING_RPT_pacmod3(STEERING_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_STEERING_RPT_pacmod3(STEERING_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_TURN_RPT_pacmod3(TURN_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_TURN_RPT_pacmod3(TURN_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_TURN_RPT_pacmod3(TURN_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_WIPER_RPT_pacmod3(WIPER_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_WIPER_RPT_pacmod3(WIPER_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_WIPER_RPT_pacmod3(WIPER_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_ACCEL_AUX_RPT_pacmod3(ACCEL_AUX_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_ACCEL_AUX_RPT_pacmod3(ACCEL_AUX_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_ACCEL_AUX_RPT_pacmod3(ACCEL_AUX_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_BRAKE_AUX_RPT_pacmod3(BRAKE_AUX_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_BRAKE_AUX_RPT_pacmod3(BRAKE_AUX_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BRAKE_AUX_RPT_pacmod3(BRAKE_AUX_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_HEADLIGHT_AUX_RPT_pacmod3(HEADLIGHT_AUX_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_HEADLIGHT_AUX_RPT_pacmod3(HEADLIGHT_AUX_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_HEADLIGHT_AUX_RPT_pacmod3(HEADLIGHT_AUX_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_SHIFT_AUX_RPT_pacmod3(SHIFT_AUX_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_SHIFT_AUX_RPT_pacmod3(SHIFT_AUX_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_SHIFT_AUX_RPT_pacmod3(SHIFT_AUX_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_STEERING_AUX_RPT_pacmod3(STEERING_AUX_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_STEERING_AUX_RPT_pacmod3(STEERING_AUX_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_STEERING_AUX_RPT_pacmod3(STEERING_AUX_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_TURN_AUX_RPT_pacmod3(TURN_AUX_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_TURN_AUX_RPT_pacmod3(TURN_AUX_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_TURN_AUX_RPT_pacmod3(TURN_AUX_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_WIPER_AUX_RPT_pacmod3(WIPER_AUX_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_WIPER_AUX_RPT_pacmod3(WIPER_AUX_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_WIPER_AUX_RPT_pacmod3(WIPER_AUX_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_VEHICLE_SPEED_RPT_pacmod3(VEHICLE_SPEED_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_VEHICLE_SPEED_RPT_pacmod3(VEHICLE_SPEED_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_VEHICLE_SPEED_RPT_pacmod3(VEHICLE_SPEED_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_BRAKE_MOTOR_RPT_1_pacmod3(BRAKE_MOTOR_RPT_1_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_BRAKE_MOTOR_RPT_1_pacmod3(BRAKE_MOTOR_RPT_1_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BRAKE_MOTOR_RPT_1_pacmod3(BRAKE_MOTOR_RPT_1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_BRAKE_MOTOR_RPT_2_pacmod3(BRAKE_MOTOR_RPT_2_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_BRAKE_MOTOR_RPT_2_pacmod3(BRAKE_MOTOR_RPT_2_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BRAKE_MOTOR_RPT_2_pacmod3(BRAKE_MOTOR_RPT_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_BRAKE_MOTOR_RPT_3_pacmod3(BRAKE_MOTOR_RPT_3_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_BRAKE_MOTOR_RPT_3_pacmod3(BRAKE_MOTOR_RPT_3_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BRAKE_MOTOR_RPT_3_pacmod3(BRAKE_MOTOR_RPT_3_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_STEERING_MOTOR_RPT_1_pacmod3(STEERING_MOTOR_RPT_1_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_STEERING_MOTOR_RPT_1_pacmod3(STEERING_MOTOR_RPT_1_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_STEERING_MOTOR_RPT_1_pacmod3(STEERING_MOTOR_RPT_1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_STEERING_MOTOR_RPT_2_pacmod3(STEERING_MOTOR_RPT_2_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_STEERING_MOTOR_RPT_2_pacmod3(STEERING_MOTOR_RPT_2_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_STEERING_MOTOR_RPT_2_pacmod3(STEERING_MOTOR_RPT_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_STEERING_MOTOR_RPT_3_pacmod3(STEERING_MOTOR_RPT_3_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_STEERING_MOTOR_RPT_3_pacmod3(STEERING_MOTOR_RPT_3_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_STEERING_MOTOR_RPT_3_pacmod3(STEERING_MOTOR_RPT_3_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_WHEEL_SPEED_RPT_pacmod3(WHEEL_SPEED_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_WHEEL_SPEED_RPT_pacmod3(WHEEL_SPEED_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_WHEEL_SPEED_RPT_pacmod3(WHEEL_SPEED_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_YAW_RATE_RPT_pacmod3(YAW_RATE_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_YAW_RATE_RPT_pacmod3(YAW_RATE_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_YAW_RATE_RPT_pacmod3(YAW_RATE_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_LAT_LON_HEADING_RPT_pacmod3(LAT_LON_HEADING_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_LAT_LON_HEADING_RPT_pacmod3(LAT_LON_HEADING_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_LAT_LON_HEADING_RPT_pacmod3(LAT_LON_HEADING_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_DATE_TIME_RPT_pacmod3(DATE_TIME_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_DATE_TIME_RPT_pacmod3(DATE_TIME_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DATE_TIME_RPT_pacmod3(DATE_TIME_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_DETECTED_OBJECT_RPT_pacmod3(DETECTED_OBJECT_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_DETECTED_OBJECT_RPT_pacmod3(DETECTED_OBJECT_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DETECTED_OBJECT_RPT_pacmod3(DETECTED_OBJECT_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_VEH_SPECIFIC_RPT_1_pacmod3(VEH_SPECIFIC_RPT_1_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_VEH_SPECIFIC_RPT_1_pacmod3(VEH_SPECIFIC_RPT_1_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_VEH_SPECIFIC_RPT_1_pacmod3(VEH_SPECIFIC_RPT_1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_VEH_DYNAMICS_RPT_pacmod3(VEH_DYNAMICS_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_VEH_DYNAMICS_RPT_pacmod3(VEH_DYNAMICS_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_VEH_DYNAMICS_RPT_pacmod3(VEH_DYNAMICS_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_VIN_RPT_pacmod3(VIN_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_VIN_RPT_pacmod3(VIN_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_VIN_RPT_pacmod3(VIN_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_OCCUPANCY_RPT_pacmod3(OCCUPANCY_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_OCCUPANCY_RPT_pacmod3(OCCUPANCY_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_OCCUPANCY_RPT_pacmod3(OCCUPANCY_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_INTERIOR_LIGHTS_RPT_pacmod3(INTERIOR_LIGHTS_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_INTERIOR_LIGHTS_RPT_pacmod3(INTERIOR_LIGHTS_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INTERIOR_LIGHTS_RPT_pacmod3(INTERIOR_LIGHTS_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_DOOR_RPT_pacmod3(DOOR_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_DOOR_RPT_pacmod3(DOOR_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DOOR_RPT_pacmod3(DOOR_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

uint32_t Unpack_REAR_LIGHTS_RPT_pacmod3(REAR_LIGHTS_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD3_USE_CANSTRUCT
uint32_t Pack_REAR_LIGHTS_RPT_pacmod3(REAR_LIGHTS_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_REAR_LIGHTS_RPT_pacmod3(REAR_LIGHTS_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD3_USE_CANSTRUCT

#ifdef __cplusplus
}
#endif
