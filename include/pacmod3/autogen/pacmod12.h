#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// DBC file version
#define VER_PACMOD12_MAJ (0U)
#define VER_PACMOD12_MIN (0U)

// include current dbc-driver compilation config
#include "pacmod12-config.h"

#ifdef PACMOD12_USE_DIAG_MONITORS
// This file must define:
// base monitor struct
// function signature for HASH calculation: (@GetFrameHash)
// function signature for getting system tick value: (@GetSystemTick)
#include "canmonitorutil.h"

#endif // PACMOD12_USE_DIAG_MONITORS


// Not for new development.
// def @GLOBAL_RPT CAN Message (16   0x10)
#define GLOBAL_RPT_IDE (0U)
#define GLOBAL_RPT_DLC (8U)
#define GLOBAL_RPT_CANID (0x10)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "CONTROL_DISABLED"
  //  1 : "CONTROL_ENABLED"
  // 
  uint8_t PACMOD_SYSTEM_ENABLED : 1;           //      Bits= 1

  //  0 : "NOT_OVERRIDDEN"
  //  1 : "OVERRIDDEN"
  // 
  uint8_t PACMOD_SYSTEM_OVERRIDE_ACTIVE : 1;   //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t USR_CAN_TIMEOUT : 1;                 //      Bits= 1

  //  0 : "NO_ACTIVE_CAN_TIMEOUT"
  //  1 : "ACTIVE_CAN_TIMEOUT"
  // 
  uint8_t STR_CAN_TIMEOUT : 1;                 //      Bits= 1

  //  0 : "NO_ACTIVE_CAN_TIMEOUT"
  //  1 : "ACTIVE_CAN_TIMEOUT"
  // 
  uint8_t BRK_CAN_TIMEOUT : 1;                 //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD_SUBSYSTEM_TIMEOUT : 1;        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t VEH_CAN_TIMEOUT : 1;                 //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD_SYSTEM_FAULT_ACTIVE : 1;      //      Bits= 1

  // Supervisory Enablement Rules apply when SUPERVISORY_ENABLE_REQUIRED is REQUIRED.  It provides backward compatibility with prior versions of this specification that did not have the SUPERVISORY_CTRL message.
  //  0 : "NOT_REQUIRED"
  //  1 : "REQUIRED"
  // 
  uint8_t SUPERVISORY_ENABLE_REQUIRED : 1;     //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CONFIG_FAULT_ACTIVE : 1;             //      Bits= 1

  uint16_t USR_CAN_READ_ERRORS;                //      Bits=16

#else

  //  0 : "CONTROL_DISABLED"
  //  1 : "CONTROL_ENABLED"
  // 
  uint8_t PACMOD_SYSTEM_ENABLED;               //      Bits= 1

  //  0 : "NOT_OVERRIDDEN"
  //  1 : "OVERRIDDEN"
  // 
  uint8_t PACMOD_SYSTEM_OVERRIDE_ACTIVE;       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t USR_CAN_TIMEOUT;                     //      Bits= 1

  //  0 : "NO_ACTIVE_CAN_TIMEOUT"
  //  1 : "ACTIVE_CAN_TIMEOUT"
  // 
  uint8_t STR_CAN_TIMEOUT;                     //      Bits= 1

  //  0 : "NO_ACTIVE_CAN_TIMEOUT"
  //  1 : "ACTIVE_CAN_TIMEOUT"
  // 
  uint8_t BRK_CAN_TIMEOUT;                     //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD_SUBSYSTEM_TIMEOUT;            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t VEH_CAN_TIMEOUT;                     //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD_SYSTEM_FAULT_ACTIVE;          //      Bits= 1

  // Supervisory Enablement Rules apply when SUPERVISORY_ENABLE_REQUIRED is REQUIRED.  It provides backward compatibility with prior versions of this specification that did not have the SUPERVISORY_CTRL message.
  //  0 : "NOT_REQUIRED"
  //  1 : "REQUIRED"
  // 
  uint8_t SUPERVISORY_ENABLE_REQUIRED;         //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CONFIG_FAULT_ACTIVE;                 //      Bits= 1

  uint16_t USR_CAN_READ_ERRORS;                //      Bits=16

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} GLOBAL_RPT_t;

// def @GLOBAL_RPT_2 CAN Message (17   0x11)
#define GLOBAL_RPT_2_IDE (0U)
#define GLOBAL_RPT_2_DLC (1U)
#define GLOBAL_RPT_2_CANID (0x11)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  // One or more systems are enabled.
  //  0 : "CONTROL_DISABLED"
  //  1 : "CONTROL_ENABLED"
  // 
  uint8_t SYSTEM_ENABLED : 1;                //      Bits= 1

  //  0 : "NOT_OVERRIDDEN"
  //  1 : "OVERRIDDEN"
  // 
  uint8_t SYSTEM_OVERRIDE_ACTIVE : 1;        //      Bits= 1

  // One or more faults are active.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SYSTEM_FAULT_ACTIVE : 1;           //      Bits= 1

  // Supervisory Enablement Rules apply when SUPERVISORY_ENABLE_REQUIRED is REQUIRED.  It provides backward compatibility with prior versions of this specification that did not have the SUPERVISORY_CTRL message.
  //  0 : "NOT_REQUIRED"
  //  1 : "REQUIRED"
  // 
  uint8_t SUPERVISORY_ENABLE_REQUIRED : 1;   //      Bits= 1

  // Global Command Sanity Check Rules and System Global Disable Rules apply when SANITY_CHECK_REQUIRED is REQUIRED.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t DISABLE_ALL_SYSTEMS : 1;           //      Bits= 1

  // System Ready Rules apply at all times.
  //  0 : "NOT_READY"
  //  1 : "READY"
  // 
  uint8_t SYSTEM_READY : 1;                  //      Bits= 1

#else

  // One or more systems are enabled.
  //  0 : "CONTROL_DISABLED"
  //  1 : "CONTROL_ENABLED"
  // 
  uint8_t SYSTEM_ENABLED;                    //      Bits= 1

  //  0 : "NOT_OVERRIDDEN"
  //  1 : "OVERRIDDEN"
  // 
  uint8_t SYSTEM_OVERRIDE_ACTIVE;            //      Bits= 1

  // One or more faults are active.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SYSTEM_FAULT_ACTIVE;               //      Bits= 1

  // Supervisory Enablement Rules apply when SUPERVISORY_ENABLE_REQUIRED is REQUIRED.  It provides backward compatibility with prior versions of this specification that did not have the SUPERVISORY_CTRL message.
  //  0 : "NOT_REQUIRED"
  //  1 : "REQUIRED"
  // 
  uint8_t SUPERVISORY_ENABLE_REQUIRED;       //      Bits= 1

  // Global Command Sanity Check Rules and System Global Disable Rules apply when SANITY_CHECK_REQUIRED is REQUIRED.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t DISABLE_ALL_SYSTEMS;               //      Bits= 1

  // System Ready Rules apply at all times.
  //  0 : "NOT_READY"
  //  1 : "READY"
  // 
  uint8_t SYSTEM_READY;                      //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} GLOBAL_RPT_2_t;

// def @COMPONENT_RPT_00 CAN Message (32   0x20)
#define COMPONENT_RPT_00_IDE (0U)
#define COMPONENT_RPT_00_DLC (6U)
#define COMPONENT_RPT_00_CANID (0x20)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "PACMOD"
  //  1 : "PACMINI"
  //  2 : "PACMICRO"
  // 
  uint8_t COMPONENT_TYPE : 4;                  //      Bits= 4

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t ACCEL : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t BRAKE : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CRUISE_CONTROL_BUTTONS : 1;          //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t DASH_CONTROLS_LEFT : 1;              //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t DASH_CONTROLS_RIGHT : 1;             //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t HAZARD_LIGHTS : 1;                   //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t HEADLIGHT : 1;                       //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t HORN : 1;                            //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t MEDIA_CONTROLS : 1;                  //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t PARKING_BRAKE : 1;                   //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t SHIFT : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t SPRAYER : 1;                         //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t STEERING : 1;                        //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t TURN : 1;                            //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t WIPER : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t WATCHDOG : 1;                        //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t BRAKE_DECEL : 1;                     //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t REAR_PASS_DOOR : 1;                  //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t ENGINE_BRAKE : 1;                    //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t MARKER_LAMP : 1;                     //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CABIN_CLIMATE : 1;                   //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CABIN_FAN_SPEED : 1;                 //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CABIN_TEMP : 1;                      //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t EXHAUST_BRAKE : 1;                   //      Bits= 1

  // Counter shall have the value of 0 with the first message transmission.  It shall increase by 1 with each subsequent message transmission up to and including the value of 0xF.  The next message transmission shall be 0, and this pattern shall repeat.
  uint8_t COUNTER : 4;                         //      Bits= 4

  // The COMPLEMENT shall be the complement of the COUNTER.  For example, if COUNTER is 0x1011, then the COMPLEMENT is 0x0100.
  uint8_t COMPLEMENT : 4;                      //      Bits= 4

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t CONFIG_FAULT : 1;                    //      Bits= 1

  // This value relates to message specific or general CAN timeouts.  It includes the watchdog component report timeout.
  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t CAN_TIMEOUT_FAULT : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INTERNAL_SUPPLY_VOLTAGE_FAULT : 1;   //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t SUPERVISORY_TIMEOUT : 1;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t SUPERVISORY_SANITY_FAULT : 1;        //      Bits= 1

  // This value relates to problems with the COUNTER and COMPLIMENT signals in the component report received from the watchdog component.
  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t WATCHDOG_SANITY_FAULT : 1;           //      Bits= 1

  // This value shall be TRUE when the WATCHDOG system present signal in the component report received from the watchdog component is FALSE or any other system present signal is TRUE.
  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t WATCHDOG_SYSTEM_PRESENT_FAULT : 1;   //      Bits= 1

  // This value will indicate whether this component is ready for by-wire control.
  //  0 : "NOT_READY"
  //  1 : "READY"
  // 
  uint8_t COMPONENT_READY : 1;                 //      Bits= 1

#else

  //  0 : "PACMOD"
  //  1 : "PACMINI"
  //  2 : "PACMICRO"
  // 
  uint8_t COMPONENT_TYPE;                      //      Bits= 4

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t ACCEL;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t BRAKE;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CRUISE_CONTROL_BUTTONS;              //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t DASH_CONTROLS_LEFT;                  //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t DASH_CONTROLS_RIGHT;                 //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t HAZARD_LIGHTS;                       //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t HEADLIGHT;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t HORN;                                //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t MEDIA_CONTROLS;                      //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t PARKING_BRAKE;                       //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t SHIFT;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t SPRAYER;                             //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t STEERING;                            //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t TURN;                                //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t WIPER;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t WATCHDOG;                            //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t BRAKE_DECEL;                         //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t REAR_PASS_DOOR;                      //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t ENGINE_BRAKE;                        //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t MARKER_LAMP;                         //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CABIN_CLIMATE;                       //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CABIN_FAN_SPEED;                     //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CABIN_TEMP;                          //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t EXHAUST_BRAKE;                       //      Bits= 1

  // Counter shall have the value of 0 with the first message transmission.  It shall increase by 1 with each subsequent message transmission up to and including the value of 0xF.  The next message transmission shall be 0, and this pattern shall repeat.
  uint8_t COUNTER;                             //      Bits= 4

  // The COMPLEMENT shall be the complement of the COUNTER.  For example, if COUNTER is 0x1011, then the COMPLEMENT is 0x0100.
  uint8_t COMPLEMENT;                          //      Bits= 4

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t CONFIG_FAULT;                        //      Bits= 1

  // This value relates to message specific or general CAN timeouts.  It includes the watchdog component report timeout.
  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t CAN_TIMEOUT_FAULT;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INTERNAL_SUPPLY_VOLTAGE_FAULT;       //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t SUPERVISORY_TIMEOUT;                 //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t SUPERVISORY_SANITY_FAULT;            //      Bits= 1

  // This value relates to problems with the COUNTER and COMPLIMENT signals in the component report received from the watchdog component.
  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t WATCHDOG_SANITY_FAULT;               //      Bits= 1

  // This value shall be TRUE when the WATCHDOG system present signal in the component report received from the watchdog component is FALSE or any other system present signal is TRUE.
  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t WATCHDOG_SYSTEM_PRESENT_FAULT;       //      Bits= 1

  // This value will indicate whether this component is ready for by-wire control.
  //  0 : "NOT_READY"
  //  1 : "READY"
  // 
  uint8_t COMPONENT_READY;                     //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} COMPONENT_RPT_00_t;

// def @COMPONENT_RPT_01 CAN Message (33   0x21)
#define COMPONENT_RPT_01_IDE (0U)
#define COMPONENT_RPT_01_DLC (6U)
#define COMPONENT_RPT_01_CANID (0x21)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "PACMOD"
  //  1 : "PACMINI"
  //  2 : "PACMICRO"
  // 
  uint8_t COMPONENT_TYPE : 4;                  //      Bits= 4

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t ACCEL : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t BRAKE : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CRUISE_CONTROL_BUTTONS : 1;          //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t DASH_CONTROLS_LEFT : 1;              //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t DASH_CONTROLS_RIGHT : 1;             //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t HAZARD_LIGHTS : 1;                   //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t HEADLIGHT : 1;                       //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t HORN : 1;                            //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t MEDIA_CONTROLS : 1;                  //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t PARKING_BRAKE : 1;                   //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t SHIFT : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t SPRAYER : 1;                         //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t STEERING : 1;                        //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t TURN : 1;                            //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t WIPER : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t WATCHDOG : 1;                        //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t BRAKE_DECEL : 1;                     //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t REAR_PASS_DOOR : 1;                  //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t ENGINE_BRAKE : 1;                    //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t MARKER_LAMP : 1;                     //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CABIN_CLIMATE : 1;                   //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CABIN_FAN_SPEED : 1;                 //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CABIN_TEMP : 1;                      //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t EXHAUST_BRAKE : 1;                   //      Bits= 1

  // Counter shall have the value of 0 with the first message transmission.  It shall increase by 1 with each subsequent message transmission up to and including the value of 0xF.  The next message transmission shall be 0, and this pattern shall repeat.
  uint8_t COUNTER : 4;                         //      Bits= 4

  // The COMPLEMENT shall be the complement of the COUNTER.  For example, if COUNTER is 0x1011, then the COMPLEMENT is 0x0100.
  uint8_t COMPLEMENT : 4;                      //      Bits= 4

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t CONFIG_FAULT : 1;                    //      Bits= 1

  // This value relates to message specific or general CAN timeouts.  It includes the watchdog component report timeout.
  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t CAN_TIMEOUT_FAULT : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INTERNAL_SUPPLY_VOLTAGE_FAULT : 1;   //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t SUPERVISORY_TIMEOUT : 1;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t SUPERVISORY_SANITY_FAULT : 1;        //      Bits= 1

  // This value relates to problems with the COUNTER and COMPLIMENT signals in the component report received from the watchdog component.
  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t WATCHDOG_SANITY_FAULT : 1;           //      Bits= 1

  // This value shall be TRUE when the WATCHDOG system present signal in the component report received from the watchdog component is FALSE or any other system present signal is TRUE.
  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t WATCHDOG_SYSTEM_PRESENT_FAULT : 1;   //      Bits= 1

  // This value will indicate whether this component is ready for by-wire control.
  //  0 : "NOT_READY"
  //  1 : "READY"
  // 
  uint8_t COMPONENT_READY : 1;                 //      Bits= 1

#else

  //  0 : "PACMOD"
  //  1 : "PACMINI"
  //  2 : "PACMICRO"
  // 
  uint8_t COMPONENT_TYPE;                      //      Bits= 4

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t ACCEL;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t BRAKE;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CRUISE_CONTROL_BUTTONS;              //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t DASH_CONTROLS_LEFT;                  //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t DASH_CONTROLS_RIGHT;                 //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t HAZARD_LIGHTS;                       //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t HEADLIGHT;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t HORN;                                //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t MEDIA_CONTROLS;                      //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t PARKING_BRAKE;                       //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t SHIFT;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t SPRAYER;                             //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t STEERING;                            //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t TURN;                                //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t WIPER;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t WATCHDOG;                            //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t BRAKE_DECEL;                         //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t REAR_PASS_DOOR;                      //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t ENGINE_BRAKE;                        //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t MARKER_LAMP;                         //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CABIN_CLIMATE;                       //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CABIN_FAN_SPEED;                     //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CABIN_TEMP;                          //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t EXHAUST_BRAKE;                       //      Bits= 1

  // Counter shall have the value of 0 with the first message transmission.  It shall increase by 1 with each subsequent message transmission up to and including the value of 0xF.  The next message transmission shall be 0, and this pattern shall repeat.
  uint8_t COUNTER;                             //      Bits= 4

  // The COMPLEMENT shall be the complement of the COUNTER.  For example, if COUNTER is 0x1011, then the COMPLEMENT is 0x0100.
  uint8_t COMPLEMENT;                          //      Bits= 4

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t CONFIG_FAULT;                        //      Bits= 1

  // This value relates to message specific or general CAN timeouts.  It includes the watchdog component report timeout.
  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t CAN_TIMEOUT_FAULT;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INTERNAL_SUPPLY_VOLTAGE_FAULT;       //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t SUPERVISORY_TIMEOUT;                 //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t SUPERVISORY_SANITY_FAULT;            //      Bits= 1

  // This value relates to problems with the COUNTER and COMPLIMENT signals in the component report received from the watchdog component.
  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t WATCHDOG_SANITY_FAULT;               //      Bits= 1

  // This value shall be TRUE when the WATCHDOG system present signal in the component report received from the watchdog component is FALSE or any other system present signal is TRUE.
  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t WATCHDOG_SYSTEM_PRESENT_FAULT;       //      Bits= 1

  // This value will indicate whether this component is ready for by-wire control.
  //  0 : "NOT_READY"
  //  1 : "READY"
  // 
  uint8_t COMPONENT_READY;                     //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} COMPONENT_RPT_01_t;

// def @COMPONENT_RPT_02 CAN Message (34   0x22)
#define COMPONENT_RPT_02_IDE (0U)
#define COMPONENT_RPT_02_DLC (6U)
#define COMPONENT_RPT_02_CANID (0x22)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "PACMOD"
  //  1 : "PACMINI"
  //  2 : "PACMICRO"
  // 
  uint8_t COMPONENT_TYPE : 4;                  //      Bits= 4

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t ACCEL : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t BRAKE : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CRUISE_CONTROL_BUTTONS : 1;          //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t DASH_CONTROLS_LEFT : 1;              //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t DASH_CONTROLS_RIGHT : 1;             //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t HAZARD_LIGHTS : 1;                   //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t HEADLIGHT : 1;                       //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t HORN : 1;                            //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t MEDIA_CONTROLS : 1;                  //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t PARKING_BRAKE : 1;                   //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t SHIFT : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t SPRAYER : 1;                         //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t STEERING : 1;                        //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t TURN : 1;                            //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t WIPER : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t WATCHDOG : 1;                        //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t BRAKE_DECEL : 1;                     //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t REAR_PASS_DOOR : 1;                  //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t ENGINE_BRAKE : 1;                    //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t MARKER_LAMP : 1;                     //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CABIN_CLIMATE : 1;                   //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CABIN_FAN_SPEED : 1;                 //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CABIN_TEMP : 1;                      //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t EXHAUST_BRAKE : 1;                   //      Bits= 1

  // Counter shall have the value of 0 with the first message transmission.  It shall increase by 1 with each subsequent message transmission up to and including the value of 0xF.  The next message transmission shall be 0, and this pattern shall repeat.
  uint8_t COUNTER : 4;                         //      Bits= 4

  // The COMPLEMENT shall be the complement of the COUNTER.  For example, if COUNTER is 0x1011, then the COMPLEMENT is 0x0100.
  uint8_t COMPLEMENT : 4;                      //      Bits= 4

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t CONFIG_FAULT : 1;                    //      Bits= 1

  // This value relates to message specific or general CAN timeouts.  It includes the watchdog component report timeout.
  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t CAN_TIMEOUT_FAULT : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INTERNAL_SUPPLY_VOLTAGE_FAULT : 1;   //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t SUPERVISORY_TIMEOUT : 1;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t SUPERVISORY_SANITY_FAULT : 1;        //      Bits= 1

  // This value relates to problems with the COUNTER and COMPLIMENT signals in the component report received from the watchdog component.
  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t WATCHDOG_SANITY_FAULT : 1;           //      Bits= 1

  // This value shall be TRUE when the WATCHDOG system present signal in the component report received from the watchdog component is FALSE or any other system present signal is TRUE.
  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t WATCHDOG_SYSTEM_PRESENT_FAULT : 1;   //      Bits= 1

  // This value will indicate whether this component is ready for by-wire control.
  //  0 : "NOT_READY"
  //  1 : "READY"
  // 
  uint8_t COMPONENT_READY : 1;                 //      Bits= 1

#else

  //  0 : "PACMOD"
  //  1 : "PACMINI"
  //  2 : "PACMICRO"
  // 
  uint8_t COMPONENT_TYPE;                      //      Bits= 4

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t ACCEL;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t BRAKE;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CRUISE_CONTROL_BUTTONS;              //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t DASH_CONTROLS_LEFT;                  //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t DASH_CONTROLS_RIGHT;                 //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t HAZARD_LIGHTS;                       //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t HEADLIGHT;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t HORN;                                //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t MEDIA_CONTROLS;                      //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t PARKING_BRAKE;                       //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t SHIFT;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t SPRAYER;                             //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t STEERING;                            //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t TURN;                                //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t WIPER;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t WATCHDOG;                            //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t BRAKE_DECEL;                         //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t REAR_PASS_DOOR;                      //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t ENGINE_BRAKE;                        //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t MARKER_LAMP;                         //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CABIN_CLIMATE;                       //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CABIN_FAN_SPEED;                     //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CABIN_TEMP;                          //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t EXHAUST_BRAKE;                       //      Bits= 1

  // Counter shall have the value of 0 with the first message transmission.  It shall increase by 1 with each subsequent message transmission up to and including the value of 0xF.  The next message transmission shall be 0, and this pattern shall repeat.
  uint8_t COUNTER;                             //      Bits= 4

  // The COMPLEMENT shall be the complement of the COUNTER.  For example, if COUNTER is 0x1011, then the COMPLEMENT is 0x0100.
  uint8_t COMPLEMENT;                          //      Bits= 4

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t CONFIG_FAULT;                        //      Bits= 1

  // This value relates to message specific or general CAN timeouts.  It includes the watchdog component report timeout.
  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t CAN_TIMEOUT_FAULT;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INTERNAL_SUPPLY_VOLTAGE_FAULT;       //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t SUPERVISORY_TIMEOUT;                 //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t SUPERVISORY_SANITY_FAULT;            //      Bits= 1

  // This value relates to problems with the COUNTER and COMPLIMENT signals in the component report received from the watchdog component.
  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t WATCHDOG_SANITY_FAULT;               //      Bits= 1

  // This value shall be TRUE when the WATCHDOG system present signal in the component report received from the watchdog component is FALSE or any other system present signal is TRUE.
  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t WATCHDOG_SYSTEM_PRESENT_FAULT;       //      Bits= 1

  // This value will indicate whether this component is ready for by-wire control.
  //  0 : "NOT_READY"
  //  1 : "READY"
  // 
  uint8_t COMPONENT_READY;                     //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} COMPONENT_RPT_02_t;

// def @COMPONENT_RPT_03 CAN Message (35   0x23)
#define COMPONENT_RPT_03_IDE (0U)
#define COMPONENT_RPT_03_DLC (6U)
#define COMPONENT_RPT_03_CANID (0x23)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "PACMOD"
  //  1 : "PACMINI"
  //  2 : "PACMICRO"
  // 
  uint8_t COMPONENT_TYPE : 4;                  //      Bits= 4

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t ACCEL : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t BRAKE : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CRUISE_CONTROL_BUTTONS : 1;          //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t DASH_CONTROLS_LEFT : 1;              //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t DASH_CONTROLS_RIGHT : 1;             //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t HAZARD_LIGHTS : 1;                   //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t HEADLIGHT : 1;                       //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t HORN : 1;                            //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t MEDIA_CONTROLS : 1;                  //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t PARKING_BRAKE : 1;                   //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t SHIFT : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t SPRAYER : 1;                         //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t STEERING : 1;                        //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t TURN : 1;                            //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t WIPER : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t WATCHDOG : 1;                        //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t BRAKE_DECEL : 1;                     //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t REAR_PASS_DOOR : 1;                  //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t ENGINE_BRAKE : 1;                    //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t MARKER_LAMP : 1;                     //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CABIN_CLIMATE : 1;                   //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CABIN_FAN_SPEED : 1;                 //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CABIN_TEMP : 1;                      //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t EXHAUST_BRAKE : 1;                   //      Bits= 1

  // Counter shall have the value of 0 with the first message transmission.  It shall increase by 1 with each subsequent message transmission up to and including the value of 0xF.  The next message transmission shall be 0, and this pattern shall repeat.
  uint8_t COUNTER : 4;                         //      Bits= 4

  // The COMPLEMENT shall be the complement of the COUNTER.  For example, if COUNTER is 0x1011, then the COMPLEMENT is 0x0100.
  uint8_t COMPLEMENT : 4;                      //      Bits= 4

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t CONFIG_FAULT : 1;                    //      Bits= 1

  // This value relates to message specific or general CAN timeouts.  It includes the watchdog component report timeout.
  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t CAN_TIMEOUT_FAULT : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INTERNAL_SUPPLY_VOLTAGE_FAULT : 1;   //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t SUPERVISORY_TIMEOUT : 1;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t SUPERVISORY_SANITY_FAULT : 1;        //      Bits= 1

  // This value relates to problems with the COUNTER and COMPLIMENT signals in the component report received from the watchdog component.
  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t WATCHDOG_SANITY_FAULT : 1;           //      Bits= 1

  // This value shall be TRUE when the WATCHDOG system present signal in the component report received from the watchdog component is FALSE or any other system present signal is TRUE.
  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t WATCHDOG_SYSTEM_PRESENT_FAULT : 1;   //      Bits= 1

  // This value will indicate whether this component is ready for by-wire control.
  //  0 : "NOT_READY"
  //  1 : "READY"
  // 
  uint8_t COMPONENT_READY : 1;                 //      Bits= 1

#else

  //  0 : "PACMOD"
  //  1 : "PACMINI"
  //  2 : "PACMICRO"
  // 
  uint8_t COMPONENT_TYPE;                      //      Bits= 4

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t ACCEL;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t BRAKE;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CRUISE_CONTROL_BUTTONS;              //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t DASH_CONTROLS_LEFT;                  //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t DASH_CONTROLS_RIGHT;                 //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t HAZARD_LIGHTS;                       //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t HEADLIGHT;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t HORN;                                //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t MEDIA_CONTROLS;                      //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t PARKING_BRAKE;                       //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t SHIFT;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t SPRAYER;                             //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t STEERING;                            //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t TURN;                                //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t WIPER;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t WATCHDOG;                            //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t BRAKE_DECEL;                         //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t REAR_PASS_DOOR;                      //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t ENGINE_BRAKE;                        //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t MARKER_LAMP;                         //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CABIN_CLIMATE;                       //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CABIN_FAN_SPEED;                     //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CABIN_TEMP;                          //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t EXHAUST_BRAKE;                       //      Bits= 1

  // Counter shall have the value of 0 with the first message transmission.  It shall increase by 1 with each subsequent message transmission up to and including the value of 0xF.  The next message transmission shall be 0, and this pattern shall repeat.
  uint8_t COUNTER;                             //      Bits= 4

  // The COMPLEMENT shall be the complement of the COUNTER.  For example, if COUNTER is 0x1011, then the COMPLEMENT is 0x0100.
  uint8_t COMPLEMENT;                          //      Bits= 4

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t CONFIG_FAULT;                        //      Bits= 1

  // This value relates to message specific or general CAN timeouts.  It includes the watchdog component report timeout.
  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t CAN_TIMEOUT_FAULT;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INTERNAL_SUPPLY_VOLTAGE_FAULT;       //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t SUPERVISORY_TIMEOUT;                 //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t SUPERVISORY_SANITY_FAULT;            //      Bits= 1

  // This value relates to problems with the COUNTER and COMPLIMENT signals in the component report received from the watchdog component.
  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t WATCHDOG_SANITY_FAULT;               //      Bits= 1

  // This value shall be TRUE when the WATCHDOG system present signal in the component report received from the watchdog component is FALSE or any other system present signal is TRUE.
  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t WATCHDOG_SYSTEM_PRESENT_FAULT;       //      Bits= 1

  // This value will indicate whether this component is ready for by-wire control.
  //  0 : "NOT_READY"
  //  1 : "READY"
  // 
  uint8_t COMPONENT_READY;                     //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} COMPONENT_RPT_03_t;

// def @COMPONENT_RPT_04 CAN Message (36   0x24)
#define COMPONENT_RPT_04_IDE (0U)
#define COMPONENT_RPT_04_DLC (6U)
#define COMPONENT_RPT_04_CANID (0x24)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "PACMOD"
  //  1 : "PACMINI"
  //  2 : "PACMICRO"
  // 
  uint8_t COMPONENT_TYPE : 4;                  //      Bits= 4

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t ACCEL : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t BRAKE : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CRUISE_CONTROL_BUTTONS : 1;          //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t DASH_CONTROLS_LEFT : 1;              //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t DASH_CONTROLS_RIGHT : 1;             //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t HAZARD_LIGHTS : 1;                   //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t HEADLIGHT : 1;                       //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t HORN : 1;                            //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t MEDIA_CONTROLS : 1;                  //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t PARKING_BRAKE : 1;                   //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t SHIFT : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t SPRAYER : 1;                         //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t STEERING : 1;                        //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t TURN : 1;                            //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t WIPER : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t WATCHDOG : 1;                        //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t BRAKE_DECEL : 1;                     //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t REAR_PASS_DOOR : 1;                  //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t ENGINE_BRAKE : 1;                    //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t MARKER_LAMP : 1;                     //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CABIN_CLIMATE : 1;                   //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CABIN_FAN_SPEED : 1;                 //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CABIN_TEMP : 1;                      //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t EXHAUST_BRAKE : 1;                   //      Bits= 1

  // Counter shall have the value of 0 with the first message transmission.  It shall increase by 1 with each subsequent message transmission up to and including the value of 0xF.  The next message transmission shall be 0, and this pattern shall repeat.
  uint8_t COUNTER : 4;                         //      Bits= 4

  // The COMPLEMENT shall be the complement of the COUNTER.  For example, if COUNTER is 0x1011, then the COMPLEMENT is 0x0100.
  uint8_t COMPLEMENT : 4;                      //      Bits= 4

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t CONFIG_FAULT : 1;                    //      Bits= 1

  // This value relates to message specific or general CAN timeouts.  It includes the watchdog component report timeout.
  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t CAN_TIMEOUT_FAULT : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INTERNAL_SUPPLY_VOLTAGE_FAULT : 1;   //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t SUPERVISORY_TIMEOUT : 1;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t SUPERVISORY_SANITY_FAULT : 1;        //      Bits= 1

  // This value relates to problems with the COUNTER and COMPLIMENT signals in the component report received from the watchdog component.
  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t WATCHDOG_SANITY_FAULT : 1;           //      Bits= 1

  // This value shall be TRUE when the WATCHDOG system present signal in the component report received from the watchdog component is FALSE or any other system present signal is TRUE.
  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t WATCHDOG_SYSTEM_PRESENT_FAULT : 1;   //      Bits= 1

  // This value will indicate whether this component is ready for by-wire control.
  //  0 : "NOT_READY"
  //  1 : "READY"
  // 
  uint8_t COMPONENT_READY : 1;                 //      Bits= 1

#else

  //  0 : "PACMOD"
  //  1 : "PACMINI"
  //  2 : "PACMICRO"
  // 
  uint8_t COMPONENT_TYPE;                      //      Bits= 4

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t ACCEL;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t BRAKE;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CRUISE_CONTROL_BUTTONS;              //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t DASH_CONTROLS_LEFT;                  //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t DASH_CONTROLS_RIGHT;                 //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t HAZARD_LIGHTS;                       //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t HEADLIGHT;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t HORN;                                //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t MEDIA_CONTROLS;                      //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t PARKING_BRAKE;                       //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t SHIFT;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t SPRAYER;                             //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t STEERING;                            //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t TURN;                                //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t WIPER;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t WATCHDOG;                            //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t BRAKE_DECEL;                         //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t REAR_PASS_DOOR;                      //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t ENGINE_BRAKE;                        //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t MARKER_LAMP;                         //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CABIN_CLIMATE;                       //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CABIN_FAN_SPEED;                     //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t CABIN_TEMP;                          //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  // 
  uint8_t EXHAUST_BRAKE;                       //      Bits= 1

  // Counter shall have the value of 0 with the first message transmission.  It shall increase by 1 with each subsequent message transmission up to and including the value of 0xF.  The next message transmission shall be 0, and this pattern shall repeat.
  uint8_t COUNTER;                             //      Bits= 4

  // The COMPLEMENT shall be the complement of the COUNTER.  For example, if COUNTER is 0x1011, then the COMPLEMENT is 0x0100.
  uint8_t COMPLEMENT;                          //      Bits= 4

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t CONFIG_FAULT;                        //      Bits= 1

  // This value relates to message specific or general CAN timeouts.  It includes the watchdog component report timeout.
  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t CAN_TIMEOUT_FAULT;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INTERNAL_SUPPLY_VOLTAGE_FAULT;       //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t SUPERVISORY_TIMEOUT;                 //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t SUPERVISORY_SANITY_FAULT;            //      Bits= 1

  // This value relates to problems with the COUNTER and COMPLIMENT signals in the component report received from the watchdog component.
  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t WATCHDOG_SANITY_FAULT;               //      Bits= 1

  // This value shall be TRUE when the WATCHDOG system present signal in the component report received from the watchdog component is FALSE or any other system present signal is TRUE.
  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t WATCHDOG_SYSTEM_PRESENT_FAULT;       //      Bits= 1

  // This value will indicate whether this component is ready for by-wire control.
  //  0 : "NOT_READY"
  //  1 : "READY"
  // 
  uint8_t COMPONENT_READY;                     //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} COMPONENT_RPT_04_t;

// def @SAFETY_FUNC_RPT CAN Message (64   0x40)
#define SAFETY_FUNC_RPT_IDE (0U)
#define SAFETY_FUNC_RPT_DLC (4U)
#define SAFETY_FUNC_RPT_CANID (0x40)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "NONE"
  //  1 : "AUTOMS_READY"
  //  2 : "AUTOMS"
  //  3 : "MANUAL_READY"
  //  4 : "CRITICAL_STOP1"
  //  5 : "CRITICAL_STOP2"
  // 
  uint8_t COMMANDED_VALUE : 4;                   //      Bits= 4

  //  0 : "MANUAL_BRAKED"
  //  1 : "MANUAL_UNBRAKED"
  //  2 : "AUTOMS_READY"
  //  3 : "AUTOMS_INACTIVE"
  //  4 : "AUTOMS_ACTIVE_BRAKED"
  //  5 : "AUTOMS_ACTIVE_UNBRAKED"
  //  6 : "MANUAL_READY"
  //  7 : "CRITICAL_STOP1"
  //  8 : "CRITICAL_STOP2"
  //  9 : "STARTUP"
  // 
  uint8_t STATE : 4;                             //      Bits= 4

  //  0 : "INVALID"
  //  1 : "MANUAL"
  //  2 : "AUTOMS"
  // 
  uint8_t AUTOMSMAN_OPCTRL : 2;                  //      Bits= 2

  //  0 : "INVALID"
  //  1 : "APPLIED"
  //  2 : "UNAPPLIED"
  // 
  uint8_t CABIN_SAFETY_BRAKE_OPCTRL : 2;         //      Bits= 2

  //  0 : "INVALID"
  //  1 : "GO"
  //  2 : "STOP"
  // 
  uint8_t REMOTE_STOP_STATUS : 2;                //      Bits= 2

  //  0 : "NOT_RUNNING"
  //  1 : "RUNNING"
  // 
  uint8_t ENGINE_STATUS : 1;                     //      Bits= 1

  //  0 : "DISABLED"
  //  1 : "ENABLED"
  // 
  uint8_t PACMOD_SYSTEM_STATUS : 1;              //      Bits= 1

  //  0 : "OKAY"
  //  1 : "FAULT"
  //  2 : "TIMEOUT"
  // 
  uint8_t USER_PC_FAULT : 2;                     //      Bits= 2

  //  0 : "OKAY"
  //  1 : "FAULT"
  //  2 : "TIMEOUT"
  // 
  uint8_t PACMOD_SYSTEM_FAULT : 2;               //      Bits= 2

  //  0 : "NOT_OBTAINABLE"
  //  1 : "OBTAINABLE"
  // 
  uint8_t MANUAL_STATE_OBTAINABLE : 1;           //      Bits= 1

  //  0 : "NOT_OBTAINABLE"
  //  1 : "OBTAINABLE"
  // 
  uint8_t AUTOMS_READY_STATE_OBTAINABLE : 1;     //      Bits= 1

  //  0 : "NOT_OBTAINABLE"
  //  1 : "OBTAINABLE"
  // 
  uint8_t AUTOMS_STATE_OBTAINABLE : 1;           //      Bits= 1

  //  0 : "NOT_OBTAINABLE"
  //  1 : "OBTAINABLE"
  // 
  uint8_t MANUAL_READY_STATE_OBTAINABLE : 1;     //      Bits= 1

  //  0 : "NOT_OBTAINABLE"
  //  1 : "OBTAINABLE"
  // 
  uint8_t CRITICAL_STOP1_STATE_OBTAINABLE : 1;   //      Bits= 1

  //  0 : "NOT_OBTAINABLE"
  //  1 : "OBTAINABLE"
  // 
  uint8_t CRITICAL_STOP2_STATE_OBTAINABLE : 1;   //      Bits= 1

#else

  //  0 : "NONE"
  //  1 : "AUTOMS_READY"
  //  2 : "AUTOMS"
  //  3 : "MANUAL_READY"
  //  4 : "CRITICAL_STOP1"
  //  5 : "CRITICAL_STOP2"
  // 
  uint8_t COMMANDED_VALUE;                       //      Bits= 4

  //  0 : "MANUAL_BRAKED"
  //  1 : "MANUAL_UNBRAKED"
  //  2 : "AUTOMS_READY"
  //  3 : "AUTOMS_INACTIVE"
  //  4 : "AUTOMS_ACTIVE_BRAKED"
  //  5 : "AUTOMS_ACTIVE_UNBRAKED"
  //  6 : "MANUAL_READY"
  //  7 : "CRITICAL_STOP1"
  //  8 : "CRITICAL_STOP2"
  //  9 : "STARTUP"
  // 
  uint8_t STATE;                                 //      Bits= 4

  //  0 : "INVALID"
  //  1 : "MANUAL"
  //  2 : "AUTOMS"
  // 
  uint8_t AUTOMSMAN_OPCTRL;                      //      Bits= 2

  //  0 : "INVALID"
  //  1 : "APPLIED"
  //  2 : "UNAPPLIED"
  // 
  uint8_t CABIN_SAFETY_BRAKE_OPCTRL;             //      Bits= 2

  //  0 : "INVALID"
  //  1 : "GO"
  //  2 : "STOP"
  // 
  uint8_t REMOTE_STOP_STATUS;                    //      Bits= 2

  //  0 : "NOT_RUNNING"
  //  1 : "RUNNING"
  // 
  uint8_t ENGINE_STATUS;                         //      Bits= 1

  //  0 : "DISABLED"
  //  1 : "ENABLED"
  // 
  uint8_t PACMOD_SYSTEM_STATUS;                  //      Bits= 1

  //  0 : "OKAY"
  //  1 : "FAULT"
  //  2 : "TIMEOUT"
  // 
  uint8_t USER_PC_FAULT;                         //      Bits= 2

  //  0 : "OKAY"
  //  1 : "FAULT"
  //  2 : "TIMEOUT"
  // 
  uint8_t PACMOD_SYSTEM_FAULT;                   //      Bits= 2

  //  0 : "NOT_OBTAINABLE"
  //  1 : "OBTAINABLE"
  // 
  uint8_t MANUAL_STATE_OBTAINABLE;               //      Bits= 1

  //  0 : "NOT_OBTAINABLE"
  //  1 : "OBTAINABLE"
  // 
  uint8_t AUTOMS_READY_STATE_OBTAINABLE;         //      Bits= 1

  //  0 : "NOT_OBTAINABLE"
  //  1 : "OBTAINABLE"
  // 
  uint8_t AUTOMS_STATE_OBTAINABLE;               //      Bits= 1

  //  0 : "NOT_OBTAINABLE"
  //  1 : "OBTAINABLE"
  // 
  uint8_t MANUAL_READY_STATE_OBTAINABLE;         //      Bits= 1

  //  0 : "NOT_OBTAINABLE"
  //  1 : "OBTAINABLE"
  // 
  uint8_t CRITICAL_STOP1_STATE_OBTAINABLE;       //      Bits= 1

  //  0 : "NOT_OBTAINABLE"
  //  1 : "OBTAINABLE"
  // 
  uint8_t CRITICAL_STOP2_STATE_OBTAINABLE;       //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} SAFETY_FUNC_RPT_t;

// def @SAFETY_BRAKE_RPT CAN Message (65   0x41)
#define SAFETY_BRAKE_RPT_IDE (0U)
#define SAFETY_BRAKE_RPT_DLC (1U)
#define SAFETY_BRAKE_RPT_CANID (0x41)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "APPLY_BRAKE"
  //  1 : "RELEASE_BRAKE"
  // 
  uint8_t COMMANDED_VALUE : 1;               //      Bits= 1

  //  0 : "BRAKE_APPLIED"
  //  1 : "BRAKE_RELEASED"
  //  2 : "BETWEEN"
  //  3 : "OUT_OF_RANGE"
  // 
  uint8_t OUTPUT_VALUE : 2;                  //      Bits= 2

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_REPORTED_FAULT : 1;        //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

  //  0 : "COMMAND_NOT_PERMITTED"
  //  1 : "COMMAND_PERMITTED"
  // 
  uint8_t COMMAND_PERMITTED : 1;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t REPORTED_FAULT : 1;                //      Bits= 1

#else

  //  0 : "APPLY_BRAKE"
  //  1 : "RELEASE_BRAKE"
  // 
  uint8_t COMMANDED_VALUE;                   //      Bits= 1

  //  0 : "BRAKE_APPLIED"
  //  1 : "BRAKE_RELEASED"
  //  2 : "BETWEEN"
  //  3 : "OUT_OF_RANGE"
  // 
  uint8_t OUTPUT_VALUE;                      //      Bits= 2

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_REPORTED_FAULT;            //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

  //  0 : "COMMAND_NOT_PERMITTED"
  //  1 : "COMMAND_PERMITTED"
  // 
  uint8_t COMMAND_PERMITTED;                 //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t REPORTED_FAULT;                    //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} SAFETY_BRAKE_RPT_t;

// def @VEHICLE_FAULT_RPT CAN Message (66   0x42)
#define VEHICLE_FAULT_RPT_IDE (0U)
#define VEHICLE_FAULT_RPT_DLC (4U)
#define VEHICLE_FAULT_RPT_CANID (0x42)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENGINE_CHECK_LIGHT : 1;                 //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t ENGINE_CHECK_LIGHT_AVAIL : 1;           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t TRC_FAULT_LIGHT : 1;                    //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t TRC_FAULT_LIGHT_AVAIL : 1;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t TRC_OFF_FAULT_LIGHT : 1;                //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t TRC_OFF_FAULT_LIGHT_AVAIL : 1;          //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ANTILOCK_BRAKE_FAULT_LIGHT : 1;         //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t ANTILOCK_BRAKE_FAULT_LIGHT_AVAIL : 1;   //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t TIRE_FAULT_LIGHT : 1;                   //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t TIRE_FAULT_LIGHT_AVAIL : 1;             //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t AIR_BAGS_FAULT_LIGHT : 1;               //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t AIR_BAGS_FAULT_LIGHT_AVAIL : 1;         //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t LOW_ENGINE_OIL_PRESSURE : 1;            //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t LOW_ENGINE_OIL_PRESSURE_AVAIL : 1;      //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t BRAKE_FAULT : 1;                        //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t BRAKE_FAULT_AVAIL : 1;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t BRK_APPLIED_POWER_REDUCED : 1;          //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t BRK_APPLIED_POWER_REDUCED_AVAIL : 1;    //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t STEERING_LOSS_STOP_SAFELY : 1;          //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t STEERING_LOSS_STOP_SAFELY_AVAIL : 1;    //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t STEERING_FAULT_SERVICE_NOW : 1;         //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t STEERING_FAULT_SERVICE_NOW_AVAIL : 1;   //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t XMSN_FAULT_SERVICE_NOW : 1;             //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t XMSN_FAULT_SERVICE_NOW_AVAIL : 1;       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t XMSN_OVER_TEMP_STOP_SAFELY : 1;         //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t XMSN_OVER_TEMP_STOP_SAFELY_AVAIL : 1;   //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t LOW_BATTERY_FEATURES_OFF : 1;           //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t LOW_BATTERY_FEATURES_OFF_AVAIL : 1;     //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CHARGING_SYSTEM_FAULT : 1;              //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t CHARGING_SYSTEM_FAULT_AVAIL : 1;        //      Bits= 1

#else

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENGINE_CHECK_LIGHT;                     //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t ENGINE_CHECK_LIGHT_AVAIL;               //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t TRC_FAULT_LIGHT;                        //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t TRC_FAULT_LIGHT_AVAIL;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t TRC_OFF_FAULT_LIGHT;                    //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t TRC_OFF_FAULT_LIGHT_AVAIL;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ANTILOCK_BRAKE_FAULT_LIGHT;             //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t ANTILOCK_BRAKE_FAULT_LIGHT_AVAIL;       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t TIRE_FAULT_LIGHT;                       //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t TIRE_FAULT_LIGHT_AVAIL;                 //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t AIR_BAGS_FAULT_LIGHT;                   //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t AIR_BAGS_FAULT_LIGHT_AVAIL;             //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t LOW_ENGINE_OIL_PRESSURE;                //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t LOW_ENGINE_OIL_PRESSURE_AVAIL;          //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t BRAKE_FAULT;                            //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t BRAKE_FAULT_AVAIL;                      //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t BRK_APPLIED_POWER_REDUCED;              //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t BRK_APPLIED_POWER_REDUCED_AVAIL;        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t STEERING_LOSS_STOP_SAFELY;              //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t STEERING_LOSS_STOP_SAFELY_AVAIL;        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t STEERING_FAULT_SERVICE_NOW;             //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t STEERING_FAULT_SERVICE_NOW_AVAIL;       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t XMSN_FAULT_SERVICE_NOW;                 //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t XMSN_FAULT_SERVICE_NOW_AVAIL;           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t XMSN_OVER_TEMP_STOP_SAFELY;             //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t XMSN_OVER_TEMP_STOP_SAFELY_AVAIL;       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t LOW_BATTERY_FEATURES_OFF;               //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t LOW_BATTERY_FEATURES_OFF_AVAIL;         //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CHARGING_SYSTEM_FAULT;                  //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t CHARGING_SYSTEM_FAULT_AVAIL;            //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} VEHICLE_FAULT_RPT_t;

// def @GLOBAL_CMD CAN Message (128  0x80)
#define GLOBAL_CMD_IDE (0U)
#define GLOBAL_CMD_DLC (2U)
#define GLOBAL_CMD_CANID (0x80)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_FAULTS : 1;                  //      Bits= 1

  // When this value is REQUIRED, the COUNTER and COMPLEMENT values must be set to enable the PACMod System.
  //  0 : "NOT_REQUIRED"
  //  1 : "REQUIRED"
  // 
  uint8_t SANITY_CHECK_REQUIRED : 1;         //      Bits= 1

  // Counter shall have the value of 0 with the first message transmission.  It shall increase by 1 with each subsequent message transmission up to and including the value of 0xF.  The next message transmission shall be 0, and this pattern shall repeat.
  uint8_t COUNTER : 4;                       //      Bits= 4

  // The COMPLEMENT shall be the complement of the COUNTER.  For example, if COUNTER is 0x1011, then the COMPLEMENT is 0x0100.
  uint8_t COMPLEMENT : 4;                    //      Bits= 4

#else

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_FAULTS;                      //      Bits= 1

  // When this value is REQUIRED, the COUNTER and COMPLEMENT values must be set to enable the PACMod System.
  //  0 : "NOT_REQUIRED"
  //  1 : "REQUIRED"
  // 
  uint8_t SANITY_CHECK_REQUIRED;             //      Bits= 1

  // Counter shall have the value of 0 with the first message transmission.  It shall increase by 1 with each subsequent message transmission up to and including the value of 0xF.  The next message transmission shall be 0, and this pattern shall repeat.
  uint8_t COUNTER;                           //      Bits= 4

  // The COMPLEMENT shall be the complement of the COUNTER.  For example, if COUNTER is 0x1011, then the COMPLEMENT is 0x0100.
  uint8_t COMPLEMENT;                        //      Bits= 4

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} GLOBAL_CMD_t;

// def @SUPERVISORY_CTRL CAN Message (129  0x81)
#define SUPERVISORY_CTRL_IDE (0U)
#define SUPERVISORY_CTRL_DLC (2U)
#define SUPERVISORY_CTRL_CANID (0x81)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  // A system shall enable only if the supervisory enablement rules are met.  See user_can_protocol.md.
  //  0 : "DISABLE_ALL_SYSTEMS"
  //  1 : "ALLOW_ENABLE"
  // 
  uint8_t ENABLE : 1;                        //      Bits= 1

  // Counter shall have the value of 0 with the first message transmission.  It shall increase by 1 with each subsequent message transmission up to and including the value of 0xF.  The next message transmission shall be 0, and this pattern shall repeat.
  uint8_t COUNTER : 4;                       //      Bits= 4

  // The COMPLEMENT shall be the complement of the COUNTER.  For example, if COUNTER is 0x1011, then the COMPLEMENT is 0x0100.
  uint8_t COMPLEMENT : 4;                    //      Bits= 4

#else

  // A system shall enable only if the supervisory enablement rules are met.  See user_can_protocol.md.
  //  0 : "DISABLE_ALL_SYSTEMS"
  //  1 : "ALLOW_ENABLE"
  // 
  uint8_t ENABLE;                            //      Bits= 1

  // Counter shall have the value of 0 with the first message transmission.  It shall increase by 1 with each subsequent message transmission up to and including the value of 0xF.  The next message transmission shall be 0, and this pattern shall repeat.
  uint8_t COUNTER;                           //      Bits= 4

  // The COMPLEMENT shall be the complement of the COUNTER.  For example, if COUNTER is 0x1011, then the COMPLEMENT is 0x0100.
  uint8_t COMPLEMENT;                        //      Bits= 4

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} SUPERVISORY_CTRL_t;

// def @SAFETY_FUNC_CMD CAN Message (192  0xc0)
#define SAFETY_FUNC_CMD_IDE (0U)
#define SAFETY_FUNC_CMD_DLC (1U)
#define SAFETY_FUNC_CMD_CANID (0xc0)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "NONE"
  //  1 : "AUTOMS_READY"
  //  2 : "AUTOMS"
  //  3 : "MANUAL_READY"
  //  4 : "CRITICAL_STOP1"
  //  5 : "CRITICAL_STOP2"
  // 
  uint8_t COMMAND : 4;                       //      Bits= 4

#else

  //  0 : "NONE"
  //  1 : "AUTOMS_READY"
  //  2 : "AUTOMS"
  //  3 : "MANUAL_READY"
  //  4 : "CRITICAL_STOP1"
  //  5 : "CRITICAL_STOP2"
  // 
  uint8_t COMMAND;                           //      Bits= 4

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} SAFETY_FUNC_CMD_t;

// def @SAFETY_BRAKE_CMD CAN Message (193  0xc1)
#define SAFETY_BRAKE_CMD_IDE (0U)
#define SAFETY_BRAKE_CMD_DLC (1U)
#define SAFETY_BRAKE_CMD_CANID (0xc1)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "APPLY_BRAKE"
  //  1 : "RELEASE_BRAKE"
  // 
  uint8_t SAFETY_BRAKE_CMD : 1;              //      Bits= 1

#else

  //  0 : "APPLY_BRAKE"
  //  1 : "RELEASE_BRAKE"
  // 
  uint8_t SAFETY_BRAKE_CMD;                  //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} SAFETY_BRAKE_CMD_t;

// def @ACCEL_CMD CAN Message (256  0x100)
#define ACCEL_CMD_IDE (0U)
#define ACCEL_CMD_DLC (3U)
#define ACCEL_CMD_CANID (0x100)
// signal: @ACCEL_CMD_ro
#define PACMOD12_ACCEL_CMD_ro_CovFactor (0.001000)
#define PACMOD12_ACCEL_CMD_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_ACCEL_CMD_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE : 1;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  uint16_t ACCEL_CMD_ro;                     //      Bits=16 Factor= 0.001000        Unit:'%'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t ACCEL_CMD_phys;
#endif // PACMOD12_USE_SIGFLOAT

#else

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE;                            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  uint16_t ACCEL_CMD_ro;                     //      Bits=16 Factor= 0.001000        Unit:'%'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t ACCEL_CMD_phys;
#endif // PACMOD12_USE_SIGFLOAT

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} ACCEL_CMD_t;

// def @BRAKE_CMD CAN Message (260  0x104)
#define BRAKE_CMD_IDE (0U)
#define BRAKE_CMD_DLC (3U)
#define BRAKE_CMD_CANID (0x104)
// signal: @BRAKE_CMD_ro
#define PACMOD12_BRAKE_CMD_ro_CovFactor (0.001000)
#define PACMOD12_BRAKE_CMD_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_BRAKE_CMD_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE : 1;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  uint16_t BRAKE_CMD_ro;                     //      Bits=16 Factor= 0.001000        Unit:'%'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t BRAKE_CMD_phys;
#endif // PACMOD12_USE_SIGFLOAT

#else

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE;                            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  uint16_t BRAKE_CMD_ro;                     //      Bits=16 Factor= 0.001000        Unit:'%'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t BRAKE_CMD_phys;
#endif // PACMOD12_USE_SIGFLOAT

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} BRAKE_CMD_t;

// def @CRUISE_CONTROL_BUTTONS_CMD CAN Message (264  0x108)
#define CRUISE_CONTROL_BUTTONS_CMD_IDE (0U)
#define CRUISE_CONTROL_BUTTONS_CMD_DLC (2U)
#define CRUISE_CONTROL_BUTTONS_CMD_CANID (0x108)
#define CRUISE_CONTROL_BUTTONS_CMD_CYC (100U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE : 1;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  // NONE = no button pressed, CNCL = Cancel, FURTHER* or CLOSER* = Change follow distance, SET_DEC* = set/decrement, RES_INC* = resume/increment, ON_OFF = Cruise On/Off, LIM* = Speed limiter On/Off. *Exact function varies across vehicles - See platform guide.
  //  7 : "CRUISE_CONTROL_LIM"
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

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE;                            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  // NONE = no button pressed, CNCL = Cancel, FURTHER* or CLOSER* = Change follow distance, SET_DEC* = set/decrement, RES_INC* = resume/increment, ON_OFF = Cruise On/Off, LIM* = Speed limiter On/Off. *Exact function varies across vehicles - See platform guide.
  //  7 : "CRUISE_CONTROL_LIM"
  //  6 : "CRUISE_CONTROL_ON_OFF"
  //  5 : "CRUISE_CONTROL_RES_INC"
  //  4 : "CRUISE_CONTROL_SET_DEC"
  //  3 : "CRUISE_CONTROL_ACC_CLOSER"
  //  2 : "CRUISE_CONTROL_ACC_FURTHER"
  //  1 : "CRUISE_CONTROL_CNCL"
  //  0 : "CRUISE_CONTROL_NONE"
  // 
  uint8_t CRUISE_CONTROL_BUTTON;             //      Bits= 8

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} CRUISE_CONTROL_BUTTONS_CMD_t;

// def @DASH_CONTROLS_LEFT_CMD CAN Message (268  0x10c)
#define DASH_CONTROLS_LEFT_CMD_IDE (0U)
#define DASH_CONTROLS_LEFT_CMD_DLC (2U)
#define DASH_CONTROLS_LEFT_CMD_CANID (0x10c)
#define DASH_CONTROLS_LEFT_CMD_CYC (100U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE : 1;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  // 
  uint8_t DASH_CONTROLS_BUTTON;              //      Bits= 8

#else

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE;                            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  // 
  uint8_t DASH_CONTROLS_BUTTON;              //      Bits= 8

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} DASH_CONTROLS_LEFT_CMD_t;

// def @DASH_CONTROLS_RIGHT_CMD CAN Message (272  0x110)
#define DASH_CONTROLS_RIGHT_CMD_IDE (0U)
#define DASH_CONTROLS_RIGHT_CMD_DLC (2U)
#define DASH_CONTROLS_RIGHT_CMD_CANID (0x110)
#define DASH_CONTROLS_RIGHT_CMD_CYC (100U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE : 1;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  // 
  uint8_t DASH_CONTROLS_BUTTON;              //      Bits= 8

#else

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE;                            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  // 
  uint8_t DASH_CONTROLS_BUTTON;              //      Bits= 8

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} DASH_CONTROLS_RIGHT_CMD_t;

// def @HAZARD_LIGHTS_CMD CAN Message (276  0x114)
#define HAZARD_LIGHTS_CMD_IDE (0U)
#define HAZARD_LIGHTS_CMD_DLC (2U)
#define HAZARD_LIGHTS_CMD_CANID (0x114)
#define HAZARD_LIGHTS_CMD_CYC (100U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE : 1;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t HAZARD_LIGHTS_CMD : 1;             //      Bits= 1

#else

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE;                            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t HAZARD_LIGHTS_CMD;                 //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} HAZARD_LIGHTS_CMD_t;

// def @HEADLIGHT_CMD CAN Message (280  0x118)
#define HEADLIGHT_CMD_IDE (0U)
#define HEADLIGHT_CMD_DLC (2U)
#define HEADLIGHT_CMD_CANID (0x118)
#define HEADLIGHT_CMD_CYC (100U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE : 1;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  //  2 : "HIGH_BEAMS"
  //  1 : "LOW_BEAMS"
  //  0 : "HEADLIGHTS_OFF"
  // 
  uint8_t HEADLIGHT_CMD;                     //      Bits= 8

#else

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE;                            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  //  2 : "HIGH_BEAMS"
  //  1 : "LOW_BEAMS"
  //  0 : "HEADLIGHTS_OFF"
  // 
  uint8_t HEADLIGHT_CMD;                     //      Bits= 8

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} HEADLIGHT_CMD_t;

// def @HORN_CMD CAN Message (284  0x11c)
#define HORN_CMD_IDE (0U)
#define HORN_CMD_DLC (2U)
#define HORN_CMD_CANID (0x11c)
#define HORN_CMD_CYC (100U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE : 1;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t HORN_CMD : 1;                      //      Bits= 1

#else

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE;                            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t HORN_CMD;                          //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} HORN_CMD_t;

// def @MEDIA_CONTROLS_CMD CAN Message (288  0x120)
#define MEDIA_CONTROLS_CMD_IDE (0U)
#define MEDIA_CONTROLS_CMD_DLC (2U)
#define MEDIA_CONTROLS_CMD_CANID (0x120)
#define MEDIA_CONTROLS_CMD_CYC (100U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE : 1;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

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

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE;                            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  //  6 : "MEDIA_CONTROL_VOL_DOWN"
  //  5 : "MEDIA_CONTROL_VOL_UP"
  //  4 : "MEDIA_CONTROL_NEXT_TRACK_HANG_UP"
  //  3 : "MEDIA_CONTROL_PREV_TRACK_ANSWER"
  //  2 : "MEDIA_CONTROL_MUTE"
  //  1 : "MEDIA_CONTROL_VOICE_COMMAND"
  //  0 : "MEDIA_CONTROL_NONE"
  // 
  uint8_t MEDIA_CONTROLS_CMD;                //      Bits= 8

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} MEDIA_CONTROLS_CMD_t;

// def @PARKING_BRAKE_CMD CAN Message (292  0x124)
#define PARKING_BRAKE_CMD_IDE (0U)
#define PARKING_BRAKE_CMD_DLC (2U)
#define PARKING_BRAKE_CMD_CANID (0x124)
#define PARKING_BRAKE_CMD_CYC (100U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE : 1;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t PARKING_BRAKE_CMD : 1;             //      Bits= 1

#else

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE;                            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t PARKING_BRAKE_CMD;                 //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} PARKING_BRAKE_CMD_t;

// def @SHIFT_CMD CAN Message (296  0x128)
#define SHIFT_CMD_IDE (0U)
#define SHIFT_CMD_DLC (2U)
#define SHIFT_CMD_CANID (0x128)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE : 1;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

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

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE;                            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  // FORWARD is also HIGH on vehicles with LOW/HIGH, PARK and LOW only available on certain Vehicles.
  //  0 : "PARK"
  //  1 : "REVERSE"
  //  2 : "NEUTRAL"
  //  3 : "FORWARD/HIGH"
  //  4 : "LOW"
  //  7 : "NONE"
  // 
  uint8_t SHIFT_CMD;                         //      Bits= 8

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} SHIFT_CMD_t;

// def @STEERING_CMD CAN Message (300  0x12c)
#define STEERING_CMD_IDE (0U)
#define STEERING_CMD_DLC (5U)
#define STEERING_CMD_CANID (0x12c)
// signal: @POSITION_ro
#define PACMOD12_POSITION_ro_CovFactor (0.001000)
#define PACMOD12_POSITION_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_POSITION_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @ROTATION_RATE_ro
#define PACMOD12_ROTATION_RATE_ro_CovFactor (0.001000)
#define PACMOD12_ROTATION_RATE_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_ROTATION_RATE_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE : 1;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  int16_t POSITION_ro;                       //  [-] Bits=16 Factor= 0.001000        Unit:'rad'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t POSITION_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t ROTATION_RATE_ro;                 //      Bits=16 Factor= 0.001000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t ROTATION_RATE_phys;
#endif // PACMOD12_USE_SIGFLOAT

#else

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE;                            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  int16_t POSITION_ro;                       //  [-] Bits=16 Factor= 0.001000        Unit:'rad'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t POSITION_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t ROTATION_RATE_ro;                 //      Bits=16 Factor= 0.001000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t ROTATION_RATE_phys;
#endif // PACMOD12_USE_SIGFLOAT

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} STEERING_CMD_t;

// def @TURN_CMD CAN Message (304  0x130)
#define TURN_CMD_IDE (0U)
#define TURN_CMD_DLC (2U)
#define TURN_CMD_CANID (0x130)
#define TURN_CMD_CYC (100U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE : 1;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  //  0 : "RIGHT"
  //  1 : "NONE"
  //  2 : "LEFT"
  //  3 : "HAZARD"
  // 
  uint8_t TURN_SIGNAL_CMD;                   //      Bits= 8

#else

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE;                            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  //  0 : "RIGHT"
  //  1 : "NONE"
  //  2 : "LEFT"
  //  3 : "HAZARD"
  // 
  uint8_t TURN_SIGNAL_CMD;                   //      Bits= 8

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} TURN_CMD_t;

// def @WIPER_CMD CAN Message (308  0x134)
#define WIPER_CMD_IDE (0U)
#define WIPER_CMD_DLC (2U)
#define WIPER_CMD_CANID (0x134)
#define WIPER_CMD_CYC (100U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE : 1;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  //  255 : "HIGH"
  //  254 : "MEDIUM"
  //  253 : "LOW"
  //  10 : "INTERMITTENT_10"
  //  9 : "INTERMITTENT_9"
  //  8 : "INTERMITTENT_8"
  //  7 : "INTERMITTENT_7"
  //  6 : "INTERMITTENT_6"
  //  5 : "INTERMITTENT_5"
  //  4 : "INTERMITTENT_4"
  //  3 : "INTERMITTENT_3"
  //  2 : "INTERMITTENT_2"
  //  1 : "INTERMITTENT_1"
  //  0 : "WIPERS_OFF"
  // 
  uint8_t WIPER_CMD;                         //      Bits= 8

#else

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE;                            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  //  255 : "HIGH"
  //  254 : "MEDIUM"
  //  253 : "LOW"
  //  10 : "INTERMITTENT_10"
  //  9 : "INTERMITTENT_9"
  //  8 : "INTERMITTENT_8"
  //  7 : "INTERMITTENT_7"
  //  6 : "INTERMITTENT_6"
  //  5 : "INTERMITTENT_5"
  //  4 : "INTERMITTENT_4"
  //  3 : "INTERMITTENT_3"
  //  2 : "INTERMITTENT_2"
  //  1 : "INTERMITTENT_1"
  //  0 : "WIPERS_OFF"
  // 
  uint8_t WIPER_CMD;                         //      Bits= 8

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} WIPER_CMD_t;

// def @SPRAYER_CMD CAN Message (312  0x138)
#define SPRAYER_CMD_IDE (0U)
#define SPRAYER_CMD_DLC (2U)
#define SPRAYER_CMD_CANID (0x138)
#define SPRAYER_CMD_CYC (100U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE : 1;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  //  0 : "NOT_SPRAYING"
  //  1 : "SPRAYING"
  // 
  uint8_t SPRAYER_CMD : 1;                   //      Bits= 1

#else

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE;                            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  //  0 : "NOT_SPRAYING"
  //  1 : "SPRAYING"
  // 
  uint8_t SPRAYER_CMD;                       //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} SPRAYER_CMD_t;

// def @BRAKE_DECEL_CMD CAN Message (316  0x13c)
#define BRAKE_DECEL_CMD_IDE (0U)
#define BRAKE_DECEL_CMD_DLC (4U)
#define BRAKE_DECEL_CMD_CANID (0x13c)
// signal: @BRAKE_DECEL_CMD_ro
#define PACMOD12_BRAKE_DECEL_CMD_ro_CovFactor (0.001000)
#define PACMOD12_BRAKE_DECEL_CMD_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_BRAKE_DECEL_CMD_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE : 1;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  uint16_t BRAKE_DECEL_CMD_ro;               //      Bits=16 Factor= 0.001000        Unit:'m/s^2'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t BRAKE_DECEL_CMD_phys;
#endif // PACMOD12_USE_SIGFLOAT

  //  0 : "NO_ENDURANCE_BRK_INTGN_ALLOWED"
  //  1 : "ONLY_ENDURANCE_BRAKES_ALLOWED"
  //  2 : "ENDURANCE_BRK_INTGRN_ALLOWED"
  // 
  uint8_t XBR_EBI_MODE : 2;                  //      Bits= 2

  //  0 : "HIGHEST_PRIORITY"
  //  1 : "HIGH_PRIORITY"
  //  2 : "MEDIUM_PRIORITY"
  //  3 : "LOW_PRIORITY"
  // 
  uint8_t XBR_PRIORITY : 2;                  //      Bits= 2

  //  0 : "OVERRIDE_DISABLE"
  //  1 : "ACCEL_CONTROL_WITH_ADDITION_MODE"
  //  2 : "ACCEL_CONTROL_WITH_MAXIMUM_MODE"
  // 
  uint8_t XBR_CONTROL_MODE : 2;              //      Bits= 2

#else

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE;                            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  uint16_t BRAKE_DECEL_CMD_ro;               //      Bits=16 Factor= 0.001000        Unit:'m/s^2'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t BRAKE_DECEL_CMD_phys;
#endif // PACMOD12_USE_SIGFLOAT

  //  0 : "NO_ENDURANCE_BRK_INTGN_ALLOWED"
  //  1 : "ONLY_ENDURANCE_BRAKES_ALLOWED"
  //  2 : "ENDURANCE_BRK_INTGRN_ALLOWED"
  // 
  uint8_t XBR_EBI_MODE;                      //      Bits= 2

  //  0 : "HIGHEST_PRIORITY"
  //  1 : "HIGH_PRIORITY"
  //  2 : "MEDIUM_PRIORITY"
  //  3 : "LOW_PRIORITY"
  // 
  uint8_t XBR_PRIORITY;                      //      Bits= 2

  //  0 : "OVERRIDE_DISABLE"
  //  1 : "ACCEL_CONTROL_WITH_ADDITION_MODE"
  //  2 : "ACCEL_CONTROL_WITH_MAXIMUM_MODE"
  // 
  uint8_t XBR_CONTROL_MODE;                  //      Bits= 2

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} BRAKE_DECEL_CMD_t;

// def @REAR_PASS_DOOR_CMD CAN Message (320  0x140)
#define REAR_PASS_DOOR_CMD_IDE (0U)
#define REAR_PASS_DOOR_CMD_DLC (2U)
#define REAR_PASS_DOOR_CMD_CANID (0x140)
#define REAR_PASS_DOOR_CMD_CYC (100U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE : 1;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  //  0 : "NEUTRAL"
  //  1 : "OPEN"
  //  2 : "CLOSE"
  // 
  uint8_t REAR_PASS_DOOR_CMD;                //      Bits= 8

#else

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE;                            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  //  0 : "NEUTRAL"
  //  1 : "OPEN"
  //  2 : "CLOSE"
  // 
  uint8_t REAR_PASS_DOOR_CMD;                //      Bits= 8

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} REAR_PASS_DOOR_CMD_t;

// def @ENGINE_BRAKE_CMD CAN Message (324  0x144)
#define ENGINE_BRAKE_CMD_IDE (0U)
#define ENGINE_BRAKE_CMD_DLC (3U)
#define ENGINE_BRAKE_CMD_CANID (0x144)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE : 1;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  // Brake aggressiveness increases with increasing numerical values. Each operator control setting maps one-to-one with each LEVEL, starting with 1. Higher LEVELs with no match map to maximum.
  //  0 : "OFF"
  //  1 : "LEVEL_1"
  //  2 : "LEVEL_2"
  //  3 : "LEVEL_3"
  //  4 : "LEVEL_4"
  //  5 : "LEVEL_5"
  // 
  uint8_t ENGINE_BRAKE_CMD;                  //      Bits= 8

  // Braking aggressivness automatic while AUTO_ON.
  //  0 : "AUTO_OFF"
  //  1 : "AUTO_ON"
  // 
  uint8_t AUTO_CMD : 2;                      //      Bits= 2

#else

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE;                            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  // Brake aggressiveness increases with increasing numerical values. Each operator control setting maps one-to-one with each LEVEL, starting with 1. Higher LEVELs with no match map to maximum.
  //  0 : "OFF"
  //  1 : "LEVEL_1"
  //  2 : "LEVEL_2"
  //  3 : "LEVEL_3"
  //  4 : "LEVEL_4"
  //  5 : "LEVEL_5"
  // 
  uint8_t ENGINE_BRAKE_CMD;                  //      Bits= 8

  // Braking aggressivness automatic while AUTO_ON.
  //  0 : "AUTO_OFF"
  //  1 : "AUTO_ON"
  // 
  uint8_t AUTO_CMD;                          //      Bits= 2

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} ENGINE_BRAKE_CMD_t;

// def @EXHAUST_BRAKE_CMD CAN Message (325  0x145)
#define EXHAUST_BRAKE_CMD_IDE (0U)
#define EXHAUST_BRAKE_CMD_DLC (2U)
#define EXHAUST_BRAKE_CMD_CANID (0x145)
#define EXHAUST_BRAKE_CMD_CYC (100U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE : 1;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t EXHAUST_BRAKE_CMD;                 //      Bits= 8

#else

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE;                            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t EXHAUST_BRAKE_CMD;                 //      Bits= 8

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} EXHAUST_BRAKE_CMD_t;

// def @MARKER_LAMP_CMD CAN Message (328  0x148)
#define MARKER_LAMP_CMD_IDE (0U)
#define MARKER_LAMP_CMD_DLC (2U)
#define MARKER_LAMP_CMD_CANID (0x148)
#define MARKER_LAMP_CMD_CYC (100U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE : 1;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t MARKER_LAMP_CMD : 1;               //      Bits= 1

#else

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE;                            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t MARKER_LAMP_CMD;                   //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} MARKER_LAMP_CMD_t;

// def @CABIN_TEMP_CMD CAN Message (332  0x14c)
#define CABIN_TEMP_CMD_IDE (0U)
#define CABIN_TEMP_CMD_DLC (2U)
#define CABIN_TEMP_CMD_CANID (0x14c)
#define CABIN_TEMP_CMD_CYC (1000U)
// signal: @CABIN_TEMP_CMD_ro
#define PACMOD12_CABIN_TEMP_CMD_ro_CovFactor (0.100000)
#define PACMOD12_CABIN_TEMP_CMD_ro_toS(x) ( (uint8_t) (((x) - (10.000000)) / (0.100000)) )
#define PACMOD12_CABIN_TEMP_CMD_ro_fromS(x) ( (((x) * (0.100000)) + (10.000000)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE : 1;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  uint8_t CABIN_TEMP_CMD_ro;                 //      Bits= 8 Offset= 10.000000          Factor= 0.100000        Unit:'deg_C'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t CABIN_TEMP_CMD_phys;
#endif // PACMOD12_USE_SIGFLOAT

#else

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE;                            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  uint8_t CABIN_TEMP_CMD_ro;                 //      Bits= 8 Offset= 10.000000          Factor= 0.100000        Unit:'deg_C'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t CABIN_TEMP_CMD_phys;
#endif // PACMOD12_USE_SIGFLOAT

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} CABIN_TEMP_CMD_t;

// def @CABIN_FAN_SPEED_CMD CAN Message (336  0x150)
#define CABIN_FAN_SPEED_CMD_IDE (0U)
#define CABIN_FAN_SPEED_CMD_DLC (2U)
#define CABIN_FAN_SPEED_CMD_CANID (0x150)
#define CABIN_FAN_SPEED_CMD_CYC (1000U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE : 1;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  //  0 : "FAN_OFF"
  //  1 : "FAN_SPEED_1"
  //  2 : "FAN_SPEED_2"
  //  3 : "FAN_SPEED_3"
  //  4 : "FAN_SPEED_4"
  //  5 : "FAN_SPEED_5"
  //  6 : "FAN_SPEED_6"
  //  7 : "FAN_SPEED_7"
  //  8 : "FAN_SPEED_8"
  //  9 : "FAN_SPEED_9"
  //  10 : "FAN_SPEED_10"
  // 
  uint8_t CABIN_FAN_SPEED_CMD;               //      Bits= 8

#else

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE;                            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  //  0 : "FAN_OFF"
  //  1 : "FAN_SPEED_1"
  //  2 : "FAN_SPEED_2"
  //  3 : "FAN_SPEED_3"
  //  4 : "FAN_SPEED_4"
  //  5 : "FAN_SPEED_5"
  //  6 : "FAN_SPEED_6"
  //  7 : "FAN_SPEED_7"
  //  8 : "FAN_SPEED_8"
  //  9 : "FAN_SPEED_9"
  //  10 : "FAN_SPEED_10"
  // 
  uint8_t CABIN_FAN_SPEED_CMD;               //      Bits= 8

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} CABIN_FAN_SPEED_CMD_t;

// def @CABIN_CLIMATE_CMD CAN Message (340  0x154)
#define CABIN_CLIMATE_CMD_IDE (0U)
#define CABIN_CLIMATE_CMD_DLC (3U)
#define CABIN_CLIMATE_CMD_CANID (0x154)
#define CABIN_CLIMATE_CMD_CYC (1000U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE : 1;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t CMD_AC_OFF_ON : 2;                 //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t CMD_MAX_AC_OFF_ON : 2;             //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t CMD_DEFROST_OFF_ON : 2;            //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t CMD_MAX_DEFROST_OFF_ON : 2;        //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t CMD_DIR_UP_OFF_ON : 2;             //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t CMD_DIR_DOWN_OFF_ON : 2;           //      Bits= 2

#else

  //  0 : "DISABLE"
  //  1 : "ENABLE"
  // 
  uint8_t ENABLE;                            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t CMD_AC_OFF_ON;                     //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t CMD_MAX_AC_OFF_ON;                 //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t CMD_DEFROST_OFF_ON;                //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t CMD_MAX_DEFROST_OFF_ON;            //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t CMD_DIR_UP_OFF_ON;                 //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t CMD_DIR_DOWN_OFF_ON;               //      Bits= 2

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} CABIN_CLIMATE_CMD_t;

// def @ACCEL_RPT CAN Message (512  0x200)
#define ACCEL_RPT_IDE (0U)
#define ACCEL_RPT_DLC (8U)
#define ACCEL_RPT_CANID (0x200)
// signal: @MANUAL_INPUT_ro
#define PACMOD12_MANUAL_INPUT_ro_CovFactor (0.001000)
#define PACMOD12_MANUAL_INPUT_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_MANUAL_INPUT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @COMMANDED_VALUE_ro
#define PACMOD12_COMMANDED_VALUE_ro_CovFactor (0.001000)
#define PACMOD12_COMMANDED_VALUE_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_COMMANDED_VALUE_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @OUTPUT_VALUE_ro
#define PACMOD12_OUTPUT_VALUE_ro_CovFactor (0.001000)
#define PACMOD12_OUTPUT_VALUE_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_OUTPUT_VALUE_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED : 1;                       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

  uint16_t MANUAL_INPUT_ro;                  //      Bits=16 Factor= 0.001000        Unit:'%'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t MANUAL_INPUT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t COMMANDED_VALUE_ro;               //      Bits=16 Factor= 0.001000        Unit:'%'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t COMMANDED_VALUE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t OUTPUT_VALUE_ro;                  //      Bits=16 Factor= 0.001000        Unit:'%'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t OUTPUT_VALUE_phys;
#endif // PACMOD12_USE_SIGFLOAT

#else

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED;                           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

  uint16_t MANUAL_INPUT_ro;                  //      Bits=16 Factor= 0.001000        Unit:'%'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t MANUAL_INPUT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t COMMANDED_VALUE_ro;               //      Bits=16 Factor= 0.001000        Unit:'%'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t COMMANDED_VALUE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t OUTPUT_VALUE_ro;                  //      Bits=16 Factor= 0.001000        Unit:'%'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t OUTPUT_VALUE_phys;
#endif // PACMOD12_USE_SIGFLOAT

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} ACCEL_RPT_t;

// def @ACCEL_CMD_LIMIT_RPT CAN Message (513  0x201)
#define ACCEL_CMD_LIMIT_RPT_IDE (0U)
#define ACCEL_CMD_LIMIT_RPT_DLC (4U)
#define ACCEL_CMD_LIMIT_RPT_CANID (0x201)
// signal: @ACCEL_CMD_LIMIT_ro
#define PACMOD12_ACCEL_CMD_LIMIT_ro_CovFactor (0.001000)
#define PACMOD12_ACCEL_CMD_LIMIT_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_ACCEL_CMD_LIMIT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @LIMITED_ACCEL_CMD_ro
#define PACMOD12_LIMITED_ACCEL_CMD_ro_CovFactor (0.001000)
#define PACMOD12_LIMITED_ACCEL_CMD_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_LIMITED_ACCEL_CMD_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  uint16_t ACCEL_CMD_LIMIT_ro;               //      Bits=16 Factor= 0.001000        Unit:'%'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t ACCEL_CMD_LIMIT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t LIMITED_ACCEL_CMD_ro;             //      Bits=16 Factor= 0.001000        Unit:'%'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t LIMITED_ACCEL_CMD_phys;
#endif // PACMOD12_USE_SIGFLOAT

#else

  uint16_t ACCEL_CMD_LIMIT_ro;               //      Bits=16 Factor= 0.001000        Unit:'%'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t ACCEL_CMD_LIMIT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t LIMITED_ACCEL_CMD_ro;             //      Bits=16 Factor= 0.001000        Unit:'%'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t LIMITED_ACCEL_CMD_phys;
#endif // PACMOD12_USE_SIGFLOAT

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} ACCEL_CMD_LIMIT_RPT_t;

// def @BRAKE_RPT CAN Message (516  0x204)
#define BRAKE_RPT_IDE (0U)
#define BRAKE_RPT_DLC (8U)
#define BRAKE_RPT_CANID (0x204)
// signal: @MANUAL_INPUT_ro
#define PACMOD12_MANUAL_INPUT_ro_CovFactor (0.001000)
#define PACMOD12_MANUAL_INPUT_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_MANUAL_INPUT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @COMMANDED_VALUE_ro
#define PACMOD12_COMMANDED_VALUE_ro_CovFactor (0.001000)
#define PACMOD12_COMMANDED_VALUE_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_COMMANDED_VALUE_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @OUTPUT_VALUE_ro
#define PACMOD12_OUTPUT_VALUE_ro_CovFactor (0.001000)
#define PACMOD12_OUTPUT_VALUE_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_OUTPUT_VALUE_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED : 1;                       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

  uint16_t MANUAL_INPUT_ro;                  //      Bits=16 Factor= 0.001000        Unit:'%'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t MANUAL_INPUT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t COMMANDED_VALUE_ro;               //      Bits=16 Factor= 0.001000        Unit:'%'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t COMMANDED_VALUE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t OUTPUT_VALUE_ro;                  //      Bits=16 Factor= 0.001000        Unit:'%'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t OUTPUT_VALUE_phys;
#endif // PACMOD12_USE_SIGFLOAT

#else

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED;                           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

  uint16_t MANUAL_INPUT_ro;                  //      Bits=16 Factor= 0.001000        Unit:'%'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t MANUAL_INPUT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t COMMANDED_VALUE_ro;               //      Bits=16 Factor= 0.001000        Unit:'%'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t COMMANDED_VALUE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t OUTPUT_VALUE_ro;                  //      Bits=16 Factor= 0.001000        Unit:'%'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t OUTPUT_VALUE_phys;
#endif // PACMOD12_USE_SIGFLOAT

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} BRAKE_RPT_t;

// def @BRAKE_CMD_LIMIT_RPT CAN Message (517  0x205)
#define BRAKE_CMD_LIMIT_RPT_IDE (0U)
#define BRAKE_CMD_LIMIT_RPT_DLC (4U)
#define BRAKE_CMD_LIMIT_RPT_CANID (0x205)
// signal: @BRAKE_CMD_LIMIT_ro
#define PACMOD12_BRAKE_CMD_LIMIT_ro_CovFactor (0.001000)
#define PACMOD12_BRAKE_CMD_LIMIT_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_BRAKE_CMD_LIMIT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @LIMITED_BRAKE_CMD_ro
#define PACMOD12_LIMITED_BRAKE_CMD_ro_CovFactor (0.001000)
#define PACMOD12_LIMITED_BRAKE_CMD_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_LIMITED_BRAKE_CMD_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  uint16_t BRAKE_CMD_LIMIT_ro;               //      Bits=16 Factor= 0.001000        Unit:'%'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t BRAKE_CMD_LIMIT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t LIMITED_BRAKE_CMD_ro;             //      Bits=16 Factor= 0.001000        Unit:'%'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t LIMITED_BRAKE_CMD_phys;
#endif // PACMOD12_USE_SIGFLOAT

#else

  uint16_t BRAKE_CMD_LIMIT_ro;               //      Bits=16 Factor= 0.001000        Unit:'%'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t BRAKE_CMD_LIMIT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t LIMITED_BRAKE_CMD_ro;             //      Bits=16 Factor= 0.001000        Unit:'%'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t LIMITED_BRAKE_CMD_phys;
#endif // PACMOD12_USE_SIGFLOAT

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} BRAKE_CMD_LIMIT_RPT_t;

// def @CRUISE_CONTROL_BUTTONS_RPT CAN Message (520  0x208)
#define CRUISE_CONTROL_BUTTONS_RPT_IDE (0U)
#define CRUISE_CONTROL_BUTTONS_RPT_DLC (4U)
#define CRUISE_CONTROL_BUTTONS_RPT_CANID (0x208)
#define CRUISE_CONTROL_BUTTONS_RPT_CYC (100U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED : 1;                       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

  // NONE = no button pressed, CNCL = Cancel, FURTHER* or CLOSER* = Change follow distance, SET_DEC* = set/decrement, RES_INC* = resume/increment, ON_OFF = Cruise On/Off, LIM* = Speed limiter On/Off. *Exact function varies across vehicles - See platform guide.
  //  7 : "CRUISE_CONTROL_LIM"
  //  6 : "CRUISE_CONTROL_ON_OFF"
  //  5 : "CRUISE_CONTROL_RES_INC"
  //  4 : "CRUISE_CONTROL_SET_DEC"
  //  3 : "CRUISE_CONTROL_ACC_CLOSER"
  //  2 : "CRUISE_CONTROL_ACC_FURTHER"
  //  1 : "CRUISE_CONTROL_CNCL"
  //  0 : "CRUISE_CONTROL_NONE"
  // 
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  // NONE = no button pressed, CNCL = Cancel, FURTHER* or CLOSER* = Change follow distance, SET_DEC* = set/decrement, RES_INC* = resume/increment, ON_OFF = Cruise On/Off, LIM* = Speed limiter On/Off. *Exact function varies across vehicles - See platform guide.
  //  7 : "CRUISE_CONTROL_LIM"
  //  6 : "CRUISE_CONTROL_ON_OFF"
  //  5 : "CRUISE_CONTROL_RES_INC"
  //  4 : "CRUISE_CONTROL_SET_DEC"
  //  3 : "CRUISE_CONTROL_ACC_CLOSER"
  //  2 : "CRUISE_CONTROL_ACC_FURTHER"
  //  1 : "CRUISE_CONTROL_CNCL"
  //  0 : "CRUISE_CONTROL_NONE"
  // 
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  // NONE = no button pressed, CNCL = Cancel, FURTHER* or CLOSER* = Change follow distance, SET_DEC* = set/decrement, RES_INC* = resume/increment, ON_OFF = Cruise On/Off, LIM* = Speed limiter On/Off. *Exact function varies across vehicles - See platform guide.
  //  7 : "CRUISE_CONTROL_LIM"
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

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED;                           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

  // NONE = no button pressed, CNCL = Cancel, FURTHER* or CLOSER* = Change follow distance, SET_DEC* = set/decrement, RES_INC* = resume/increment, ON_OFF = Cruise On/Off, LIM* = Speed limiter On/Off. *Exact function varies across vehicles - See platform guide.
  //  7 : "CRUISE_CONTROL_LIM"
  //  6 : "CRUISE_CONTROL_ON_OFF"
  //  5 : "CRUISE_CONTROL_RES_INC"
  //  4 : "CRUISE_CONTROL_SET_DEC"
  //  3 : "CRUISE_CONTROL_ACC_CLOSER"
  //  2 : "CRUISE_CONTROL_ACC_FURTHER"
  //  1 : "CRUISE_CONTROL_CNCL"
  //  0 : "CRUISE_CONTROL_NONE"
  // 
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  // NONE = no button pressed, CNCL = Cancel, FURTHER* or CLOSER* = Change follow distance, SET_DEC* = set/decrement, RES_INC* = resume/increment, ON_OFF = Cruise On/Off, LIM* = Speed limiter On/Off. *Exact function varies across vehicles - See platform guide.
  //  7 : "CRUISE_CONTROL_LIM"
  //  6 : "CRUISE_CONTROL_ON_OFF"
  //  5 : "CRUISE_CONTROL_RES_INC"
  //  4 : "CRUISE_CONTROL_SET_DEC"
  //  3 : "CRUISE_CONTROL_ACC_CLOSER"
  //  2 : "CRUISE_CONTROL_ACC_FURTHER"
  //  1 : "CRUISE_CONTROL_CNCL"
  //  0 : "CRUISE_CONTROL_NONE"
  // 
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  // NONE = no button pressed, CNCL = Cancel, FURTHER* or CLOSER* = Change follow distance, SET_DEC* = set/decrement, RES_INC* = resume/increment, ON_OFF = Cruise On/Off, LIM* = Speed limiter On/Off. *Exact function varies across vehicles - See platform guide.
  //  7 : "CRUISE_CONTROL_LIM"
  //  6 : "CRUISE_CONTROL_ON_OFF"
  //  5 : "CRUISE_CONTROL_RES_INC"
  //  4 : "CRUISE_CONTROL_SET_DEC"
  //  3 : "CRUISE_CONTROL_ACC_CLOSER"
  //  2 : "CRUISE_CONTROL_ACC_FURTHER"
  //  1 : "CRUISE_CONTROL_CNCL"
  //  0 : "CRUISE_CONTROL_NONE"
  // 
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} CRUISE_CONTROL_BUTTONS_RPT_t;

// def @DASH_CONTROLS_LEFT_RPT CAN Message (524  0x20c)
#define DASH_CONTROLS_LEFT_RPT_IDE (0U)
#define DASH_CONTROLS_LEFT_RPT_DLC (4U)
#define DASH_CONTROLS_LEFT_RPT_CANID (0x20c)
#define DASH_CONTROLS_LEFT_RPT_CYC (100U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED : 1;                       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

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

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED;                           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

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

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} DASH_CONTROLS_LEFT_RPT_t;

// def @DASH_CONTROLS_RIGHT_RPT CAN Message (528  0x210)
#define DASH_CONTROLS_RIGHT_RPT_IDE (0U)
#define DASH_CONTROLS_RIGHT_RPT_DLC (4U)
#define DASH_CONTROLS_RIGHT_RPT_CANID (0x210)
#define DASH_CONTROLS_RIGHT_RPT_CYC (100U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED : 1;                       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

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

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED;                           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

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

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} DASH_CONTROLS_RIGHT_RPT_t;

// def @HAZARD_LIGHTS_RPT CAN Message (532  0x214)
#define HAZARD_LIGHTS_RPT_IDE (0U)
#define HAZARD_LIGHTS_RPT_DLC (4U)
#define HAZARD_LIGHTS_RPT_CANID (0x214)
#define HAZARD_LIGHTS_RPT_CYC (100U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED : 1;                       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

  // Platforms vary on use of latched or momentary interface, see platform user guide.
  //  0 : "OFF/NOT_PRESSED"
  //  1 : "ON/PRESSED"
  // 
  uint8_t MANUAL_INPUT : 1;                  //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t COMMANDED_VALUE : 1;               //      Bits= 1

  // Platforms vary on use of latched or momentary interface, see platform user guide.
  //  0 : "OFF/NOT_PRESSED"
  //  1 : "ON/PRESSED"
  // 
  uint8_t OUTPUT_VALUE : 1;                  //      Bits= 1

#else

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED;                           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

  // Platforms vary on use of latched or momentary interface, see platform user guide.
  //  0 : "OFF/NOT_PRESSED"
  //  1 : "ON/PRESSED"
  // 
  uint8_t MANUAL_INPUT;                      //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t COMMANDED_VALUE;                   //      Bits= 1

  // Platforms vary on use of latched or momentary interface, see platform user guide.
  //  0 : "OFF/NOT_PRESSED"
  //  1 : "ON/PRESSED"
  // 
  uint8_t OUTPUT_VALUE;                      //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} HAZARD_LIGHTS_RPT_t;

// def @HEADLIGHT_RPT CAN Message (536  0x218)
#define HEADLIGHT_RPT_IDE (0U)
#define HEADLIGHT_RPT_DLC (4U)
#define HEADLIGHT_RPT_CANID (0x218)
#define HEADLIGHT_RPT_CYC (100U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED : 1;                       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

  //  2 : "HIGH_BEAMS"
  //  1 : "LOW_BEAMS"
  //  0 : "HEADLIGHTS_OFF"
  // 
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  2 : "HIGH_BEAMS"
  //  1 : "LOW_BEAMS"
  //  0 : "HEADLIGHTS_OFF"
  // 
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  2 : "HIGH_BEAMS"
  //  1 : "LOW_BEAMS"
  //  0 : "HEADLIGHTS_OFF"
  // 
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#else

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED;                           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

  //  2 : "HIGH_BEAMS"
  //  1 : "LOW_BEAMS"
  //  0 : "HEADLIGHTS_OFF"
  // 
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  2 : "HIGH_BEAMS"
  //  1 : "LOW_BEAMS"
  //  0 : "HEADLIGHTS_OFF"
  // 
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  2 : "HIGH_BEAMS"
  //  1 : "LOW_BEAMS"
  //  0 : "HEADLIGHTS_OFF"
  // 
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} HEADLIGHT_RPT_t;

// def @HORN_RPT CAN Message (540  0x21c)
#define HORN_RPT_IDE (0U)
#define HORN_RPT_DLC (4U)
#define HORN_RPT_CANID (0x21c)
#define HORN_RPT_CYC (100U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED : 1;                       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

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

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED;                           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

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

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} HORN_RPT_t;

// def @MEDIA_CONTROLS_RPT CAN Message (544  0x220)
#define MEDIA_CONTROLS_RPT_IDE (0U)
#define MEDIA_CONTROLS_RPT_DLC (4U)
#define MEDIA_CONTROLS_RPT_CANID (0x220)
#define MEDIA_CONTROLS_RPT_CYC (100U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED : 1;                       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

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

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED;                           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

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

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} MEDIA_CONTROLS_RPT_t;

// def @PARKING_BRAKE_RPT CAN Message (548  0x224)
#define PARKING_BRAKE_RPT_IDE (0U)
#define PARKING_BRAKE_RPT_DLC (4U)
#define PARKING_BRAKE_RPT_CANID (0x224)
#define PARKING_BRAKE_RPT_CYC (100U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED : 1;                       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t MANUAL_INPUT : 1;                  //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t COMMANDED_VALUE : 1;               //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t OUTPUT_VALUE : 1;                  //      Bits= 1

#else

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED;                           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t MANUAL_INPUT;                      //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t COMMANDED_VALUE;                   //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t OUTPUT_VALUE;                      //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} PARKING_BRAKE_RPT_t;

// def @SHIFT_RPT CAN Message (552  0x228)
#define SHIFT_RPT_IDE (0U)
#define SHIFT_RPT_DLC (4U)
#define SHIFT_RPT_CANID (0x228)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED : 1;                       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

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

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED;                           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

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

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} SHIFT_RPT_t;

// def @STEERING_RPT CAN Message (556  0x22c)
#define STEERING_RPT_IDE (0U)
#define STEERING_RPT_DLC (8U)
#define STEERING_RPT_CANID (0x22c)
// signal: @MANUAL_INPUT_ro
#define PACMOD12_MANUAL_INPUT_ro_CovFactor (0.001000)
#define PACMOD12_MANUAL_INPUT_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_MANUAL_INPUT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @COMMANDED_VALUE_ro
#define PACMOD12_COMMANDED_VALUE_ro_CovFactor (0.001000)
#define PACMOD12_COMMANDED_VALUE_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_COMMANDED_VALUE_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @OUTPUT_VALUE_ro
#define PACMOD12_OUTPUT_VALUE_ro_CovFactor (0.001000)
#define PACMOD12_OUTPUT_VALUE_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_OUTPUT_VALUE_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED : 1;                       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

  int16_t MANUAL_INPUT_ro;                   //  [-] Bits=16 Factor= 0.001000        Unit:'rad'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t MANUAL_INPUT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int16_t COMMANDED_VALUE_ro;                //  [-] Bits=16 Factor= 0.001000        Unit:'rad'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t COMMANDED_VALUE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int16_t OUTPUT_VALUE_ro;                   //  [-] Bits=16 Factor= 0.001000        Unit:'rad'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t OUTPUT_VALUE_phys;
#endif // PACMOD12_USE_SIGFLOAT

#else

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED;                           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

  int16_t MANUAL_INPUT_ro;                   //  [-] Bits=16 Factor= 0.001000        Unit:'rad'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t MANUAL_INPUT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int16_t COMMANDED_VALUE_ro;                //  [-] Bits=16 Factor= 0.001000        Unit:'rad'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t COMMANDED_VALUE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int16_t OUTPUT_VALUE_ro;                   //  [-] Bits=16 Factor= 0.001000        Unit:'rad'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t OUTPUT_VALUE_phys;
#endif // PACMOD12_USE_SIGFLOAT

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} STEERING_RPT_t;

// def @STEERING_CMD_LIMIT_RPT CAN Message (557  0x22d)
#define STEERING_CMD_LIMIT_RPT_IDE (0U)
#define STEERING_CMD_LIMIT_RPT_DLC (8U)
#define STEERING_CMD_LIMIT_RPT_CANID (0x22d)
// signal: @POSITION_CMD_LIMIT_ro
#define PACMOD12_POSITION_CMD_LIMIT_ro_CovFactor (0.001000)
#define PACMOD12_POSITION_CMD_LIMIT_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_POSITION_CMD_LIMIT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @LIMITED_POSITION_CMD_ro
#define PACMOD12_LIMITED_POSITION_CMD_ro_CovFactor (0.001000)
#define PACMOD12_LIMITED_POSITION_CMD_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_LIMITED_POSITION_CMD_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @ROTATION_RATE_CMD_LIMIT_ro
#define PACMOD12_ROTATION_RATE_CMD_LIMIT_ro_CovFactor (0.001000)
#define PACMOD12_ROTATION_RATE_CMD_LIMIT_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_ROTATION_RATE_CMD_LIMIT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @LIMITED_ROTATION_RATE_ro
#define PACMOD12_LIMITED_ROTATION_RATE_ro_CovFactor (0.001000)
#define PACMOD12_LIMITED_ROTATION_RATE_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_LIMITED_ROTATION_RATE_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  int16_t POSITION_CMD_LIMIT_ro;             //  [-] Bits=16 Factor= 0.001000        Unit:'rad'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t POSITION_CMD_LIMIT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int16_t LIMITED_POSITION_CMD_ro;           //  [-] Bits=16 Factor= 0.001000        Unit:'rad'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t LIMITED_POSITION_CMD_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t ROTATION_RATE_CMD_LIMIT_ro;       //      Bits=16 Factor= 0.001000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t ROTATION_RATE_CMD_LIMIT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t LIMITED_ROTATION_RATE_ro;         //      Bits=16 Factor= 0.001000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t LIMITED_ROTATION_RATE_phys;
#endif // PACMOD12_USE_SIGFLOAT

#else

  int16_t POSITION_CMD_LIMIT_ro;             //  [-] Bits=16 Factor= 0.001000        Unit:'rad'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t POSITION_CMD_LIMIT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int16_t LIMITED_POSITION_CMD_ro;           //  [-] Bits=16 Factor= 0.001000        Unit:'rad'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t LIMITED_POSITION_CMD_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t ROTATION_RATE_CMD_LIMIT_ro;       //      Bits=16 Factor= 0.001000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t ROTATION_RATE_CMD_LIMIT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t LIMITED_ROTATION_RATE_ro;         //      Bits=16 Factor= 0.001000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t LIMITED_ROTATION_RATE_phys;
#endif // PACMOD12_USE_SIGFLOAT

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} STEERING_CMD_LIMIT_RPT_t;

// def @TURN_RPT CAN Message (560  0x230)
#define TURN_RPT_IDE (0U)
#define TURN_RPT_DLC (4U)
#define TURN_RPT_CANID (0x230)
#define TURN_RPT_CYC (100U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED : 1;                       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

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

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED;                           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

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

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} TURN_RPT_t;

// def @WIPER_RPT CAN Message (564  0x234)
#define WIPER_RPT_IDE (0U)
#define WIPER_RPT_DLC (4U)
#define WIPER_RPT_CANID (0x234)
#define WIPER_RPT_CYC (100U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED : 1;                       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

  //  255 : "HIGH"
  //  254 : "MEDIUM"
  //  253 : "LOW"
  //  10 : "INTERMITTENT_10"
  //  9 : "INTERMITTENT_9"
  //  8 : "INTERMITTENT_8"
  //  7 : "INTERMITTENT_7"
  //  6 : "INTERMITTENT_6"
  //  5 : "INTERMITTENT_5"
  //  4 : "INTERMITTENT_4"
  //  3 : "INTERMITTENT_3"
  //  2 : "INTERMITTENT_2"
  //  1 : "INTERMITTENT_1"
  //  0 : "WIPERS_OFF"
  // 
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  255 : "HIGH"
  //  254 : "MEDIUM"
  //  253 : "LOW"
  //  10 : "INTERMITTENT_10"
  //  9 : "INTERMITTENT_9"
  //  8 : "INTERMITTENT_8"
  //  7 : "INTERMITTENT_7"
  //  6 : "INTERMITTENT_6"
  //  5 : "INTERMITTENT_5"
  //  4 : "INTERMITTENT_4"
  //  3 : "INTERMITTENT_3"
  //  2 : "INTERMITTENT_2"
  //  1 : "INTERMITTENT_1"
  //  0 : "WIPERS_OFF"
  // 
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  255 : "HIGH"
  //  254 : "MEDIUM"
  //  253 : "LOW"
  //  10 : "INTERMITTENT_10"
  //  9 : "INTERMITTENT_9"
  //  8 : "INTERMITTENT_8"
  //  7 : "INTERMITTENT_7"
  //  6 : "INTERMITTENT_6"
  //  5 : "INTERMITTENT_5"
  //  4 : "INTERMITTENT_4"
  //  3 : "INTERMITTENT_3"
  //  2 : "INTERMITTENT_2"
  //  1 : "INTERMITTENT_1"
  //  0 : "WIPERS_OFF"
  // 
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#else

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED;                           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

  //  255 : "HIGH"
  //  254 : "MEDIUM"
  //  253 : "LOW"
  //  10 : "INTERMITTENT_10"
  //  9 : "INTERMITTENT_9"
  //  8 : "INTERMITTENT_8"
  //  7 : "INTERMITTENT_7"
  //  6 : "INTERMITTENT_6"
  //  5 : "INTERMITTENT_5"
  //  4 : "INTERMITTENT_4"
  //  3 : "INTERMITTENT_3"
  //  2 : "INTERMITTENT_2"
  //  1 : "INTERMITTENT_1"
  //  0 : "WIPERS_OFF"
  // 
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  255 : "HIGH"
  //  254 : "MEDIUM"
  //  253 : "LOW"
  //  10 : "INTERMITTENT_10"
  //  9 : "INTERMITTENT_9"
  //  8 : "INTERMITTENT_8"
  //  7 : "INTERMITTENT_7"
  //  6 : "INTERMITTENT_6"
  //  5 : "INTERMITTENT_5"
  //  4 : "INTERMITTENT_4"
  //  3 : "INTERMITTENT_3"
  //  2 : "INTERMITTENT_2"
  //  1 : "INTERMITTENT_1"
  //  0 : "WIPERS_OFF"
  // 
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  255 : "HIGH"
  //  254 : "MEDIUM"
  //  253 : "LOW"
  //  10 : "INTERMITTENT_10"
  //  9 : "INTERMITTENT_9"
  //  8 : "INTERMITTENT_8"
  //  7 : "INTERMITTENT_7"
  //  6 : "INTERMITTENT_6"
  //  5 : "INTERMITTENT_5"
  //  4 : "INTERMITTENT_4"
  //  3 : "INTERMITTENT_3"
  //  2 : "INTERMITTENT_2"
  //  1 : "INTERMITTENT_1"
  //  0 : "WIPERS_OFF"
  // 
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} WIPER_RPT_t;

// def @SPRAYER_RPT CAN Message (568  0x238)
#define SPRAYER_RPT_IDE (0U)
#define SPRAYER_RPT_DLC (4U)
#define SPRAYER_RPT_CANID (0x238)
#define SPRAYER_RPT_CYC (100U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED : 1;                       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

  //  0 : "NOT_SPRAYING"
  //  1 : "SPRAYING"
  // 
  uint8_t MANUAL_INPUT : 1;                  //      Bits= 1

  //  0 : "NOT_SPRAYING"
  //  1 : "SPRAYING"
  // 
  uint8_t COMMANDED_VALUE : 1;               //      Bits= 1

  //  0 : "NOT_SPRAYING"
  //  1 : "SPRAYING"
  // 
  uint8_t OUTPUT_VALUE : 1;                  //      Bits= 1

#else

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED;                           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

  //  0 : "NOT_SPRAYING"
  //  1 : "SPRAYING"
  // 
  uint8_t MANUAL_INPUT;                      //      Bits= 1

  //  0 : "NOT_SPRAYING"
  //  1 : "SPRAYING"
  // 
  uint8_t COMMANDED_VALUE;                   //      Bits= 1

  //  0 : "NOT_SPRAYING"
  //  1 : "SPRAYING"
  // 
  uint8_t OUTPUT_VALUE;                      //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} SPRAYER_RPT_t;

// def @BRAKE_DECEL_RPT CAN Message (572  0x23c)
#define BRAKE_DECEL_RPT_IDE (0U)
#define BRAKE_DECEL_RPT_DLC (7U)
#define BRAKE_DECEL_RPT_CANID (0x23c)
// signal: @MANUAL_INPUT_ro
#define PACMOD12_MANUAL_INPUT_ro_CovFactor (0.001000)
#define PACMOD12_MANUAL_INPUT_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_MANUAL_INPUT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @COMMANDED_VALUE_ro
#define PACMOD12_COMMANDED_VALUE_ro_CovFactor (0.001000)
#define PACMOD12_COMMANDED_VALUE_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_COMMANDED_VALUE_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @OUTPUT_VALUE_ro
#define PACMOD12_OUTPUT_VALUE_ro_CovFactor (0.001000)
#define PACMOD12_OUTPUT_VALUE_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_OUTPUT_VALUE_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED : 1;                       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

  uint16_t MANUAL_INPUT_ro;                  //      Bits=16 Factor= 0.001000        Unit:'m/s^2'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t MANUAL_INPUT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t COMMANDED_VALUE_ro;               //      Bits=16 Factor= 0.001000        Unit:'m/s^2'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t COMMANDED_VALUE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t OUTPUT_VALUE_ro;                  //      Bits=16 Factor= 0.001000        Unit:'m/s^2'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t OUTPUT_VALUE_phys;
#endif // PACMOD12_USE_SIGFLOAT

#else

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED;                           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

  uint16_t MANUAL_INPUT_ro;                  //      Bits=16 Factor= 0.001000        Unit:'m/s^2'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t MANUAL_INPUT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t COMMANDED_VALUE_ro;               //      Bits=16 Factor= 0.001000        Unit:'m/s^2'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t COMMANDED_VALUE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t OUTPUT_VALUE_ro;                  //      Bits=16 Factor= 0.001000        Unit:'m/s^2'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t OUTPUT_VALUE_phys;
#endif // PACMOD12_USE_SIGFLOAT

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} BRAKE_DECEL_RPT_t;

// def @REAR_PASS_DOOR_RPT CAN Message (576  0x240)
#define REAR_PASS_DOOR_RPT_IDE (0U)
#define REAR_PASS_DOOR_RPT_DLC (4U)
#define REAR_PASS_DOOR_RPT_CANID (0x240)
#define REAR_PASS_DOOR_RPT_CYC (100U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED : 1;                       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

  //  0 : "NEUTRAL"
  //  1 : "OPEN"
  //  2 : "CLOSE"
  // 
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  0 : "NEUTRAL"
  //  1 : "OPEN"
  //  2 : "CLOSE"
  // 
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  0 : "NEUTRAL"
  //  1 : "OPEN"
  //  2 : "CLOSE"
  // 
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#else

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED;                           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

  //  0 : "NEUTRAL"
  //  1 : "OPEN"
  //  2 : "CLOSE"
  // 
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  0 : "NEUTRAL"
  //  1 : "OPEN"
  //  2 : "CLOSE"
  // 
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  0 : "NEUTRAL"
  //  1 : "OPEN"
  //  2 : "CLOSE"
  // 
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} REAR_PASS_DOOR_RPT_t;

// def @ENGINE_BRAKE_RPT CAN Message (580  0x244)
#define ENGINE_BRAKE_RPT_IDE (0U)
#define ENGINE_BRAKE_RPT_DLC (5U)
#define ENGINE_BRAKE_RPT_CANID (0x244)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED : 1;                       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

  // Brake aggressiveness increases with increasing numerical values. Each operator control setting maps one-to-one with each LEVEL, starting with 1. Higher LEVELs with no match map to maximum.
  //  0 : "OFF"
  //  1 : "LEVEL_1"
  //  2 : "LEVEL_2"
  //  3 : "LEVEL_3"
  //  4 : "LEVEL_4"
  //  5 : "LEVEL_5"
  // 
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  // Brake aggressiveness increases with increasing numerical values. Each operator control setting maps one-to-one with each LEVEL, starting with 1. Higher LEVELs with no match map to maximum.
  //  0 : "OFF"
  //  1 : "LEVEL_1"
  //  2 : "LEVEL_2"
  //  3 : "LEVEL_3"
  //  4 : "LEVEL_4"
  //  5 : "LEVEL_5"
  // 
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  // Brake aggressiveness increases with increasing numerical values. Each operator control setting maps one-to-one with each LEVEL, starting with 1. Higher LEVELs with no match map to maximum.
  //  0 : "OFF"
  //  1 : "LEVEL_1"
  //  2 : "LEVEL_2"
  //  3 : "LEVEL_3"
  //  4 : "LEVEL_4"
  //  5 : "LEVEL_5"
  // 
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

  // Braking aggressiveness automatic while AUTO_ON.
  //  0 : "AUTO_OFF"
  //  1 : "AUTO_ON"
  // 
  uint8_t MAN_AUTO : 2;                      //      Bits= 2

  // Braking aggressiveness automatic while AUTO_ON.
  //  0 : "AUTO_OFF"
  //  1 : "AUTO_ON"
  // 
  uint8_t CMD_AUTO : 2;                      //      Bits= 2

  // Braking aggressiveness automatic while AUTO_ON.
  //  0 : "AUTO_OFF"
  //  1 : "AUTO_ON"
  // 
  uint8_t OUT_AUTO : 2;                      //      Bits= 2

#else

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED;                           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

  // Brake aggressiveness increases with increasing numerical values. Each operator control setting maps one-to-one with each LEVEL, starting with 1. Higher LEVELs with no match map to maximum.
  //  0 : "OFF"
  //  1 : "LEVEL_1"
  //  2 : "LEVEL_2"
  //  3 : "LEVEL_3"
  //  4 : "LEVEL_4"
  //  5 : "LEVEL_5"
  // 
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  // Brake aggressiveness increases with increasing numerical values. Each operator control setting maps one-to-one with each LEVEL, starting with 1. Higher LEVELs with no match map to maximum.
  //  0 : "OFF"
  //  1 : "LEVEL_1"
  //  2 : "LEVEL_2"
  //  3 : "LEVEL_3"
  //  4 : "LEVEL_4"
  //  5 : "LEVEL_5"
  // 
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  // Brake aggressiveness increases with increasing numerical values. Each operator control setting maps one-to-one with each LEVEL, starting with 1. Higher LEVELs with no match map to maximum.
  //  0 : "OFF"
  //  1 : "LEVEL_1"
  //  2 : "LEVEL_2"
  //  3 : "LEVEL_3"
  //  4 : "LEVEL_4"
  //  5 : "LEVEL_5"
  // 
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

  // Braking aggressiveness automatic while AUTO_ON.
  //  0 : "AUTO_OFF"
  //  1 : "AUTO_ON"
  // 
  uint8_t MAN_AUTO;                          //      Bits= 2

  // Braking aggressiveness automatic while AUTO_ON.
  //  0 : "AUTO_OFF"
  //  1 : "AUTO_ON"
  // 
  uint8_t CMD_AUTO;                          //      Bits= 2

  // Braking aggressiveness automatic while AUTO_ON.
  //  0 : "AUTO_OFF"
  //  1 : "AUTO_ON"
  // 
  uint8_t OUT_AUTO;                          //      Bits= 2

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} ENGINE_BRAKE_RPT_t;

// def @EXHAUST_BRAKE_RPT CAN Message (581  0x245)
#define EXHAUST_BRAKE_RPT_IDE (0U)
#define EXHAUST_BRAKE_RPT_DLC (4U)
#define EXHAUST_BRAKE_RPT_CANID (0x245)
#define EXHAUST_BRAKE_RPT_CYC (100U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED : 1;                       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

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

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED;                           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

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

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} EXHAUST_BRAKE_RPT_t;

// def @MARKER_LAMP_RPT CAN Message (584  0x248)
#define MARKER_LAMP_RPT_IDE (0U)
#define MARKER_LAMP_RPT_DLC (4U)
#define MARKER_LAMP_RPT_CANID (0x248)
#define MARKER_LAMP_RPT_CYC (100U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED : 1;                       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t MANUAL_INPUT : 1;                  //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t COMMANDED_VALUE : 1;               //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t OUTPUT_VALUE : 1;                  //      Bits= 1

#else

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED;                           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t MANUAL_INPUT;                      //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t COMMANDED_VALUE;                   //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t OUTPUT_VALUE;                      //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} MARKER_LAMP_RPT_t;

// def @CABIN_TEMP_RPT CAN Message (588  0x24c)
#define CABIN_TEMP_RPT_IDE (0U)
#define CABIN_TEMP_RPT_DLC (4U)
#define CABIN_TEMP_RPT_CANID (0x24c)
#define CABIN_TEMP_RPT_CYC (1000U)
// signal: @MANUAL_INPUT_ro
#define PACMOD12_MANUAL_INPUT_ro_CovFactor (0.100000)
#define PACMOD12_MANUAL_INPUT_ro_toS(x) ( (uint8_t) (((x) - (10.000000)) / (0.100000)) )
#define PACMOD12_MANUAL_INPUT_ro_fromS(x) ( (((x) * (0.100000)) + (10.000000)) )
// signal: @COMMANDED_VALUE_ro
#define PACMOD12_COMMANDED_VALUE_ro_CovFactor (0.100000)
#define PACMOD12_COMMANDED_VALUE_ro_toS(x) ( (uint8_t) (((x) - (10.000000)) / (0.100000)) )
#define PACMOD12_COMMANDED_VALUE_ro_fromS(x) ( (((x) * (0.100000)) + (10.000000)) )
// signal: @OUTPUT_VALUE_ro
#define PACMOD12_OUTPUT_VALUE_ro_CovFactor (0.100000)
#define PACMOD12_OUTPUT_VALUE_ro_toS(x) ( (uint8_t) (((x) - (10.000000)) / (0.100000)) )
#define PACMOD12_OUTPUT_VALUE_ro_fromS(x) ( (((x) * (0.100000)) + (10.000000)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED : 1;                       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

  uint8_t MANUAL_INPUT_ro;                   //      Bits= 8 Offset= 10.000000          Factor= 0.100000        Unit:'deg_C'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t MANUAL_INPUT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint8_t COMMANDED_VALUE_ro;                //      Bits= 8 Offset= 10.000000          Factor= 0.100000        Unit:'deg_C'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t COMMANDED_VALUE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint8_t OUTPUT_VALUE_ro;                   //      Bits= 8 Offset= 10.000000          Factor= 0.100000        Unit:'deg_C'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t OUTPUT_VALUE_phys;
#endif // PACMOD12_USE_SIGFLOAT

#else

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED;                           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

  uint8_t MANUAL_INPUT_ro;                   //      Bits= 8 Offset= 10.000000          Factor= 0.100000        Unit:'deg_C'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t MANUAL_INPUT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint8_t COMMANDED_VALUE_ro;                //      Bits= 8 Offset= 10.000000          Factor= 0.100000        Unit:'deg_C'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t COMMANDED_VALUE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint8_t OUTPUT_VALUE_ro;                   //      Bits= 8 Offset= 10.000000          Factor= 0.100000        Unit:'deg_C'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t OUTPUT_VALUE_phys;
#endif // PACMOD12_USE_SIGFLOAT

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} CABIN_TEMP_RPT_t;

// def @CABIN_FAN_SPEED_RPT CAN Message (592  0x250)
#define CABIN_FAN_SPEED_RPT_IDE (0U)
#define CABIN_FAN_SPEED_RPT_DLC (4U)
#define CABIN_FAN_SPEED_RPT_CANID (0x250)
#define CABIN_FAN_SPEED_RPT_CYC (1000U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED : 1;                       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

  //  0 : "FAN_OFF"
  //  1 : "FAN_SPEED_1"
  //  2 : "FAN_SPEED_2"
  //  3 : "FAN_SPEED_3"
  //  4 : "FAN_SPEED_4"
  //  5 : "FAN_SPEED_5"
  //  6 : "FAN_SPEED_6"
  //  7 : "FAN_SPEED_7"
  //  8 : "FAN_SPEED_8"
  //  9 : "FAN_SPEED_9"
  //  10 : "FAN_SPEED_10"
  // 
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  0 : "FAN_OFF"
  //  1 : "FAN_SPEED_1"
  //  2 : "FAN_SPEED_2"
  //  3 : "FAN_SPEED_3"
  //  4 : "FAN_SPEED_4"
  //  5 : "FAN_SPEED_5"
  //  6 : "FAN_SPEED_6"
  //  7 : "FAN_SPEED_7"
  //  8 : "FAN_SPEED_8"
  //  9 : "FAN_SPEED_9"
  //  10 : "FAN_SPEED_10"
  // 
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  0 : "FAN_OFF"
  //  1 : "FAN_SPEED_1"
  //  2 : "FAN_SPEED_2"
  //  3 : "FAN_SPEED_3"
  //  4 : "FAN_SPEED_4"
  //  5 : "FAN_SPEED_5"
  //  6 : "FAN_SPEED_6"
  //  7 : "FAN_SPEED_7"
  //  8 : "FAN_SPEED_8"
  //  9 : "FAN_SPEED_9"
  //  10 : "FAN_SPEED_10"
  // 
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#else

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED;                           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

  //  0 : "FAN_OFF"
  //  1 : "FAN_SPEED_1"
  //  2 : "FAN_SPEED_2"
  //  3 : "FAN_SPEED_3"
  //  4 : "FAN_SPEED_4"
  //  5 : "FAN_SPEED_5"
  //  6 : "FAN_SPEED_6"
  //  7 : "FAN_SPEED_7"
  //  8 : "FAN_SPEED_8"
  //  9 : "FAN_SPEED_9"
  //  10 : "FAN_SPEED_10"
  // 
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  0 : "FAN_OFF"
  //  1 : "FAN_SPEED_1"
  //  2 : "FAN_SPEED_2"
  //  3 : "FAN_SPEED_3"
  //  4 : "FAN_SPEED_4"
  //  5 : "FAN_SPEED_5"
  //  6 : "FAN_SPEED_6"
  //  7 : "FAN_SPEED_7"
  //  8 : "FAN_SPEED_8"
  //  9 : "FAN_SPEED_9"
  //  10 : "FAN_SPEED_10"
  // 
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  0 : "FAN_OFF"
  //  1 : "FAN_SPEED_1"
  //  2 : "FAN_SPEED_2"
  //  3 : "FAN_SPEED_3"
  //  4 : "FAN_SPEED_4"
  //  5 : "FAN_SPEED_5"
  //  6 : "FAN_SPEED_6"
  //  7 : "FAN_SPEED_7"
  //  8 : "FAN_SPEED_8"
  //  9 : "FAN_SPEED_9"
  //  10 : "FAN_SPEED_10"
  // 
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} CABIN_FAN_SPEED_RPT_t;

// def @CABIN_CLIMATE_RPT CAN Message (596  0x254)
#define CABIN_CLIMATE_RPT_IDE (0U)
#define CABIN_CLIMATE_RPT_DLC (7U)
#define CABIN_CLIMATE_RPT_CANID (0x254)
#define CABIN_CLIMATE_RPT_CYC (1000U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED : 1;                       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t MAN_AC_OFF_ON : 2;                 //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t MAN_MAX_AC_OFF_ON : 2;             //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t MAN_DEFROST_OFF_ON : 2;            //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t MAN_MAX_DEFROST_OFF_ON : 2;        //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t MAN_DIR_UP_OFF_ON : 2;             //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t MAN_DIR_DOWN_OFF_ON : 2;           //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t CMD_AC_OFF_ON : 2;                 //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t CMD_MAX_AC_OFF_ON : 2;             //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t CMD_DEFROST_OFF_ON : 2;            //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t CMD_MAX_DEFROST_OFF_ON : 2;        //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t CMD_DIR_UP_OFF_ON : 2;             //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t CMD_DIR_DOWN_OFF_ON : 2;           //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t OUT_AC_OFF_ON : 2;                 //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t OUT_MAX_AC_OFF_ON : 2;             //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t OUT_DEFROST_OFF_ON : 2;            //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t OUT_MAX_DEFROST_OFF_ON : 2;        //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t OUT_DIR_UP_OFF_ON : 2;             //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t OUT_DIR_DOWN_OFF_ON : 2;           //      Bits= 2

#else

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENABLED;                           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  // 
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t MAN_AC_OFF_ON;                     //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t MAN_MAX_AC_OFF_ON;                 //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t MAN_DEFROST_OFF_ON;                //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t MAN_MAX_DEFROST_OFF_ON;            //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t MAN_DIR_UP_OFF_ON;                 //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t MAN_DIR_DOWN_OFF_ON;               //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t CMD_AC_OFF_ON;                     //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t CMD_MAX_AC_OFF_ON;                 //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t CMD_DEFROST_OFF_ON;                //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t CMD_MAX_DEFROST_OFF_ON;            //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t CMD_DIR_UP_OFF_ON;                 //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t CMD_DIR_DOWN_OFF_ON;               //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t OUT_AC_OFF_ON;                     //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t OUT_MAX_AC_OFF_ON;                 //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t OUT_DEFROST_OFF_ON;                //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t OUT_MAX_DEFROST_OFF_ON;            //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t OUT_DIR_UP_OFF_ON;                 //      Bits= 2

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t OUT_DIR_DOWN_OFF_ON;               //      Bits= 2

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} CABIN_CLIMATE_RPT_t;

// def @ACCEL_AUX_RPT CAN Message (768  0x300)
#define ACCEL_AUX_RPT_IDE (0U)
#define ACCEL_AUX_RPT_DLC (6U)
#define ACCEL_AUX_RPT_CANID (0x300)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  // OPERATOR_INTERACTION shall have the value of 1 if the driver is moving, changing, or otherwise touching the operator control(s) that relates to this signal to an extent that is detectable. Otherwise, the value shall be 0.
  //  0 : "NO_INTERACTION"
  //  1 : "INTERACTION"
  // 
  uint8_t OPERATOR_INTERACTION : 1;             //      Bits= 1

  // ACCEL_LIMITING_ACTIVE shall have the value of 1 if a limiting function is reducing an excessive command.  Otherwise, the value shall be 0.
  //  0 : "NOT_ACTIVE"
  //  1 : "ACTIVE"
  // 
  uint8_t ACCEL_LIMITING_ACTIVE : 1;            //      Bits= 1

  // PARK_BRAKE_INTERLOCK_ACTIVE shall have the value of 1 if an active parking brake prevents or reduces the command. Otherwise, the value shall be 0.
  //  0 : "NOT_ACTIVE"
  //  1 : "ACTIVE"
  // 
  uint8_t PRK_BRK_INTERLOCK_ACTIVE : 1;         //      Bits= 1

  // BRAKE_INTERLOCK_ACTIVE shall have the value of 1 if an active brake prevents or reduces the command. Otherwise, the value shall be 0.
  //  0 : "NOT_ACTIVE"
  //  1 : "ACTIVE"
  // 
  uint8_t BRAKE_INTERLOCK_ACTIVE : 1;           //      Bits= 1

  // CALIBRATION_STATUS shall indicate the status of a calibration of this system. If this system has more than one calibration, then those calibrations are INACTIVE.
  //  0 : "INACTIVE"
  //  1 : "ACTIVE"
  //  2 : "COMPLETE"
  //  3 : "ERROR"
  // 
  uint8_t CALIBRATION_STATUS : 3;               //      Bits= 3

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t OPERATOR_INTERACTION_AVAIL : 1;       //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t ACCEL_LIMITING_ACTIVE_AVAIL : 1;      //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t PRK_BRK_INTERLOCK_ACTIVE_AVAIL : 1;   //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t BRAKE_INTERLOCK_ACTIVE_AVAIL : 1;     //      Bits= 1

#else

  // OPERATOR_INTERACTION shall have the value of 1 if the driver is moving, changing, or otherwise touching the operator control(s) that relates to this signal to an extent that is detectable. Otherwise, the value shall be 0.
  //  0 : "NO_INTERACTION"
  //  1 : "INTERACTION"
  // 
  uint8_t OPERATOR_INTERACTION;                 //      Bits= 1

  // ACCEL_LIMITING_ACTIVE shall have the value of 1 if a limiting function is reducing an excessive command.  Otherwise, the value shall be 0.
  //  0 : "NOT_ACTIVE"
  //  1 : "ACTIVE"
  // 
  uint8_t ACCEL_LIMITING_ACTIVE;                //      Bits= 1

  // PARK_BRAKE_INTERLOCK_ACTIVE shall have the value of 1 if an active parking brake prevents or reduces the command. Otherwise, the value shall be 0.
  //  0 : "NOT_ACTIVE"
  //  1 : "ACTIVE"
  // 
  uint8_t PRK_BRK_INTERLOCK_ACTIVE;             //      Bits= 1

  // BRAKE_INTERLOCK_ACTIVE shall have the value of 1 if an active brake prevents or reduces the command. Otherwise, the value shall be 0.
  //  0 : "NOT_ACTIVE"
  //  1 : "ACTIVE"
  // 
  uint8_t BRAKE_INTERLOCK_ACTIVE;               //      Bits= 1

  // CALIBRATION_STATUS shall indicate the status of a calibration of this system. If this system has more than one calibration, then those calibrations are INACTIVE.
  //  0 : "INACTIVE"
  //  1 : "ACTIVE"
  //  2 : "COMPLETE"
  //  3 : "ERROR"
  // 
  uint8_t CALIBRATION_STATUS;                   //      Bits= 3

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t OPERATOR_INTERACTION_AVAIL;           //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t ACCEL_LIMITING_ACTIVE_AVAIL;          //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t PRK_BRK_INTERLOCK_ACTIVE_AVAIL;       //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t BRAKE_INTERLOCK_ACTIVE_AVAIL;         //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} ACCEL_AUX_RPT_t;

// def @BRAKE_AUX_RPT CAN Message (772  0x304)
#define BRAKE_AUX_RPT_IDE (0U)
#define BRAKE_AUX_RPT_DLC (8U)
#define BRAKE_AUX_RPT_CANID (0x304)
// signal: @BRAKE_PRESSURE_ro
#define PACMOD12_BRAKE_PRESSURE_ro_CovFactor (0.000100)
#define PACMOD12_BRAKE_PRESSURE_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.000100)) )
#define PACMOD12_BRAKE_PRESSURE_ro_fromS(x) ( (((x) * (0.000100)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  uint16_t BRAKE_PRESSURE_ro;                //      Bits=16 Factor= 0.000100        Unit:'%'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t BRAKE_PRESSURE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  // OPERATOR_INTERACTION shall have the value of 1 if the driver is moving, changing, or otherwise touching the operator control(s) that relates to this signal to an extent that is detectable. Otherwise, the value shall be 0.
  //  0 : "NO_INTERACTION"
  //  1 : "INTERACTION"
  // 
  uint8_t OPERATOR_INTERACTION : 1;          //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t BRAKE_ON_OFF : 1;                  //      Bits= 1

  // BRAKE_LIMITING_ACTIVE shall have the value of 1 if a limiting function is reducing an excessive command.  Otherwise, the value shall be 0.
  //  0 : "NOT_ACTIVE"
  //  1 : "ACTIVE"
  // 
  uint8_t BRAKE_LIMITING_ACTIVE : 1;         //      Bits= 1

  // BRAKE_REDUCED_ASSIST shall have the value of 1 if Brake System is in a reduced assist mode. Otherwise, the value shall be 0.
  //  0 : "NOT_REDUCED"
  //  1 : "REDUCED"
  // 
  uint8_t BRAKE_REDUCED_ASSIST : 1;          //      Bits= 1

  // CALIBRATION_STATUS shall indicate the status of a calibration of this system. If this system has more than one calibration, then those calibrations are INACTIVE.
  //  0 : "INACTIVE"
  //  1 : "ACTIVE"
  //  2 : "COMPLETE"
  //  3 : "ERROR"
  // 
  uint8_t CALIBRATION_STATUS : 3;            //      Bits= 3

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t BRAKE_PRESSURE_AVAIL : 1;          //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t OPERATOR_INTERACTION_AVAIL : 1;    //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t BRAKE_ON_OFF_AVAIL : 1;            //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t BRAKE_LIMITING_ACTIVE_AVAIL : 1;   //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t BRAKE_REDUCED_ASSIST_AVAIL : 1;    //      Bits= 1

#else

  uint16_t BRAKE_PRESSURE_ro;                //      Bits=16 Factor= 0.000100        Unit:'%'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t BRAKE_PRESSURE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  // OPERATOR_INTERACTION shall have the value of 1 if the driver is moving, changing, or otherwise touching the operator control(s) that relates to this signal to an extent that is detectable. Otherwise, the value shall be 0.
  //  0 : "NO_INTERACTION"
  //  1 : "INTERACTION"
  // 
  uint8_t OPERATOR_INTERACTION;              //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t BRAKE_ON_OFF;                      //      Bits= 1

  // BRAKE_LIMITING_ACTIVE shall have the value of 1 if a limiting function is reducing an excessive command.  Otherwise, the value shall be 0.
  //  0 : "NOT_ACTIVE"
  //  1 : "ACTIVE"
  // 
  uint8_t BRAKE_LIMITING_ACTIVE;             //      Bits= 1

  // BRAKE_REDUCED_ASSIST shall have the value of 1 if Brake System is in a reduced assist mode. Otherwise, the value shall be 0.
  //  0 : "NOT_REDUCED"
  //  1 : "REDUCED"
  // 
  uint8_t BRAKE_REDUCED_ASSIST;              //      Bits= 1

  // CALIBRATION_STATUS shall indicate the status of a calibration of this system. If this system has more than one calibration, then those calibrations are INACTIVE.
  //  0 : "INACTIVE"
  //  1 : "ACTIVE"
  //  2 : "COMPLETE"
  //  3 : "ERROR"
  // 
  uint8_t CALIBRATION_STATUS;                //      Bits= 3

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t BRAKE_PRESSURE_AVAIL;              //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t OPERATOR_INTERACTION_AVAIL;        //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t BRAKE_ON_OFF_AVAIL;                //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t BRAKE_LIMITING_ACTIVE_AVAIL;       //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t BRAKE_REDUCED_ASSIST_AVAIL;        //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} BRAKE_AUX_RPT_t;

// def @HEADLIGHT_AUX_RPT CAN Message (792  0x318)
#define HEADLIGHT_AUX_RPT_IDE (0U)
#define HEADLIGHT_AUX_RPT_DLC (3U)
#define HEADLIGHT_AUX_RPT_CANID (0x318)
#define HEADLIGHT_AUX_RPT_CYC (100U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t HEADLIGHTS_ON : 1;                 //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t HEADLIGHTS_ON_BRIGHT : 1;          //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t FOG_LIGHTS_ON : 1;                 //      Bits= 1

  //  3 : "HEADLIGHTS_ON_AUTO_MODE"
  //  2 : "HEADLIGHTS_ON_MANUAL_MODE"
  //  1 : "PARKING_LIGHTS_ONLY"
  //  0 : "HEADLIGHTS_OFF"
  // 
  uint8_t HEADLIGHTS_MODE;                   //      Bits= 8

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t HEADLIGHTS_ON_AVAIL : 1;           //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t HEADLIGHTS_ON_BRIGHT_AVAIL : 1;    //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t FOG_LIGHTS_ON_AVAIL : 1;           //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t HEADLIGHTS_MODE_AVAIL : 1;         //      Bits= 1

#else

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t HEADLIGHTS_ON;                     //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t HEADLIGHTS_ON_BRIGHT;              //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t FOG_LIGHTS_ON;                     //      Bits= 1

  //  3 : "HEADLIGHTS_ON_AUTO_MODE"
  //  2 : "HEADLIGHTS_ON_MANUAL_MODE"
  //  1 : "PARKING_LIGHTS_ONLY"
  //  0 : "HEADLIGHTS_OFF"
  // 
  uint8_t HEADLIGHTS_MODE;                   //      Bits= 8

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t HEADLIGHTS_ON_AVAIL;               //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t HEADLIGHTS_ON_BRIGHT_AVAIL;        //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t FOG_LIGHTS_ON_AVAIL;               //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t HEADLIGHTS_MODE_AVAIL;             //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} HEADLIGHT_AUX_RPT_t;

// def @PARKING_BRAKE_AUX_RPT CAN Message (804  0x324)
#define PARKING_BRAKE_AUX_RPT_IDE (0U)
#define PARKING_BRAKE_AUX_RPT_DLC (2U)
#define PARKING_BRAKE_AUX_RPT_CANID (0x324)
#define PARKING_BRAKE_AUX_RPT_CYC (100U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "RELEASED"
  //  1 : "APPLIED"
  //  2 : "BETWEEN"
  //  3 : "ERROR"
  // 
  uint8_t PARKING_BRAKE_STATUS : 2;          //      Bits= 2

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t PARKING_BRAKE_STATUS_AVAIL : 1;    //      Bits= 1

#else

  //  0 : "RELEASED"
  //  1 : "APPLIED"
  //  2 : "BETWEEN"
  //  3 : "ERROR"
  // 
  uint8_t PARKING_BRAKE_STATUS;              //      Bits= 2

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t PARKING_BRAKE_STATUS_AVAIL;        //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} PARKING_BRAKE_AUX_RPT_t;

// def @SHIFT_AUX_RPT CAN Message (808  0x328)
#define SHIFT_AUX_RPT_IDE (0U)
#define SHIFT_AUX_RPT_DLC (3U)
#define SHIFT_AUX_RPT_CANID (0x328)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "NOT_BETWEEN_GEARS"
  //  1 : "BETWEEN_GEARS"
  // 
  uint8_t BETWEEN_GEARS : 1;                  //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t STAY_IN_NEUTRAL_MODE : 1;           //      Bits= 1

  //  0 : "NOT_ACTIVE"
  //  1 : "ACTIVE"
  // 
  uint8_t BRAKE_INTERLOCK_ACTIVE : 1;         //      Bits= 1

  //  0 : "NOT_ACTIVE"
  //  1 : "ACTIVE"
  // 
  uint8_t SPEED_INTERLOCK_ACTIVE : 1;         //      Bits= 1

  //  0 : "SILENT"
  //  1 : "BEEP"
  // 
  uint8_t WRITE_TO_CONFIG : 1;                //      Bits= 1

  // CALIBRATION_STATUS shall indicate the status of a calibration of this system. If this system has more than one calibration, then those calibrations are INACTIVE.
  //  0 : "INACTIVE"
  //  1 : "ACTIVE"
  //  2 : "COMPLETE"
  //  3 : "ERROR"
  // 
  uint8_t CALIBRATION_STATUS : 3;             //      Bits= 3

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t BETWEEN_GEARS_AVAIL : 1;            //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t STAY_IN_NEUTRAL_MODE_AVAIL : 1;     //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t BRAKE_INTERLOCK_ACTIVE_AVAIL : 1;   //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t SPEED_INTERLOCK_ACTIVE_AVAIL : 1;   //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t WRITE_TO_CONFIG_IS_VALID : 1;       //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t GEAR_NUMBER_AVAIL : 1;              //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t SHIFT_MODE_AVAIL : 1;               //      Bits= 1

  // Gear number is positive for forward gears.
  int8_t GEAR_NUMBER : 6;                     //  [-] Bits= 6

  //  0 : "STANDARD"
  //  1 : "SPORT"
  // 
  uint8_t SHIFT_MODE : 2;                     //      Bits= 2

#else

  //  0 : "NOT_BETWEEN_GEARS"
  //  1 : "BETWEEN_GEARS"
  // 
  uint8_t BETWEEN_GEARS;                      //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t STAY_IN_NEUTRAL_MODE;               //      Bits= 1

  //  0 : "NOT_ACTIVE"
  //  1 : "ACTIVE"
  // 
  uint8_t BRAKE_INTERLOCK_ACTIVE;             //      Bits= 1

  //  0 : "NOT_ACTIVE"
  //  1 : "ACTIVE"
  // 
  uint8_t SPEED_INTERLOCK_ACTIVE;             //      Bits= 1

  //  0 : "SILENT"
  //  1 : "BEEP"
  // 
  uint8_t WRITE_TO_CONFIG;                    //      Bits= 1

  // CALIBRATION_STATUS shall indicate the status of a calibration of this system. If this system has more than one calibration, then those calibrations are INACTIVE.
  //  0 : "INACTIVE"
  //  1 : "ACTIVE"
  //  2 : "COMPLETE"
  //  3 : "ERROR"
  // 
  uint8_t CALIBRATION_STATUS;                 //      Bits= 3

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t BETWEEN_GEARS_AVAIL;                //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t STAY_IN_NEUTRAL_MODE_AVAIL;         //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t BRAKE_INTERLOCK_ACTIVE_AVAIL;       //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t SPEED_INTERLOCK_ACTIVE_AVAIL;       //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t WRITE_TO_CONFIG_IS_VALID;           //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t GEAR_NUMBER_AVAIL;                  //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t SHIFT_MODE_AVAIL;                   //      Bits= 1

  // Gear number is positive for forward gears.
  int8_t GEAR_NUMBER;                         //  [-] Bits= 6

  //  0 : "STANDARD"
  //  1 : "SPORT"
  // 
  uint8_t SHIFT_MODE;                         //      Bits= 2

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} SHIFT_AUX_RPT_t;

// def @STEERING_AUX_RPT CAN Message (812  0x32c)
#define STEERING_AUX_RPT_IDE (0U)
#define STEERING_AUX_RPT_DLC (8U)
#define STEERING_AUX_RPT_CANID (0x32c)
// signal: @STEERING_TORQUE_ro
#define PACMOD12_STEERING_TORQUE_ro_CovFactor (0.000100)
#define PACMOD12_STEERING_TORQUE_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.000100)) )
#define PACMOD12_STEERING_TORQUE_ro_fromS(x) ( (((x) * (0.000100)) + (0.000000)) )
// signal: @ROTATION_RATE_ro
#define PACMOD12_ROTATION_RATE_ro_CovFactor (0.001000)
#define PACMOD12_ROTATION_RATE_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_ROTATION_RATE_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  int16_t STEERING_TORQUE_ro;                     //  [-] Bits=16 Factor= 0.000100        Unit:'%'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t STEERING_TORQUE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t ROTATION_RATE_ro;                      //      Bits=16 Factor= 0.001000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t ROTATION_RATE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  // OPERATOR_INTERACTION shall have the value of 1 if the driver is moving, changing, or otherwise touching the operator control(s) that relates to this signal to an extent that is detectable. Otherwise, the value shall be 0.
  //  0 : "NO_INTERACTION"
  //  1 : "INTERACTION"
  // 
  uint8_t OPERATOR_INTERACTION : 1;               //      Bits= 1

  // Counterclockwise rotation is positive according to the right-hand rule.
  //  0 : "POSITIVE"
  //  1 : "NEGATIVE"
  // 
  uint8_t ROTATION_RATE_SIGN : 1;                 //      Bits= 1

  //  0 : "NOT_CALIBRATED"
  //  1 : "CALIBRATED"
  // 
  uint8_t VEHICLE_ANGLE_CALIB_STATUS : 1;         //      Bits= 1

  // STEERING_LIMITING_ACTIVE shall have the value of 1 if a limiting function is reducing an excessive command.  Otherwise, the value shall be 0.
  //  0 : "NOT_ACTIVE"
  //  1 : "ACTIVE"
  // 
  uint8_t STEERING_LIMITING_ACTIVE : 1;           //      Bits= 1

  // CALIBRATION_STATUS shall indicate the status of a calibration of this system. If this system has more than one calibration, then those calibrations are INACTIVE.
  //  0 : "INACTIVE"
  //  1 : "ACTIVE"
  //  2 : "COMPLETE"
  //  3 : "ERROR"
  // 
  uint8_t CALIBRATION_STATUS : 3;                 //      Bits= 3

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t STEERING_TORQUE_AVAIL : 1;              //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t ROTATION_RATE_AVAIL : 1;                //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t OPERATOR_INTERACTION_AVAIL : 1;         //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t ROTATION_RATE_SIGN_AVAIL : 1;           //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t VEHICLE_ANGLE_CALIB_STATUS_AVAIL : 1;   //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t STEERING_LIMITING_ACTIVE_AVAIL : 1;     //      Bits= 1

#else

  int16_t STEERING_TORQUE_ro;                     //  [-] Bits=16 Factor= 0.000100        Unit:'%'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t STEERING_TORQUE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t ROTATION_RATE_ro;                      //      Bits=16 Factor= 0.001000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t ROTATION_RATE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  // OPERATOR_INTERACTION shall have the value of 1 if the driver is moving, changing, or otherwise touching the operator control(s) that relates to this signal to an extent that is detectable. Otherwise, the value shall be 0.
  //  0 : "NO_INTERACTION"
  //  1 : "INTERACTION"
  // 
  uint8_t OPERATOR_INTERACTION;                   //      Bits= 1

  // Counterclockwise rotation is positive according to the right-hand rule.
  //  0 : "POSITIVE"
  //  1 : "NEGATIVE"
  // 
  uint8_t ROTATION_RATE_SIGN;                     //      Bits= 1

  //  0 : "NOT_CALIBRATED"
  //  1 : "CALIBRATED"
  // 
  uint8_t VEHICLE_ANGLE_CALIB_STATUS;             //      Bits= 1

  // STEERING_LIMITING_ACTIVE shall have the value of 1 if a limiting function is reducing an excessive command.  Otherwise, the value shall be 0.
  //  0 : "NOT_ACTIVE"
  //  1 : "ACTIVE"
  // 
  uint8_t STEERING_LIMITING_ACTIVE;               //      Bits= 1

  // CALIBRATION_STATUS shall indicate the status of a calibration of this system. If this system has more than one calibration, then those calibrations are INACTIVE.
  //  0 : "INACTIVE"
  //  1 : "ACTIVE"
  //  2 : "COMPLETE"
  //  3 : "ERROR"
  // 
  uint8_t CALIBRATION_STATUS;                     //      Bits= 3

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t STEERING_TORQUE_AVAIL;                  //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t ROTATION_RATE_AVAIL;                    //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t OPERATOR_INTERACTION_AVAIL;             //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t ROTATION_RATE_SIGN_AVAIL;               //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t VEHICLE_ANGLE_CALIB_STATUS_AVAIL;       //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t STEERING_LIMITING_ACTIVE_AVAIL;         //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} STEERING_AUX_RPT_t;

// def @TURN_AUX_RPT CAN Message (816  0x330)
#define TURN_AUX_RPT_IDE (0U)
#define TURN_AUX_RPT_DLC (2U)
#define TURN_AUX_RPT_CANID (0x330)
#define TURN_AUX_RPT_CYC (100U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t DRIVER_BLINKER_BULB_ON : 1;         //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t PASS_BLINKER_BULB_ON : 1;           //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t DRIVER_BLINKER_BULB_ON_AVAIL : 1;   //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t PASS_BLINKER_BULB_ON_AVAIL : 1;     //      Bits= 1

#else

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t DRIVER_BLINKER_BULB_ON;             //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t PASS_BLINKER_BULB_ON;               //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t DRIVER_BLINKER_BULB_ON_AVAIL;       //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t PASS_BLINKER_BULB_ON_AVAIL;         //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} TURN_AUX_RPT_t;

// def @WIPER_AUX_RPT CAN Message (820  0x334)
#define WIPER_AUX_RPT_IDE (0U)
#define WIPER_AUX_RPT_DLC (2U)
#define WIPER_AUX_RPT_CANID (0x334)
#define WIPER_AUX_RPT_CYC (100U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "NOT_WIPING"
  //  1 : "WIPING"
  // 
  uint8_t FRONT_WIPING : 1;                  //      Bits= 1

  //  0 : "NOT_SPRAYING"
  //  1 : "SPRAYING"
  // 
  uint8_t FRONT_SPRAYING : 1;                //      Bits= 1

  //  0 : "NOT_WIPING"
  //  1 : "WIPING"
  // 
  uint8_t REAR_WIPING : 1;                   //      Bits= 1

  //  0 : "NOT_SPRAYING"
  //  1 : "SPRAYING"
  // 
  uint8_t REAR_SPRAYING : 1;                 //      Bits= 1

  //  0 : "NOT_NEAR_EMPTY"
  //  1 : "NEAR_EMPTY"
  // 
  uint8_t SPRAY_NEAR_EMPTY : 1;              //      Bits= 1

  //  0 : "NOT_EMPTY"
  //  1 : "EMPTY"
  // 
  uint8_t SPRAY_EMPTY : 1;                   //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t FRONT_WIPING_AVAIL : 1;            //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t FRONT_SPRAYING_AVAIL : 1;          //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t REAR_WIPING_AVAIL : 1;             //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t REAR_SPRAYING_AVAIL : 1;           //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t SPRAY_NEAR_EMPTY_AVAIL : 1;        //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t SPRAY_EMPTY_AVAIL : 1;             //      Bits= 1

#else

  //  0 : "NOT_WIPING"
  //  1 : "WIPING"
  // 
  uint8_t FRONT_WIPING;                      //      Bits= 1

  //  0 : "NOT_SPRAYING"
  //  1 : "SPRAYING"
  // 
  uint8_t FRONT_SPRAYING;                    //      Bits= 1

  //  0 : "NOT_WIPING"
  //  1 : "WIPING"
  // 
  uint8_t REAR_WIPING;                       //      Bits= 1

  //  0 : "NOT_SPRAYING"
  //  1 : "SPRAYING"
  // 
  uint8_t REAR_SPRAYING;                     //      Bits= 1

  //  0 : "NOT_NEAR_EMPTY"
  //  1 : "NEAR_EMPTY"
  // 
  uint8_t SPRAY_NEAR_EMPTY;                  //      Bits= 1

  //  0 : "NOT_EMPTY"
  //  1 : "EMPTY"
  // 
  uint8_t SPRAY_EMPTY;                       //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t FRONT_WIPING_AVAIL;                //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t FRONT_SPRAYING_AVAIL;              //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t REAR_WIPING_AVAIL;                 //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t REAR_SPRAYING_AVAIL;               //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t SPRAY_NEAR_EMPTY_AVAIL;            //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t SPRAY_EMPTY_AVAIL;                 //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} WIPER_AUX_RPT_t;

// def @BRAKE_DECEL_AUX_RPT CAN Message (824  0x338)
#define BRAKE_DECEL_AUX_RPT_IDE (0U)
#define BRAKE_DECEL_AUX_RPT_DLC (3U)
#define BRAKE_DECEL_AUX_RPT_CANID (0x338)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "NO_BRAKE_DEMAND"
  //  1 : "DRIVERS_BRAKE_DEMAND"
  //  2 : "ADDITION_MODE_OF_XBR_ACCEL_CTRL"
  //  3 : "MAXIMUM_MODE_OF_XBR_ACCEL_CTRL"
  // 
  uint8_t XBR_ACTIVE_CONTROL_MODE : 4;         //      Bits= 4

  //  0 : "ANY_EXT_BRK_DEMAND_WILL_BE_ACPTD"
  //  2 : "NO_EXT_BRK_DEMAND_WILL_BE_ACPTD"
  // 
  uint8_t XBR_SYSTEM_STATE : 2;                //      Bits= 2

  //  0 : "FOUNDATION_BRAKES_NOT_IN_USE"
  //  1 : "FOUNDATION_BRAKES_IN_USE"
  // 
  uint8_t FOUNDATION_BRAKE_USE : 2;            //      Bits= 2

  //  0 : "INACTIVE"
  //  1 : "ACTIVE"
  //  2 : "ACTIVE_BUT_INACTIVE_SOON"
  //  6 : "ERROR"
  // 
  uint8_t HILL_HOLDER_MODE : 3;                //      Bits= 3

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t XBR_ACTIVE_CONTROL_MODE_AVAIL : 1;   //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t XBR_SYSTEM_STATE_AVAIL : 1;          //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t FOUNDATION_BRAKE_USE_AVAIL : 1;      //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t HILL_HOLDER_MODE_AVAIL : 1;          //      Bits= 1

#else

  //  0 : "NO_BRAKE_DEMAND"
  //  1 : "DRIVERS_BRAKE_DEMAND"
  //  2 : "ADDITION_MODE_OF_XBR_ACCEL_CTRL"
  //  3 : "MAXIMUM_MODE_OF_XBR_ACCEL_CTRL"
  // 
  uint8_t XBR_ACTIVE_CONTROL_MODE;             //      Bits= 4

  //  0 : "ANY_EXT_BRK_DEMAND_WILL_BE_ACPTD"
  //  2 : "NO_EXT_BRK_DEMAND_WILL_BE_ACPTD"
  // 
  uint8_t XBR_SYSTEM_STATE;                    //      Bits= 2

  //  0 : "FOUNDATION_BRAKES_NOT_IN_USE"
  //  1 : "FOUNDATION_BRAKES_IN_USE"
  // 
  uint8_t FOUNDATION_BRAKE_USE;                //      Bits= 2

  //  0 : "INACTIVE"
  //  1 : "ACTIVE"
  //  2 : "ACTIVE_BUT_INACTIVE_SOON"
  //  6 : "ERROR"
  // 
  uint8_t HILL_HOLDER_MODE;                    //      Bits= 3

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t XBR_ACTIVE_CONTROL_MODE_AVAIL;       //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t XBR_SYSTEM_STATE_AVAIL;              //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t FOUNDATION_BRAKE_USE_AVAIL;          //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t HILL_HOLDER_MODE_AVAIL;              //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} BRAKE_DECEL_AUX_RPT_t;

// def @ENGINE_BRAKE_AUX_RPT CAN Message (836  0x344)
#define ENGINE_BRAKE_AUX_RPT_IDE (0U)
#define ENGINE_BRAKE_AUX_RPT_DLC (4U)
#define ENGINE_BRAKE_AUX_RPT_CANID (0x344)
// signal: @ACTUAL_ENGINE_BRK_TORQUE_ro
#define PACMOD12_ACTUAL_ENGINE_BRK_TORQUE_ro_CovFactor (1)
#define PACMOD12_ACTUAL_ENGINE_BRK_TORQUE_ro_toS(x) ( (int16_t) ((x) - (-125)) )
#define PACMOD12_ACTUAL_ENGINE_BRK_TORQUE_ro_fromS(x) ( ((x) + (-125)) )
// signal: @INTENDED_ENGINE_BRK_TORQUE_ro
#define PACMOD12_INTENDED_ENGINE_BRK_TORQUE_ro_CovFactor (1)
#define PACMOD12_INTENDED_ENGINE_BRK_TORQUE_ro_toS(x) ( (int16_t) ((x) - (-125)) )
#define PACMOD12_INTENDED_ENGINE_BRK_TORQUE_ro_fromS(x) ( ((x) + (-125)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "BRAKING_NOT_ACTIVE"
  //  1 : "BRAKING_ACTIVE"
  // 
  uint8_t ENGINE_BRAKE_STATUS : 2;                //      Bits= 2

  int16_t ACTUAL_ENGINE_BRK_TORQUE_ro;            //  [-] Bits= 8 Offset= -125               Unit:'%'

#ifdef PACMOD12_USE_SIGFLOAT
  int16_t ACTUAL_ENGINE_BRK_TORQUE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int16_t INTENDED_ENGINE_BRK_TORQUE_ro;          //  [-] Bits= 8 Offset= -125               Unit:'%'

#ifdef PACMOD12_USE_SIGFLOAT
  int16_t INTENDED_ENGINE_BRK_TORQUE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t ENGINE_BRAKE_STATUS_AVAIL : 1;          //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t ACTUAL_ENGINE_BRK_TORQUE_AVAIL : 1;     //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t INTENDED_ENGINE_BRK_TORQUE_AVAIL : 1;   //      Bits= 1

#else

  //  0 : "BRAKING_NOT_ACTIVE"
  //  1 : "BRAKING_ACTIVE"
  // 
  uint8_t ENGINE_BRAKE_STATUS;                    //      Bits= 2

  int16_t ACTUAL_ENGINE_BRK_TORQUE_ro;            //  [-] Bits= 8 Offset= -125               Unit:'%'

#ifdef PACMOD12_USE_SIGFLOAT
  int16_t ACTUAL_ENGINE_BRK_TORQUE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int16_t INTENDED_ENGINE_BRK_TORQUE_ro;          //  [-] Bits= 8 Offset= -125               Unit:'%'

#ifdef PACMOD12_USE_SIGFLOAT
  int16_t INTENDED_ENGINE_BRK_TORQUE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t ENGINE_BRAKE_STATUS_AVAIL;              //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t ACTUAL_ENGINE_BRK_TORQUE_AVAIL;         //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t INTENDED_ENGINE_BRK_TORQUE_AVAIL;       //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} ENGINE_BRAKE_AUX_RPT_t;

// def @VEHICLE_SPEED_RPT CAN Message (1024 0x400)
#define VEHICLE_SPEED_RPT_IDE (0U)
#define VEHICLE_SPEED_RPT_DLC (3U)
#define VEHICLE_SPEED_RPT_CANID (0x400)
// signal: @VEHICLE_SPEED_ro
#define PACMOD12_VEHICLE_SPEED_ro_CovFactor (0.010000)
#define PACMOD12_VEHICLE_SPEED_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.010000)) )
#define PACMOD12_VEHICLE_SPEED_ro_fromS(x) ( (((x) * (0.010000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  int16_t VEHICLE_SPEED_ro;                  //  [-] Bits=16 Factor= 0.010000        Unit:'m/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t VEHICLE_SPEED_phys;
#endif // PACMOD12_USE_SIGFLOAT

  //  0 : "INVALID"
  //  1 : "VALID"
  // 
  uint8_t VEHICLE_SPEED_VALID : 1;           //      Bits= 1

#else

  int16_t VEHICLE_SPEED_ro;                  //  [-] Bits=16 Factor= 0.010000        Unit:'m/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t VEHICLE_SPEED_phys;
#endif // PACMOD12_USE_SIGFLOAT

  //  0 : "INVALID"
  //  1 : "VALID"
  // 
  uint8_t VEHICLE_SPEED_VALID;               //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} VEHICLE_SPEED_RPT_t;

// def @BRAKE_MOTOR_RPT_1 CAN Message (1025 0x401)
#define BRAKE_MOTOR_RPT_1_IDE (0U)
#define BRAKE_MOTOR_RPT_1_DLC (8U)
#define BRAKE_MOTOR_RPT_1_CANID (0x401)
// signal: @MOTOR_CURRENT_ro
#define PACMOD12_MOTOR_CURRENT_ro_CovFactor (0.001000)
#define PACMOD12_MOTOR_CURRENT_ro_toS(x) ( (int32_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_MOTOR_CURRENT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @SHAFT_POSITION_ro
#define PACMOD12_SHAFT_POSITION_ro_CovFactor (0.001000)
#define PACMOD12_SHAFT_POSITION_ro_toS(x) ( (int32_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_SHAFT_POSITION_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  int32_t MOTOR_CURRENT_ro;                  //  [-] Bits=32 Factor= 0.001000        Unit:'amps'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t MOTOR_CURRENT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int32_t SHAFT_POSITION_ro;                 //  [-] Bits=32 Factor= 0.001000        Unit:'radians'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t SHAFT_POSITION_phys;
#endif // PACMOD12_USE_SIGFLOAT

#else

  int32_t MOTOR_CURRENT_ro;                  //  [-] Bits=32 Factor= 0.001000        Unit:'amps'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t MOTOR_CURRENT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int32_t SHAFT_POSITION_ro;                 //  [-] Bits=32 Factor= 0.001000        Unit:'radians'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t SHAFT_POSITION_phys;
#endif // PACMOD12_USE_SIGFLOAT

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} BRAKE_MOTOR_RPT_1_t;

// def @BRAKE_MOTOR_RPT_2 CAN Message (1026 0x402)
#define BRAKE_MOTOR_RPT_2_IDE (0U)
#define BRAKE_MOTOR_RPT_2_DLC (8U)
#define BRAKE_MOTOR_RPT_2_CANID (0x402)
// signal: @ANGULAR_SPEED_ro
#define PACMOD12_ANGULAR_SPEED_ro_CovFactor (0.100000)
#define PACMOD12_ANGULAR_SPEED_ro_toS(x) ( (int32_t) (((x) - (0.000000)) / (0.100000)) )
#define PACMOD12_ANGULAR_SPEED_ro_fromS(x) ( (((x) * (0.100000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  int16_t ENCODER_TEMPERATURE;               //  [-] Bits=16 Unit:'deg_C'

  int16_t MOTOR_TEMPERATURE;                 //  [-] Bits=16 Unit:'deg_C'

  int32_t ANGULAR_SPEED_ro;                  //  [-] Bits=32 Factor= 0.100000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t ANGULAR_SPEED_phys;
#endif // PACMOD12_USE_SIGFLOAT

#else

  int16_t ENCODER_TEMPERATURE;               //  [-] Bits=16 Unit:'deg_C'

  int16_t MOTOR_TEMPERATURE;                 //  [-] Bits=16 Unit:'deg_C'

  int32_t ANGULAR_SPEED_ro;                  //  [-] Bits=32 Factor= 0.100000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t ANGULAR_SPEED_phys;
#endif // PACMOD12_USE_SIGFLOAT

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} BRAKE_MOTOR_RPT_2_t;

// def @BRAKE_MOTOR_RPT_3 CAN Message (1027 0x403)
#define BRAKE_MOTOR_RPT_3_IDE (0U)
#define BRAKE_MOTOR_RPT_3_DLC (8U)
#define BRAKE_MOTOR_RPT_3_CANID (0x403)
// signal: @TORQUE_OUTPUT_ro
#define PACMOD12_TORQUE_OUTPUT_ro_CovFactor (0.001000)
#define PACMOD12_TORQUE_OUTPUT_ro_toS(x) ( (int32_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_TORQUE_OUTPUT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @TORQUE_INPUT_ro
#define PACMOD12_TORQUE_INPUT_ro_CovFactor (0.001000)
#define PACMOD12_TORQUE_INPUT_ro_toS(x) ( (int32_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_TORQUE_INPUT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  int32_t TORQUE_OUTPUT_ro;                  //  [-] Bits=32 Factor= 0.001000        Unit:'N-m'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t TORQUE_OUTPUT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int32_t TORQUE_INPUT_ro;                   //  [-] Bits=32 Factor= 0.001000        Unit:'N-m'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t TORQUE_INPUT_phys;
#endif // PACMOD12_USE_SIGFLOAT

#else

  int32_t TORQUE_OUTPUT_ro;                  //  [-] Bits=32 Factor= 0.001000        Unit:'N-m'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t TORQUE_OUTPUT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int32_t TORQUE_INPUT_ro;                   //  [-] Bits=32 Factor= 0.001000        Unit:'N-m'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t TORQUE_INPUT_phys;
#endif // PACMOD12_USE_SIGFLOAT

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} BRAKE_MOTOR_RPT_3_t;

// def @STEERING_MOTOR_RPT_1 CAN Message (1028 0x404)
#define STEERING_MOTOR_RPT_1_IDE (0U)
#define STEERING_MOTOR_RPT_1_DLC (8U)
#define STEERING_MOTOR_RPT_1_CANID (0x404)
// signal: @MOTOR_CURRENT_ro
#define PACMOD12_MOTOR_CURRENT_ro_CovFactor (0.001000)
#define PACMOD12_MOTOR_CURRENT_ro_toS(x) ( (int32_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_MOTOR_CURRENT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @SHAFT_POSITION_ro
#define PACMOD12_SHAFT_POSITION_ro_CovFactor (0.001000)
#define PACMOD12_SHAFT_POSITION_ro_toS(x) ( (int32_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_SHAFT_POSITION_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  int32_t MOTOR_CURRENT_ro;                  //  [-] Bits=32 Factor= 0.001000        Unit:'amps'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t MOTOR_CURRENT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int32_t SHAFT_POSITION_ro;                 //  [-] Bits=32 Factor= 0.001000        Unit:'radians'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t SHAFT_POSITION_phys;
#endif // PACMOD12_USE_SIGFLOAT

#else

  int32_t MOTOR_CURRENT_ro;                  //  [-] Bits=32 Factor= 0.001000        Unit:'amps'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t MOTOR_CURRENT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int32_t SHAFT_POSITION_ro;                 //  [-] Bits=32 Factor= 0.001000        Unit:'radians'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t SHAFT_POSITION_phys;
#endif // PACMOD12_USE_SIGFLOAT

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} STEERING_MOTOR_RPT_1_t;

// def @STEERING_MOTOR_RPT_2 CAN Message (1029 0x405)
#define STEERING_MOTOR_RPT_2_IDE (0U)
#define STEERING_MOTOR_RPT_2_DLC (8U)
#define STEERING_MOTOR_RPT_2_CANID (0x405)
// signal: @ANGULAR_SPEED_ro
#define PACMOD12_ANGULAR_SPEED_ro_CovFactor (0.100000)
#define PACMOD12_ANGULAR_SPEED_ro_toS(x) ( (int32_t) (((x) - (0.000000)) / (0.100000)) )
#define PACMOD12_ANGULAR_SPEED_ro_fromS(x) ( (((x) * (0.100000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  int16_t ENCODER_TEMPERATURE;               //  [-] Bits=16 Unit:'deg_C'

  int16_t MOTOR_TEMPERATURE;                 //  [-] Bits=16 Unit:'deg_C'

  int32_t ANGULAR_SPEED_ro;                  //  [-] Bits=32 Factor= 0.100000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t ANGULAR_SPEED_phys;
#endif // PACMOD12_USE_SIGFLOAT

#else

  int16_t ENCODER_TEMPERATURE;               //  [-] Bits=16 Unit:'deg_C'

  int16_t MOTOR_TEMPERATURE;                 //  [-] Bits=16 Unit:'deg_C'

  int32_t ANGULAR_SPEED_ro;                  //  [-] Bits=32 Factor= 0.100000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t ANGULAR_SPEED_phys;
#endif // PACMOD12_USE_SIGFLOAT

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} STEERING_MOTOR_RPT_2_t;

// def @STEERING_MOTOR_RPT_3 CAN Message (1030 0x406)
#define STEERING_MOTOR_RPT_3_IDE (0U)
#define STEERING_MOTOR_RPT_3_DLC (8U)
#define STEERING_MOTOR_RPT_3_CANID (0x406)
// signal: @TORQUE_OUTPUT_ro
#define PACMOD12_TORQUE_OUTPUT_ro_CovFactor (0.001000)
#define PACMOD12_TORQUE_OUTPUT_ro_toS(x) ( (int32_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_TORQUE_OUTPUT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @TORQUE_INPUT_ro
#define PACMOD12_TORQUE_INPUT_ro_CovFactor (0.001000)
#define PACMOD12_TORQUE_INPUT_ro_toS(x) ( (int32_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_TORQUE_INPUT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  int32_t TORQUE_OUTPUT_ro;                  //  [-] Bits=32 Factor= 0.001000        Unit:'N-m'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t TORQUE_OUTPUT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int32_t TORQUE_INPUT_ro;                   //  [-] Bits=32 Factor= 0.001000        Unit:'N-m'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t TORQUE_INPUT_phys;
#endif // PACMOD12_USE_SIGFLOAT

#else

  int32_t TORQUE_OUTPUT_ro;                  //  [-] Bits=32 Factor= 0.001000        Unit:'N-m'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t TORQUE_OUTPUT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int32_t TORQUE_INPUT_ro;                   //  [-] Bits=32 Factor= 0.001000        Unit:'N-m'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t TORQUE_INPUT_phys;
#endif // PACMOD12_USE_SIGFLOAT

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} STEERING_MOTOR_RPT_3_t;

// def @WHEEL_SPEED_RPT CAN Message (1031 0x407)
#define WHEEL_SPEED_RPT_IDE (0U)
#define WHEEL_SPEED_RPT_DLC (8U)
#define WHEEL_SPEED_RPT_CANID (0x407)
// signal: @WHEEL_SPD_AXLE_1_LEFT_ro
#define PACMOD12_WHEEL_SPD_AXLE_1_LEFT_ro_CovFactor (0.010000)
#define PACMOD12_WHEEL_SPD_AXLE_1_LEFT_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.010000)) )
#define PACMOD12_WHEEL_SPD_AXLE_1_LEFT_ro_fromS(x) ( (((x) * (0.010000)) + (0.000000)) )
// signal: @WHEEL_SPD_AXLE_1_RIGHT_ro
#define PACMOD12_WHEEL_SPD_AXLE_1_RIGHT_ro_CovFactor (0.010000)
#define PACMOD12_WHEEL_SPD_AXLE_1_RIGHT_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.010000)) )
#define PACMOD12_WHEEL_SPD_AXLE_1_RIGHT_ro_fromS(x) ( (((x) * (0.010000)) + (0.000000)) )
// signal: @WHEEL_SPD_AXLE_2_LEFT_ro
#define PACMOD12_WHEEL_SPD_AXLE_2_LEFT_ro_CovFactor (0.010000)
#define PACMOD12_WHEEL_SPD_AXLE_2_LEFT_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.010000)) )
#define PACMOD12_WHEEL_SPD_AXLE_2_LEFT_ro_fromS(x) ( (((x) * (0.010000)) + (0.000000)) )
// signal: @WHEEL_SPD_AXLE_2_RIGHT_ro
#define PACMOD12_WHEEL_SPD_AXLE_2_RIGHT_ro_CovFactor (0.010000)
#define PACMOD12_WHEEL_SPD_AXLE_2_RIGHT_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.010000)) )
#define PACMOD12_WHEEL_SPD_AXLE_2_RIGHT_ro_fromS(x) ( (((x) * (0.010000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  int16_t WHEEL_SPD_AXLE_1_LEFT_ro;          //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t WHEEL_SPD_AXLE_1_LEFT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int16_t WHEEL_SPD_AXLE_1_RIGHT_ro;         //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t WHEEL_SPD_AXLE_1_RIGHT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int16_t WHEEL_SPD_AXLE_2_LEFT_ro;          //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t WHEEL_SPD_AXLE_2_LEFT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int16_t WHEEL_SPD_AXLE_2_RIGHT_ro;         //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t WHEEL_SPD_AXLE_2_RIGHT_phys;
#endif // PACMOD12_USE_SIGFLOAT

#else

  int16_t WHEEL_SPD_AXLE_1_LEFT_ro;          //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t WHEEL_SPD_AXLE_1_LEFT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int16_t WHEEL_SPD_AXLE_1_RIGHT_ro;         //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t WHEEL_SPD_AXLE_1_RIGHT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int16_t WHEEL_SPD_AXLE_2_LEFT_ro;          //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t WHEEL_SPD_AXLE_2_LEFT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int16_t WHEEL_SPD_AXLE_2_RIGHT_ro;         //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t WHEEL_SPD_AXLE_2_RIGHT_phys;
#endif // PACMOD12_USE_SIGFLOAT

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} WHEEL_SPEED_RPT_t;

// def @SOFTWARE_VERSION_RPT_00 CAN Message (1032 0x408)
#define SOFTWARE_VERSION_RPT_00_IDE (0U)
#define SOFTWARE_VERSION_RPT_00_DLC (7U)
#define SOFTWARE_VERSION_RPT_00_CANID (0x408)
#define SOFTWARE_VERSION_RPT_00_CYC (1000U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MAJOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MINOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t PATCH;                             //      Bits= 8

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD0;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD1;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD2;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD3;                            //      Bits= 8 Unit:'ASCII'

#else

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MAJOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MINOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t PATCH;                             //      Bits= 8

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD0;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD1;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD2;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD3;                            //      Bits= 8 Unit:'ASCII'

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} SOFTWARE_VERSION_RPT_00_t;

// def @SOFTWARE_VERSION_RPT_01 CAN Message (1033 0x409)
#define SOFTWARE_VERSION_RPT_01_IDE (0U)
#define SOFTWARE_VERSION_RPT_01_DLC (7U)
#define SOFTWARE_VERSION_RPT_01_CANID (0x409)
#define SOFTWARE_VERSION_RPT_01_CYC (1000U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MAJOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MINOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t PATCH;                             //      Bits= 8

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD0;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD1;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD2;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD3;                            //      Bits= 8 Unit:'ASCII'

#else

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MAJOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MINOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t PATCH;                             //      Bits= 8

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD0;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD1;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD2;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD3;                            //      Bits= 8 Unit:'ASCII'

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} SOFTWARE_VERSION_RPT_01_t;

// def @SOFTWARE_VERSION_RPT_02 CAN Message (1034 0x40a)
#define SOFTWARE_VERSION_RPT_02_IDE (0U)
#define SOFTWARE_VERSION_RPT_02_DLC (7U)
#define SOFTWARE_VERSION_RPT_02_CANID (0x40a)
#define SOFTWARE_VERSION_RPT_02_CYC (1000U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MAJOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MINOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t PATCH;                             //      Bits= 8

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD0;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD1;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD2;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD3;                            //      Bits= 8 Unit:'ASCII'

#else

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MAJOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MINOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t PATCH;                             //      Bits= 8

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD0;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD1;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD2;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD3;                            //      Bits= 8 Unit:'ASCII'

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} SOFTWARE_VERSION_RPT_02_t;

// def @SOFTWARE_VERSION_RPT_03 CAN Message (1035 0x40b)
#define SOFTWARE_VERSION_RPT_03_IDE (0U)
#define SOFTWARE_VERSION_RPT_03_DLC (7U)
#define SOFTWARE_VERSION_RPT_03_CANID (0x40b)
#define SOFTWARE_VERSION_RPT_03_CYC (1000U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MAJOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MINOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t PATCH;                             //      Bits= 8

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD0;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD1;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD2;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD3;                            //      Bits= 8 Unit:'ASCII'

#else

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MAJOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MINOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t PATCH;                             //      Bits= 8

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD0;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD1;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD2;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD3;                            //      Bits= 8 Unit:'ASCII'

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} SOFTWARE_VERSION_RPT_03_t;

// def @SOFTWARE_VERSION_RPT_04 CAN Message (1036 0x40c)
#define SOFTWARE_VERSION_RPT_04_IDE (0U)
#define SOFTWARE_VERSION_RPT_04_DLC (7U)
#define SOFTWARE_VERSION_RPT_04_CANID (0x40c)
#define SOFTWARE_VERSION_RPT_04_CYC (1000U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MAJOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MINOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t PATCH;                             //      Bits= 8

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD0;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD1;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD2;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD3;                            //      Bits= 8 Unit:'ASCII'

#else

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MAJOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MINOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t PATCH;                             //      Bits= 8

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD0;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD1;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD2;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD3;                            //      Bits= 8 Unit:'ASCII'

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} SOFTWARE_VERSION_RPT_04_t;

// Not for new development.
// def @YAW_RATE_RPT CAN Message (1037 0x40d)
#define YAW_RATE_RPT_IDE (0U)
#define YAW_RATE_RPT_DLC (2U)
#define YAW_RATE_RPT_CANID (0x40d)
// signal: @YAW_RATE_ro
#define PACMOD12_YAW_RATE_ro_CovFactor (0.010000)
#define PACMOD12_YAW_RATE_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.010000)) )
#define PACMOD12_YAW_RATE_ro_fromS(x) ( (((x) * (0.010000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  int16_t YAW_RATE_ro;                       //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t YAW_RATE_phys;
#endif // PACMOD12_USE_SIGFLOAT

#else

  int16_t YAW_RATE_ro;                       //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t YAW_RATE_phys;
#endif // PACMOD12_USE_SIGFLOAT

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} YAW_RATE_RPT_t;

// def @LAT_LON_HEADING_RPT CAN Message (1038 0x40e)
#define LAT_LON_HEADING_RPT_IDE (0U)
#define LAT_LON_HEADING_RPT_DLC (8U)
#define LAT_LON_HEADING_RPT_CANID (0x40e)
// signal: @HEADING_ro
#define PACMOD12_HEADING_ro_CovFactor (0.010000)
#define PACMOD12_HEADING_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.010000)) )
#define PACMOD12_HEADING_ro_fromS(x) ( (((x) * (0.010000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  int8_t LATITUDE_DEGREES;                   //  [-] Bits= 8 Unit:'deg'

  int8_t LATITUDE_MINUTES;                   //  [-] Bits= 8 Unit:'min'

  int8_t LATITUDE_SECONDS;                   //  [-] Bits= 8 Unit:'sec'

  int8_t LONGITUDE_DEGREES;                  //  [-] Bits= 8 Unit:'deg'

  int8_t LONGITUDE_MINUTES;                  //  [-] Bits= 8 Unit:'min'

  int8_t LONGITUDE_SECONDS;                  //  [-] Bits= 8 Unit:'sec'

  int16_t HEADING_ro;                        //  [-] Bits=16 Factor= 0.010000        Unit:'deg'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t HEADING_phys;
#endif // PACMOD12_USE_SIGFLOAT

#else

  int8_t LATITUDE_DEGREES;                   //  [-] Bits= 8 Unit:'deg'

  int8_t LATITUDE_MINUTES;                   //  [-] Bits= 8 Unit:'min'

  int8_t LATITUDE_SECONDS;                   //  [-] Bits= 8 Unit:'sec'

  int8_t LONGITUDE_DEGREES;                  //  [-] Bits= 8 Unit:'deg'

  int8_t LONGITUDE_MINUTES;                  //  [-] Bits= 8 Unit:'min'

  int8_t LONGITUDE_SECONDS;                  //  [-] Bits= 8 Unit:'sec'

  int16_t HEADING_ro;                        //  [-] Bits=16 Factor= 0.010000        Unit:'deg'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t HEADING_phys;
#endif // PACMOD12_USE_SIGFLOAT

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} LAT_LON_HEADING_RPT_t;

// def @DATE_TIME_RPT CAN Message (1039 0x40f)
#define DATE_TIME_RPT_IDE (0U)
#define DATE_TIME_RPT_DLC (6U)
#define DATE_TIME_RPT_CANID (0x40f)
#define DATE_TIME_RPT_CYC (250U)
// signal: @DATE_YEAR_ro
#define PACMOD12_DATE_YEAR_ro_CovFactor (1)
#define PACMOD12_DATE_YEAR_ro_toS(x) ( (uint16_t) ((x) - (2000)) )
#define PACMOD12_DATE_YEAR_ro_fromS(x) ( ((x) + (2000)) )
// signal: @DATE_MONTH_ro
#define PACMOD12_DATE_MONTH_ro_CovFactor (1)
#define PACMOD12_DATE_MONTH_ro_toS(x) ( (uint16_t) ((x) - (1)) )
#define PACMOD12_DATE_MONTH_ro_fromS(x) ( ((x) + (1)) )
// signal: @DATE_DAY_ro
#define PACMOD12_DATE_DAY_ro_CovFactor (1)
#define PACMOD12_DATE_DAY_ro_toS(x) ( (uint16_t) ((x) - (1)) )
#define PACMOD12_DATE_DAY_ro_fromS(x) ( ((x) + (1)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  uint16_t DATE_YEAR_ro;                     //      Bits= 8 Offset= 2000               Unit:'yr'

#ifdef PACMOD12_USE_SIGFLOAT
  uint16_t DATE_YEAR_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t DATE_MONTH_ro;                    //      Bits= 8 Offset= 1                  Unit:'mon'

#ifdef PACMOD12_USE_SIGFLOAT
  uint16_t DATE_MONTH_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t DATE_DAY_ro;                      //      Bits= 8 Offset= 1                  Unit:'dy'

#ifdef PACMOD12_USE_SIGFLOAT
  uint16_t DATE_DAY_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint8_t TIME_HOUR;                         //      Bits= 8 Unit:'hr'

  uint8_t TIME_MINUTE;                       //      Bits= 8 Unit:'min'

  uint8_t TIME_SECOND;                       //      Bits= 8 Unit:'sec'

#else

  uint16_t DATE_YEAR_ro;                     //      Bits= 8 Offset= 2000               Unit:'yr'

#ifdef PACMOD12_USE_SIGFLOAT
  uint16_t DATE_YEAR_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t DATE_MONTH_ro;                    //      Bits= 8 Offset= 1                  Unit:'mon'

#ifdef PACMOD12_USE_SIGFLOAT
  uint16_t DATE_MONTH_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t DATE_DAY_ro;                      //      Bits= 8 Offset= 1                  Unit:'dy'

#ifdef PACMOD12_USE_SIGFLOAT
  uint16_t DATE_DAY_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint8_t TIME_HOUR;                         //      Bits= 8 Unit:'hr'

  uint8_t TIME_MINUTE;                       //      Bits= 8 Unit:'min'

  uint8_t TIME_SECOND;                       //      Bits= 8 Unit:'sec'

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} DATE_TIME_RPT_t;

// def @ENGINE_RPT CAN Message (1040 0x410)
#define ENGINE_RPT_IDE (0U)
#define ENGINE_RPT_DLC (7U)
#define ENGINE_RPT_CANID (0x410)
// signal: @ENGINE_SPEED_ro
#define PACMOD12_ENGINE_SPEED_ro_CovFactor (0.250000)
#define PACMOD12_ENGINE_SPEED_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.250000)) )
#define PACMOD12_ENGINE_SPEED_ro_fromS(x) ( (((x) * (0.250000)) + (0.000000)) )
// signal: @ENGINE_TORQUE_ro
#define PACMOD12_ENGINE_TORQUE_ro_CovFactor (0.062500)
#define PACMOD12_ENGINE_TORQUE_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.062500)) )
#define PACMOD12_ENGINE_TORQUE_ro_fromS(x) ( (((x) * (0.062500)) + (0.000000)) )
// signal: @ENGINE_COOLANT_TEMP_ro
#define PACMOD12_ENGINE_COOLANT_TEMP_ro_CovFactor (1)
#define PACMOD12_ENGINE_COOLANT_TEMP_ro_toS(x) ( (int16_t) ((x) - (-40)) )
#define PACMOD12_ENGINE_COOLANT_TEMP_ro_fromS(x) ( ((x) + (-40)) )
// signal: @FUEL_LEVEL_ro
#define PACMOD12_FUEL_LEVEL_ro_CovFactor (0.005000)
#define PACMOD12_FUEL_LEVEL_ro_toS(x) ( (uint8_t) (((x) - (0.000000)) / (0.005000)) )
#define PACMOD12_FUEL_LEVEL_ro_fromS(x) ( (((x) * (0.005000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  uint16_t ENGINE_SPEED_ro;                  //      Bits=16 Factor= 0.250000        Unit:'rpm'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t ENGINE_SPEED_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t ENGINE_TORQUE_ro;                 //      Bits=16 Factor= 0.062500        Unit:'N-m'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t ENGINE_TORQUE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int16_t ENGINE_COOLANT_TEMP_ro;            //      Bits= 8 Offset= -40                Unit:'deg_C'

#ifdef PACMOD12_USE_SIGFLOAT
  int16_t ENGINE_COOLANT_TEMP_phys;
#endif // PACMOD12_USE_SIGFLOAT

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t ENGINE_SPEED_AVAIL : 1;            //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t ENGINE_TORQUE_AVAIL : 1;           //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t ENGINE_COOLANT_TEMP_AVAIL : 1;     //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t FUEL_LEVEL_AVAIL : 1;              //      Bits= 1

  uint8_t FUEL_LEVEL_ro;                     //      Bits= 8 Factor= 0.005000        Unit:'%'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t FUEL_LEVEL_phys;
#endif // PACMOD12_USE_SIGFLOAT

#else

  uint16_t ENGINE_SPEED_ro;                  //      Bits=16 Factor= 0.250000        Unit:'rpm'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t ENGINE_SPEED_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t ENGINE_TORQUE_ro;                 //      Bits=16 Factor= 0.062500        Unit:'N-m'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t ENGINE_TORQUE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int16_t ENGINE_COOLANT_TEMP_ro;            //      Bits= 8 Offset= -40                Unit:'deg_C'

#ifdef PACMOD12_USE_SIGFLOAT
  int16_t ENGINE_COOLANT_TEMP_phys;
#endif // PACMOD12_USE_SIGFLOAT

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t ENGINE_SPEED_AVAIL;                //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t ENGINE_TORQUE_AVAIL;               //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t ENGINE_COOLANT_TEMP_AVAIL;         //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t FUEL_LEVEL_AVAIL;                  //      Bits= 1

  uint8_t FUEL_LEVEL_ro;                     //      Bits= 8 Factor= 0.005000        Unit:'%'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t FUEL_LEVEL_phys;
#endif // PACMOD12_USE_SIGFLOAT

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} ENGINE_RPT_t;

// def @DETECTED_OBJECT_RPT CAN Message (1041 0x411)
#define DETECTED_OBJECT_RPT_IDE (0U)
#define DETECTED_OBJECT_RPT_DLC (6U)
#define DETECTED_OBJECT_RPT_CANID (0x411)
// signal: @FRONT_OBJECT_DISTANCE_LOW_RES_ro
#define PACMOD12_FRONT_OBJECT_DISTANCE_LOW_RES_ro_CovFactor (0.001000)
#define PACMOD12_FRONT_OBJECT_DISTANCE_LOW_RES_ro_toS(x) ( (uint32_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_FRONT_OBJECT_DISTANCE_LOW_RES_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @FRONT_OBJECT_DISTANCE_HIGH_RES_ro
#define PACMOD12_FRONT_OBJECT_DISTANCE_HIGH_RES_ro_CovFactor (0.001000)
#define PACMOD12_FRONT_OBJECT_DISTANCE_HIGH_RES_ro_toS(x) ( (uint32_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_FRONT_OBJECT_DISTANCE_HIGH_RES_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  uint32_t FRONT_OBJECT_DISTANCE_LOW_RES_ro;       //      Bits=24 Factor= 0.001000        Unit:'m'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t FRONT_OBJECT_DISTANCE_LOW_RES_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint32_t FRONT_OBJECT_DISTANCE_HIGH_RES_ro;      //      Bits=24 Factor= 0.001000        Unit:'m'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t FRONT_OBJECT_DISTANCE_HIGH_RES_phys;
#endif // PACMOD12_USE_SIGFLOAT

#else

  uint32_t FRONT_OBJECT_DISTANCE_LOW_RES_ro;       //      Bits=24 Factor= 0.001000        Unit:'m'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t FRONT_OBJECT_DISTANCE_LOW_RES_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint32_t FRONT_OBJECT_DISTANCE_HIGH_RES_ro;      //      Bits=24 Factor= 0.001000        Unit:'m'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t FRONT_OBJECT_DISTANCE_HIGH_RES_phys;
#endif // PACMOD12_USE_SIGFLOAT

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} DETECTED_OBJECT_RPT_t;

// def @VEH_DYNAMICS_RPT CAN Message (1043 0x413)
#define VEH_DYNAMICS_RPT_IDE (0U)
#define VEH_DYNAMICS_RPT_DLC (1U)
#define VEH_DYNAMICS_RPT_CANID (0x413)
// signal: @VEH_G_FORCES_ro
#define PACMOD12_VEH_G_FORCES_ro_CovFactor (0.001000)
#define PACMOD12_VEH_G_FORCES_ro_toS(x) ( (uint8_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_VEH_G_FORCES_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  uint8_t VEH_G_FORCES_ro;                   //      Bits= 8 Factor= 0.001000        Unit:'m/s^2'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t VEH_G_FORCES_phys;
#endif // PACMOD12_USE_SIGFLOAT

#else

  uint8_t VEH_G_FORCES_ro;                   //      Bits= 8 Factor= 0.001000        Unit:'m/s^2'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t VEH_G_FORCES_phys;
#endif // PACMOD12_USE_SIGFLOAT

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} VEH_DYNAMICS_RPT_t;

// Not for new development.
// def @VIN_RPT CAN Message (1044 0x414)
#define VIN_RPT_IDE (0U)
#define VIN_RPT_DLC (7U)
#define VIN_RPT_CANID (0x414)
#define VIN_RPT_CYC (1000U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  uint32_t VEH_MFG_CODE;                     //      Bits=24

  uint8_t VEH_MY_CODE;                       //      Bits= 8

  uint32_t VEH_SERIAL;                       //      Bits=24

#else

  uint32_t VEH_MFG_CODE;                     //      Bits=24

  uint8_t VEH_MY_CODE;                       //      Bits= 8

  uint32_t VEH_SERIAL;                       //      Bits=24

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} VIN_RPT_t;

// def @OCCUPANCY_RPT CAN Message (1045 0x415)
#define OCCUPANCY_RPT_IDE (0U)
#define OCCUPANCY_RPT_DLC (2U)
#define OCCUPANCY_RPT_CANID (0x415)
#define OCCUPANCY_RPT_CYC (250U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "NOT_OCCUPIED"
  //  1 : "OCCUPIED"
  // 
  uint8_t DRIVER_SEAT_OCCUPIED : 1;               //      Bits= 1

  //  0 : "NOT_OCCUPIED"
  //  1 : "OCCUPIED"
  // 
  uint8_t PASS_SEAT_OCCUPIED : 1;                 //      Bits= 1

  //  0 : "NOT_OCCUPIED"
  //  1 : "OCCUPIED"
  // 
  uint8_t REAR_SEAT_OCCUPIED : 1;                 //      Bits= 1

  //  0 : "NOT_BUCKLED"
  //  1 : "BUCKLED"
  // 
  uint8_t DRIVER_SEATBELT_BUCKLED : 1;            //      Bits= 1

  //  0 : "NOT_BUCKLED"
  //  1 : "BUCKLED"
  // 
  uint8_t PASS_SEATBELT_BUCKLED : 1;              //      Bits= 1

  //  0 : "NOT_BUCKLED"
  //  1 : "BUCKLED"
  // 
  uint8_t DRVR_REAR_SEATBELT_BUCKLED : 1;         //      Bits= 1

  //  0 : "NOT_BUCKLED"
  //  1 : "BUCKLED"
  // 
  uint8_t PASS_REAR_SEATBELT_BUCKLED : 1;         //      Bits= 1

  //  0 : "NOT_BUCKLED"
  //  1 : "BUCKLED"
  // 
  uint8_t CTR_REAR_SEATBELT_BUCKLED : 1;          //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t DRIVER_SEAT_OCCUPIED_AVAIL : 1;         //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t PASS_SEAT_OCCUPIED_AVAIL : 1;           //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t REAR_SEAT_OCCUPIED_AVAIL : 1;           //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t DRIVER_SEATBELT_BUCKLED_AVAIL : 1;      //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t PASS_SEATBELT_BUCKLED_AVAIL : 1;        //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t DRVR_REAR_SEATBELT_BUCKLED_AVAIL : 1;   //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t PASS_REAR_SEATBELT_BUCKLED_AVAIL : 1;   //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t CTR_REAR_SEATBELT_BUCKLED_AVAIL : 1;    //      Bits= 1

#else

  //  0 : "NOT_OCCUPIED"
  //  1 : "OCCUPIED"
  // 
  uint8_t DRIVER_SEAT_OCCUPIED;                   //      Bits= 1

  //  0 : "NOT_OCCUPIED"
  //  1 : "OCCUPIED"
  // 
  uint8_t PASS_SEAT_OCCUPIED;                     //      Bits= 1

  //  0 : "NOT_OCCUPIED"
  //  1 : "OCCUPIED"
  // 
  uint8_t REAR_SEAT_OCCUPIED;                     //      Bits= 1

  //  0 : "NOT_BUCKLED"
  //  1 : "BUCKLED"
  // 
  uint8_t DRIVER_SEATBELT_BUCKLED;                //      Bits= 1

  //  0 : "NOT_BUCKLED"
  //  1 : "BUCKLED"
  // 
  uint8_t PASS_SEATBELT_BUCKLED;                  //      Bits= 1

  //  0 : "NOT_BUCKLED"
  //  1 : "BUCKLED"
  // 
  uint8_t DRVR_REAR_SEATBELT_BUCKLED;             //      Bits= 1

  //  0 : "NOT_BUCKLED"
  //  1 : "BUCKLED"
  // 
  uint8_t PASS_REAR_SEATBELT_BUCKLED;             //      Bits= 1

  //  0 : "NOT_BUCKLED"
  //  1 : "BUCKLED"
  // 
  uint8_t CTR_REAR_SEATBELT_BUCKLED;              //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t DRIVER_SEAT_OCCUPIED_AVAIL;             //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t PASS_SEAT_OCCUPIED_AVAIL;               //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t REAR_SEAT_OCCUPIED_AVAIL;               //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t DRIVER_SEATBELT_BUCKLED_AVAIL;          //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t PASS_SEATBELT_BUCKLED_AVAIL;            //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t DRVR_REAR_SEATBELT_BUCKLED_AVAIL;       //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t PASS_REAR_SEATBELT_BUCKLED_AVAIL;       //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t CTR_REAR_SEATBELT_BUCKLED_AVAIL;        //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} OCCUPANCY_RPT_t;

// def @INTERIOR_LIGHTS_RPT CAN Message (1046 0x416)
#define INTERIOR_LIGHTS_RPT_IDE (0U)
#define INTERIOR_LIGHTS_RPT_DLC (3U)
#define INTERIOR_LIGHTS_RPT_CANID (0x416)
#define INTERIOR_LIGHTS_RPT_CYC (250U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t FRONT_DOME_LIGHTS_ON : 1;          //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t REAR_DOME_LIGHTS_ON : 1;           //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t MOOD_LIGHTS_ON : 1;                //      Bits= 1

  //  0 : "DARK"
  //  1 : "BRIGHT"
  // 
  uint8_t AMBIENT_LIGHT_SENSOR : 1;          //      Bits= 1

  // Lowest dim level assigned to DIM_LEVEL_MIN, the dimmest light. Highest dim level assigned to DIM_LEVEL_MAX, the brightest light. Dim levels in between start with DIM_LEVEL_1.
  //  24 : "DIM_LEVEL_23"
  //  23 : "DIM_LEVEL_22"
  //  22 : "DIM_LEVEL_21"
  //  21 : "DIM_LEVEL_20"
  //  20 : "DIM_LEVEL_19"
  //  19 : "DIM_LEVEL_18"
  //  18 : "DIM_LEVEL_17"
  //  17 : "DIM_LEVEL_16"
  //  16 : "DIM_LEVEL_15"
  //  15 : "DIM_LEVEL_14"
  //  14 : "DIM_LEVEL_13"
  //  13 : "DIM_LEVEL_12"
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
  uint8_t DIM_LEVEL;                         //      Bits= 8

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t FRONT_DOME_LIGHTS_ON_AVAIL : 1;    //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t REAR_DOME_LIGHTS_ON_AVAIL : 1;     //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t MOOD_LIGHTS_ON_AVAIL : 1;          //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t DIM_LEVEL_AVAIL : 1;               //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t AMBIENT_LIGHT_SENSOR_AVAIL : 1;    //      Bits= 1

#else

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t FRONT_DOME_LIGHTS_ON;              //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t REAR_DOME_LIGHTS_ON;               //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t MOOD_LIGHTS_ON;                    //      Bits= 1

  //  0 : "DARK"
  //  1 : "BRIGHT"
  // 
  uint8_t AMBIENT_LIGHT_SENSOR;              //      Bits= 1

  // Lowest dim level assigned to DIM_LEVEL_MIN, the dimmest light. Highest dim level assigned to DIM_LEVEL_MAX, the brightest light. Dim levels in between start with DIM_LEVEL_1.
  //  24 : "DIM_LEVEL_23"
  //  23 : "DIM_LEVEL_22"
  //  22 : "DIM_LEVEL_21"
  //  21 : "DIM_LEVEL_20"
  //  20 : "DIM_LEVEL_19"
  //  19 : "DIM_LEVEL_18"
  //  18 : "DIM_LEVEL_17"
  //  17 : "DIM_LEVEL_16"
  //  16 : "DIM_LEVEL_15"
  //  15 : "DIM_LEVEL_14"
  //  14 : "DIM_LEVEL_13"
  //  13 : "DIM_LEVEL_12"
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
  uint8_t DIM_LEVEL;                         //      Bits= 8

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t FRONT_DOME_LIGHTS_ON_AVAIL;        //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t REAR_DOME_LIGHTS_ON_AVAIL;         //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t MOOD_LIGHTS_ON_AVAIL;              //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t DIM_LEVEL_AVAIL;                   //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t AMBIENT_LIGHT_SENSOR_AVAIL;        //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} INTERIOR_LIGHTS_RPT_t;

// def @DOOR_RPT CAN Message (1047 0x417)
#define DOOR_RPT_IDE (0U)
#define DOOR_RPT_DLC (2U)
#define DOOR_RPT_CANID (0x417)
#define DOOR_RPT_CYC (250U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "CLOSED"
  //  1 : "OPEN"
  // 
  uint8_t DRIVER_DOOR_OPEN : 1;              //      Bits= 1

  //  0 : "CLOSED"
  //  1 : "OPEN"
  // 
  uint8_t PASS_DOOR_OPEN : 1;                //      Bits= 1

  //  0 : "CLOSED"
  //  1 : "OPEN"
  // 
  uint8_t REAR_DRIVER_DOOR_OPEN : 1;         //      Bits= 1

  //  0 : "CLOSED"
  //  1 : "OPEN"
  // 
  uint8_t REAR_PASS_DOOR_OPEN : 1;           //      Bits= 1

  //  0 : "CLOSED"
  //  1 : "OPEN"
  // 
  uint8_t HOOD_OPEN : 1;                     //      Bits= 1

  //  0 : "CLOSED"
  //  1 : "OPEN"
  // 
  uint8_t TRUNK_OPEN : 1;                    //      Bits= 1

  //  0 : "CLOSED"
  //  1 : "OPEN"
  // 
  uint8_t FUEL_DOOR_OPEN : 1;                //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t DRIVER_DOOR_OPEN_AVAIL : 1;        //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t PASS_DOOR_OPEN_AVAIL : 1;          //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t REAR_DRIVER_DOOR_OPEN_AVAIL : 1;   //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t REAR_PASS_DOOR_OPEN_AVAIL : 1;     //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t HOOD_OPEN_AVAIL : 1;               //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t TRUNK_OPEN_AVAIL : 1;              //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t FUEL_DOOR_OPEN_AVAIL : 1;          //      Bits= 1

#else

  //  0 : "CLOSED"
  //  1 : "OPEN"
  // 
  uint8_t DRIVER_DOOR_OPEN;                  //      Bits= 1

  //  0 : "CLOSED"
  //  1 : "OPEN"
  // 
  uint8_t PASS_DOOR_OPEN;                    //      Bits= 1

  //  0 : "CLOSED"
  //  1 : "OPEN"
  // 
  uint8_t REAR_DRIVER_DOOR_OPEN;             //      Bits= 1

  //  0 : "CLOSED"
  //  1 : "OPEN"
  // 
  uint8_t REAR_PASS_DOOR_OPEN;               //      Bits= 1

  //  0 : "CLOSED"
  //  1 : "OPEN"
  // 
  uint8_t HOOD_OPEN;                         //      Bits= 1

  //  0 : "CLOSED"
  //  1 : "OPEN"
  // 
  uint8_t TRUNK_OPEN;                        //      Bits= 1

  //  0 : "CLOSED"
  //  1 : "OPEN"
  // 
  uint8_t FUEL_DOOR_OPEN;                    //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t DRIVER_DOOR_OPEN_AVAIL;            //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t PASS_DOOR_OPEN_AVAIL;              //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t REAR_DRIVER_DOOR_OPEN_AVAIL;       //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t REAR_PASS_DOOR_OPEN_AVAIL;         //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t HOOD_OPEN_AVAIL;                   //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t TRUNK_OPEN_AVAIL;                  //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t FUEL_DOOR_OPEN_AVAIL;              //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} DOOR_RPT_t;

// def @REAR_LIGHTS_RPT CAN Message (1048 0x418)
#define REAR_LIGHTS_RPT_IDE (0U)
#define REAR_LIGHTS_RPT_DLC (2U)
#define REAR_LIGHTS_RPT_CANID (0x418)
#define REAR_LIGHTS_RPT_CYC (250U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t BRAKE_LIGHTS_ON : 1;               //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t REVERSE_LIGHTS_ON : 1;             //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t BRAKE_LIGHTS_ON_AVAIL : 1;         //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t REVERSE_LIGHTS_ON_AVAIL : 1;       //      Bits= 1

#else

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t BRAKE_LIGHTS_ON;                   //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  // 
  uint8_t REVERSE_LIGHTS_ON;                 //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t BRAKE_LIGHTS_ON_AVAIL;             //      Bits= 1

  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t REVERSE_LIGHTS_ON_AVAIL;           //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} REAR_LIGHTS_RPT_t;

// def @LINEAR_ACCEL_RPT CAN Message (1049 0x419)
#define LINEAR_ACCEL_RPT_IDE (0U)
#define LINEAR_ACCEL_RPT_DLC (7U)
#define LINEAR_ACCEL_RPT_CANID (0x419)
// signal: @LATERAL_ACCEL_ro
#define PACMOD12_LATERAL_ACCEL_ro_CovFactor (0.010000)
#define PACMOD12_LATERAL_ACCEL_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.010000)) )
#define PACMOD12_LATERAL_ACCEL_ro_fromS(x) ( (((x) * (0.010000)) + (0.000000)) )
// signal: @LONGITUDINAL_ACCEL_ro
#define PACMOD12_LONGITUDINAL_ACCEL_ro_CovFactor (0.010000)
#define PACMOD12_LONGITUDINAL_ACCEL_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.010000)) )
#define PACMOD12_LONGITUDINAL_ACCEL_ro_fromS(x) ( (((x) * (0.010000)) + (0.000000)) )
// signal: @VERTICAL_ACCEL_ro
#define PACMOD12_VERTICAL_ACCEL_ro_CovFactor (0.010000)
#define PACMOD12_VERTICAL_ACCEL_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.010000)) )
#define PACMOD12_VERTICAL_ACCEL_ro_fromS(x) ( (((x) * (0.010000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "NEW_DATA_NOT_RX"
  //  1 : "NEW_DATA_RX"
  // 
  uint8_t LATERAL_NEW_DATA_RX : 1;           //      Bits= 1

  //  0 : "NEW_DATA_NOT_RX"
  //  1 : "NEW_DATA_RX"
  // 
  uint8_t LONGITUDINAL_NEW_DATA_RX : 1;      //      Bits= 1

  //  0 : "NEW_DATA_NOT_RX"
  //  1 : "NEW_DATA_RX"
  // 
  uint8_t VERTICAL_NEW_DATA_RX : 1;          //      Bits= 1

  //  0 : "NOT_VALID"
  //  1 : "VALID"
  // 
  uint8_t LATERAL_VALID : 1;                 //      Bits= 1

  //  0 : "NOT_VALID"
  //  1 : "VALID"
  // 
  uint8_t LONGITUDINAL_VALID : 1;            //      Bits= 1

  //  0 : "NOT_VALID"
  //  1 : "VALID"
  // 
  uint8_t VERTICAL_VALID : 1;                //      Bits= 1

  int16_t LATERAL_ACCEL_ro;                  //  [-] Bits=16 Factor= 0.010000        Unit:'m/s^2'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t LATERAL_ACCEL_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int16_t LONGITUDINAL_ACCEL_ro;             //  [-] Bits=16 Factor= 0.010000        Unit:'m/s^2'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t LONGITUDINAL_ACCEL_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int16_t VERTICAL_ACCEL_ro;                 //  [-] Bits=16 Factor= 0.010000        Unit:'m/s^2'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t VERTICAL_ACCEL_phys;
#endif // PACMOD12_USE_SIGFLOAT

#else

  //  0 : "NEW_DATA_NOT_RX"
  //  1 : "NEW_DATA_RX"
  // 
  uint8_t LATERAL_NEW_DATA_RX;               //      Bits= 1

  //  0 : "NEW_DATA_NOT_RX"
  //  1 : "NEW_DATA_RX"
  // 
  uint8_t LONGITUDINAL_NEW_DATA_RX;          //      Bits= 1

  //  0 : "NEW_DATA_NOT_RX"
  //  1 : "NEW_DATA_RX"
  // 
  uint8_t VERTICAL_NEW_DATA_RX;              //      Bits= 1

  //  0 : "NOT_VALID"
  //  1 : "VALID"
  // 
  uint8_t LATERAL_VALID;                     //      Bits= 1

  //  0 : "NOT_VALID"
  //  1 : "VALID"
  // 
  uint8_t LONGITUDINAL_VALID;                //      Bits= 1

  //  0 : "NOT_VALID"
  //  1 : "VALID"
  // 
  uint8_t VERTICAL_VALID;                    //      Bits= 1

  int16_t LATERAL_ACCEL_ro;                  //  [-] Bits=16 Factor= 0.010000        Unit:'m/s^2'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t LATERAL_ACCEL_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int16_t LONGITUDINAL_ACCEL_ro;             //  [-] Bits=16 Factor= 0.010000        Unit:'m/s^2'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t LONGITUDINAL_ACCEL_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int16_t VERTICAL_ACCEL_ro;                 //  [-] Bits=16 Factor= 0.010000        Unit:'m/s^2'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t VERTICAL_ACCEL_phys;
#endif // PACMOD12_USE_SIGFLOAT

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} LINEAR_ACCEL_RPT_t;

// def @ANG_VEL_RPT CAN Message (1050 0x41a)
#define ANG_VEL_RPT_IDE (0U)
#define ANG_VEL_RPT_DLC (7U)
#define ANG_VEL_RPT_CANID (0x41a)
// signal: @PITCH_VEL_ro
#define PACMOD12_PITCH_VEL_ro_CovFactor (0.001000)
#define PACMOD12_PITCH_VEL_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_PITCH_VEL_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @ROLL_VEL_ro
#define PACMOD12_ROLL_VEL_ro_CovFactor (0.001000)
#define PACMOD12_ROLL_VEL_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_ROLL_VEL_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @YAW_VEL_ro
#define PACMOD12_YAW_VEL_ro_CovFactor (0.001000)
#define PACMOD12_YAW_VEL_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD12_YAW_VEL_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "NEW_DATA_NOT_RX"
  //  1 : "NEW_DATA_RX"
  // 
  uint8_t PITCH_NEW_DATA_RX : 1;             //      Bits= 1

  //  0 : "NEW_DATA_NOT_RX"
  //  1 : "NEW_DATA_RX"
  // 
  uint8_t ROLL_NEW_DATA_RX : 1;              //      Bits= 1

  //  0 : "NEW_DATA_NOT_RX"
  //  1 : "NEW_DATA_RX"
  // 
  uint8_t YAW_NEW_DATA_RX : 1;               //      Bits= 1

  //  0 : "NOT_VALID"
  //  1 : "VALID"
  // 
  uint8_t PITCH_VALID : 1;                   //      Bits= 1

  //  0 : "NOT_VALID"
  //  1 : "VALID"
  // 
  uint8_t ROLL_VALID : 1;                    //      Bits= 1

  //  0 : "NOT_VALID"
  //  1 : "VALID"
  // 
  uint8_t YAW_VALID : 1;                     //      Bits= 1

  int16_t PITCH_VEL_ro;                      //  [-] Bits=16 Factor= 0.001000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t PITCH_VEL_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int16_t ROLL_VEL_ro;                       //  [-] Bits=16 Factor= 0.001000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t ROLL_VEL_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int16_t YAW_VEL_ro;                        //  [-] Bits=16 Factor= 0.001000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t YAW_VEL_phys;
#endif // PACMOD12_USE_SIGFLOAT

#else

  //  0 : "NEW_DATA_NOT_RX"
  //  1 : "NEW_DATA_RX"
  // 
  uint8_t PITCH_NEW_DATA_RX;                 //      Bits= 1

  //  0 : "NEW_DATA_NOT_RX"
  //  1 : "NEW_DATA_RX"
  // 
  uint8_t ROLL_NEW_DATA_RX;                  //      Bits= 1

  //  0 : "NEW_DATA_NOT_RX"
  //  1 : "NEW_DATA_RX"
  // 
  uint8_t YAW_NEW_DATA_RX;                   //      Bits= 1

  //  0 : "NOT_VALID"
  //  1 : "VALID"
  // 
  uint8_t PITCH_VALID;                       //      Bits= 1

  //  0 : "NOT_VALID"
  //  1 : "VALID"
  // 
  uint8_t ROLL_VALID;                        //      Bits= 1

  //  0 : "NOT_VALID"
  //  1 : "VALID"
  // 
  uint8_t YAW_VALID;                         //      Bits= 1

  int16_t PITCH_VEL_ro;                      //  [-] Bits=16 Factor= 0.001000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t PITCH_VEL_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int16_t ROLL_VEL_ro;                       //  [-] Bits=16 Factor= 0.001000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t ROLL_VEL_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int16_t YAW_VEL_ro;                        //  [-] Bits=16 Factor= 0.001000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t YAW_VEL_phys;
#endif // PACMOD12_USE_SIGFLOAT

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} ANG_VEL_RPT_t;

// def @NOTIFICATION_CMD CAN Message (1051 0x41b)
#define NOTIFICATION_CMD_IDE (0U)
#define NOTIFICATION_CMD_DLC (1U)
#define NOTIFICATION_CMD_CANID (0x41b)
#define NOTIFICATION_CMD_CYC (250U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "NOT_MUTED"
  //  1 : "MUTED"
  // 
  uint8_t BUZZER_MUTE : 1;                   //      Bits= 1

  //  0 : "NO_ACTION"
  //  1 : "WHITE"
  // 
  uint8_t UNDERDASH_LIGHTS_WHITE : 1;        //      Bits= 1

#else

  //  0 : "NOT_MUTED"
  //  1 : "MUTED"
  // 
  uint8_t BUZZER_MUTE;                       //      Bits= 1

  //  0 : "NO_ACTION"
  //  1 : "WHITE"
  // 
  uint8_t UNDERDASH_LIGHTS_WHITE;            //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} NOTIFICATION_CMD_t;

// def @ESTOP_RPT CAN Message (1052 0x41c)
#define ESTOP_RPT_IDE (0U)
#define ESTOP_RPT_DLC (1U)
#define ESTOP_RPT_CANID (0x41c)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "RELEASED"
  //  1 : "PRESSED"
  // 
  uint8_t ESTOP : 1;                         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t ESTOP_FAULT : 1;                   //      Bits= 1

#else

  //  0 : "RELEASED"
  //  1 : "PRESSED"
  // 
  uint8_t ESTOP;                             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  // 
  uint8_t ESTOP_FAULT;                       //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} ESTOP_RPT_t;

// Commands follow the list of priorities listed in user_can_protocol.md.
// def @USER_NOTIFICATION_CMD CAN Message (1053 0x41d)
#define USER_NOTIFICATION_CMD_IDE (0U)
#define USER_NOTIFICATION_CMD_DLC (2U)
#define USER_NOTIFICATION_CMD_CANID (0x41d)
#define USER_NOTIFICATION_CMD_CYC (250U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "NOT_MUTED"
  //  1 : "MUTED"
  // 
  uint8_t BUZZER_MUTE : 1;                   //      Bits= 1

  //  0 : "NO_ACTION"
  //  1 : "WHITE"
  //  2 : "RED"
  //  3 : "GREEN"
  //  4 : "BLUE"
  //  5 : "PURPLE"
  //  6 : "ORANGE"
  //  7 : "YELLOW"
  //  8 : "TEAL"
  //  9 : "PINK"
  //  10 : "OFF"
  // 
  uint8_t LIGHT_COMMAND : 4;                 //      Bits= 4

  //  0 : "NO_ACTION"
  //  1 : "BUZZER_ON"
  // 
  uint8_t BUZZER_ON : 1;                     //      Bits= 1

  //  0 : "ENABLED"
  //  1 : "DISABLED"
  // 
  uint8_t BUZZER_MUTE_INDICATOR : 2;         //      Bits= 2

  //  0 : "NO_ACTION"
  //  1 : "MIN_BRIGHTNESS"
  //  2 : "LEVEL_1"
  //  3 : "LEVEL_2"
  //  4 : "LEVEL_3"
  //  5 : "LEVEL_4"
  //  6 : "LEVEL_5"
  //  7 : "LEVEL_6"
  //  8 : "LEVEL_7"
  //  9 : "LEVEL_8"
  //  10 : "LEVEL_9"
  //  11 : "LEVEL_10"
  //  12 : "LEVEL_11"
  //  13 : "MAX_BRIGHTNESS"
  // 
  uint8_t LED_BRIGHTNESS : 4;                //      Bits= 4

#else

  //  0 : "NOT_MUTED"
  //  1 : "MUTED"
  // 
  uint8_t BUZZER_MUTE;                       //      Bits= 1

  //  0 : "NO_ACTION"
  //  1 : "WHITE"
  //  2 : "RED"
  //  3 : "GREEN"
  //  4 : "BLUE"
  //  5 : "PURPLE"
  //  6 : "ORANGE"
  //  7 : "YELLOW"
  //  8 : "TEAL"
  //  9 : "PINK"
  //  10 : "OFF"
  // 
  uint8_t LIGHT_COMMAND;                     //      Bits= 4

  //  0 : "NO_ACTION"
  //  1 : "BUZZER_ON"
  // 
  uint8_t BUZZER_ON;                         //      Bits= 1

  //  0 : "ENABLED"
  //  1 : "DISABLED"
  // 
  uint8_t BUZZER_MUTE_INDICATOR;             //      Bits= 2

  //  0 : "NO_ACTION"
  //  1 : "MIN_BRIGHTNESS"
  //  2 : "LEVEL_1"
  //  3 : "LEVEL_2"
  //  4 : "LEVEL_3"
  //  5 : "LEVEL_4"
  //  6 : "LEVEL_5"
  //  7 : "LEVEL_6"
  //  8 : "LEVEL_7"
  //  9 : "LEVEL_8"
  //  10 : "LEVEL_9"
  //  11 : "LEVEL_10"
  //  12 : "LEVEL_11"
  //  13 : "MAX_BRIGHTNESS"
  // 
  uint8_t LED_BRIGHTNESS;                    //      Bits= 4

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} USER_NOTIFICATION_CMD_t;

// def @TIRE_PRESSURE_RPT CAN Message (1054 0x41e)
#define TIRE_PRESSURE_RPT_IDE (0U)
#define TIRE_PRESSURE_RPT_DLC (4U)
#define TIRE_PRESSURE_RPT_CANID (0x41e)
#define TIRE_PRESSURE_RPT_CYC (100U)
// signal: @FRONT_LEFT_TIRE_PRESSURE_ro
#define PACMOD12_FRONT_LEFT_TIRE_PRESSURE_ro_CovFactor (4)
#define PACMOD12_FRONT_LEFT_TIRE_PRESSURE_ro_toS(x) ( (uint16_t) ((x) / (4)) )
#define PACMOD12_FRONT_LEFT_TIRE_PRESSURE_ro_fromS(x) ( ((x) * (4)) )
// signal: @FRONT_RIGHT_TIRE_PRESSURE_ro
#define PACMOD12_FRONT_RIGHT_TIRE_PRESSURE_ro_CovFactor (4)
#define PACMOD12_FRONT_RIGHT_TIRE_PRESSURE_ro_toS(x) ( (uint16_t) ((x) / (4)) )
#define PACMOD12_FRONT_RIGHT_TIRE_PRESSURE_ro_fromS(x) ( ((x) * (4)) )
// signal: @REAR_LEFT_TIRE_PRESSURE_ro
#define PACMOD12_REAR_LEFT_TIRE_PRESSURE_ro_CovFactor (4)
#define PACMOD12_REAR_LEFT_TIRE_PRESSURE_ro_toS(x) ( (uint16_t) ((x) / (4)) )
#define PACMOD12_REAR_LEFT_TIRE_PRESSURE_ro_fromS(x) ( ((x) * (4)) )
// signal: @REAR_RIGHT_TIRE_PRESSURE_ro
#define PACMOD12_REAR_RIGHT_TIRE_PRESSURE_ro_CovFactor (4)
#define PACMOD12_REAR_RIGHT_TIRE_PRESSURE_ro_toS(x) ( (uint16_t) ((x) / (4)) )
#define PACMOD12_REAR_RIGHT_TIRE_PRESSURE_ro_fromS(x) ( ((x) * (4)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  uint16_t FRONT_LEFT_TIRE_PRESSURE_ro;       //      Bits= 8 Factor= 4               Unit:'kPa'

#ifdef PACMOD12_USE_SIGFLOAT
  uint16_t FRONT_LEFT_TIRE_PRESSURE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t FRONT_RIGHT_TIRE_PRESSURE_ro;      //      Bits= 8 Factor= 4               Unit:'kPa'

#ifdef PACMOD12_USE_SIGFLOAT
  uint16_t FRONT_RIGHT_TIRE_PRESSURE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t REAR_LEFT_TIRE_PRESSURE_ro;        //      Bits= 8 Factor= 4               Unit:'kPa'

#ifdef PACMOD12_USE_SIGFLOAT
  uint16_t REAR_LEFT_TIRE_PRESSURE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t REAR_RIGHT_TIRE_PRESSURE_ro;       //      Bits= 8 Factor= 4               Unit:'kPa'

#ifdef PACMOD12_USE_SIGFLOAT
  uint16_t REAR_RIGHT_TIRE_PRESSURE_phys;
#endif // PACMOD12_USE_SIGFLOAT

#else

  uint16_t FRONT_LEFT_TIRE_PRESSURE_ro;       //      Bits= 8 Factor= 4               Unit:'kPa'

#ifdef PACMOD12_USE_SIGFLOAT
  uint16_t FRONT_LEFT_TIRE_PRESSURE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t FRONT_RIGHT_TIRE_PRESSURE_ro;      //      Bits= 8 Factor= 4               Unit:'kPa'

#ifdef PACMOD12_USE_SIGFLOAT
  uint16_t FRONT_RIGHT_TIRE_PRESSURE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t REAR_LEFT_TIRE_PRESSURE_ro;        //      Bits= 8 Factor= 4               Unit:'kPa'

#ifdef PACMOD12_USE_SIGFLOAT
  uint16_t REAR_LEFT_TIRE_PRESSURE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t REAR_RIGHT_TIRE_PRESSURE_ro;       //      Bits= 8 Factor= 4               Unit:'kPa'

#ifdef PACMOD12_USE_SIGFLOAT
  uint16_t REAR_RIGHT_TIRE_PRESSURE_phys;
#endif // PACMOD12_USE_SIGFLOAT

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} TIRE_PRESSURE_RPT_t;

// def @DRIVE_TRAIN_FEATURE_RPT CAN Message (1055 0x41f)
#define DRIVE_TRAIN_FEATURE_RPT_IDE (0U)
#define DRIVE_TRAIN_FEATURE_RPT_DLC (2U)
#define DRIVE_TRAIN_FEATURE_RPT_CANID (0x41f)
#define DRIVE_TRAIN_FEATURE_RPT_CYC (100U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "NOT_ACTIVE"
  //  1 : "ACTIVE"
  // 
  uint8_t ANTILOCK_BRAKE_ACTIVE : 1;           //      Bits= 1

  //  0 : "NOT_ACTIVE"
  //  1 : "ACTIVE"
  // 
  uint8_t TRACTION_CONTROL_ACTIVE : 1;         //      Bits= 1

  //  0 : "NOT_ACTIVE"
  //  1 : "ACTIVE"
  // 
  uint8_t FOUR_WHEEL_DRIVE_ACTIVE : 1;         //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t ANTILOCK_BRAKE_ACTIVE_AVAIL : 1;     //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t TRACTION_CONTROL_ACTIVE_AVAIL : 1;   //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t FOUR_WHEEL_DRIVE_ACTIVE_AVAIL : 1;   //      Bits= 1

  //  0 : "NORMAL"
  //  1 : "ECO"
  //  2 : "SPORT"
  // 
  uint8_t DRIVE_MODE : 4;                      //      Bits= 4

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t DRIVE_MODE_AVAIL : 1;                //      Bits= 1

#else

  //  0 : "NOT_ACTIVE"
  //  1 : "ACTIVE"
  // 
  uint8_t ANTILOCK_BRAKE_ACTIVE;               //      Bits= 1

  //  0 : "NOT_ACTIVE"
  //  1 : "ACTIVE"
  // 
  uint8_t TRACTION_CONTROL_ACTIVE;             //      Bits= 1

  //  0 : "NOT_ACTIVE"
  //  1 : "ACTIVE"
  // 
  uint8_t FOUR_WHEEL_DRIVE_ACTIVE;             //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t ANTILOCK_BRAKE_ACTIVE_AVAIL;         //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t TRACTION_CONTROL_ACTIVE_AVAIL;       //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t FOUR_WHEEL_DRIVE_ACTIVE_AVAIL;       //      Bits= 1

  //  0 : "NORMAL"
  //  1 : "ECO"
  //  2 : "SPORT"
  // 
  uint8_t DRIVE_MODE;                          //      Bits= 4

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t DRIVE_MODE_AVAIL;                    //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} DRIVE_TRAIN_FEATURE_RPT_t;

// def @SAFETY_FUNC_CRITICAL_STOP_RPT CAN Message (1056 0x420)
#define SAFETY_FUNC_CRITICAL_STOP_RPT_IDE (0U)
#define SAFETY_FUNC_CRITICAL_STOP_RPT_DLC (3U)
#define SAFETY_FUNC_CRITICAL_STOP_RPT_CANID (0x420)
#define SAFETY_FUNC_CRITICAL_STOP_RPT_CYC (1000U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  // This value shall latch while the safety function is in the critical stop state.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t AUTOMSMAN_OPCTRL_FAULT : 1;            //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t REMOTE_STOP_FAULT : 1;                 //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SAFETY_BRAKE_OPCTRL_OFF : 1;           //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SAFETY_BRAKE_CMD_TIMEOUT : 1;          //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SAFETY_FUNC_CMD_TIMEOUT : 1;           //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SAFETY_FUNC_CRITICAL_STOP_1_CMD : 1;   //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SAFETY_FUNC_CRITICAL_STOP_2_CMD : 1;   //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SAFETY_FUNC_NONE_CMD : 1;              //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD_SYSTEM_TIMEOUT : 1;             //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD_SYSTEM_FAULT : 1;               //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD_SYSTEM_NOT_ACTIVE : 1;          //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t VEHICLE_REPORT_TIMEOUT : 1;            //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t VEHICLE_REPORT_FAULT : 1;              //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t LOW_ENGINE_RPM : 1;                    //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state. It shall be TRUE when the primary microprocessor detects a fault on signal 1.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PRI_SAFETY_BRAKE_SIGNAL_1_FAULT : 1;   //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state. It shall be TRUE when the primary microprocessor detects a fault on signal 2.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PRI_SAFETY_BRAKE_SIGNAL_2_FAULT : 1;   //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state. It shall be TRUE when the secondary microprocessor detects a fault on signal 1.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SEC_SAFETY_BRAKE_SIGNAL_1_FAULT : 1;   //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state. It shall be TRUE when the secondary microprocessor detects a fault on signal 2.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SEC_SAFETY_BRAKE_SIGNAL_2_FAULT : 1;   //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PRIMARY_PROCESSOR_FAULT : 1;           //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SECONDARY_PROCESSOR_FAULT : 1;         //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t REMOTE_STOP_CMD : 1;                   //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state. It shall be TRUE when the primary microprocessor detects disagreement between safety brake command and safety brake response.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PRI_SAFETY_BRAKE_PRESSURE_FAULT : 1;   //      Bits= 1

#else

  // This value shall latch while the safety function is in the critical stop state.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t AUTOMSMAN_OPCTRL_FAULT;                //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t REMOTE_STOP_FAULT;                     //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SAFETY_BRAKE_OPCTRL_OFF;               //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SAFETY_BRAKE_CMD_TIMEOUT;              //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SAFETY_FUNC_CMD_TIMEOUT;               //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SAFETY_FUNC_CRITICAL_STOP_1_CMD;       //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SAFETY_FUNC_CRITICAL_STOP_2_CMD;       //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SAFETY_FUNC_NONE_CMD;                  //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD_SYSTEM_TIMEOUT;                 //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD_SYSTEM_FAULT;                   //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD_SYSTEM_NOT_ACTIVE;              //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t VEHICLE_REPORT_TIMEOUT;                //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t VEHICLE_REPORT_FAULT;                  //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t LOW_ENGINE_RPM;                        //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state. It shall be TRUE when the primary microprocessor detects a fault on signal 1.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PRI_SAFETY_BRAKE_SIGNAL_1_FAULT;       //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state. It shall be TRUE when the primary microprocessor detects a fault on signal 2.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PRI_SAFETY_BRAKE_SIGNAL_2_FAULT;       //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state. It shall be TRUE when the secondary microprocessor detects a fault on signal 1.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SEC_SAFETY_BRAKE_SIGNAL_1_FAULT;       //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state. It shall be TRUE when the secondary microprocessor detects a fault on signal 2.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SEC_SAFETY_BRAKE_SIGNAL_2_FAULT;       //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PRIMARY_PROCESSOR_FAULT;               //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SECONDARY_PROCESSOR_FAULT;             //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t REMOTE_STOP_CMD;                       //      Bits= 1

  // This value shall latch while the safety function is in the critical stop state. It shall be TRUE when the primary microprocessor detects disagreement between safety brake command and safety brake response.
  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PRI_SAFETY_BRAKE_PRESSURE_FAULT;       //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} SAFETY_FUNC_CRITICAL_STOP_RPT_t;

// def @WATCHDOG_RPT_2 CAN Message (1057 0x421)
#define WATCHDOG_RPT_2_IDE (0U)
#define WATCHDOG_RPT_2_DLC (6U)
#define WATCHDOG_RPT_2_CANID (0x421)
#define WATCHDOG_RPT_2_CYC (1000U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ACCEL_RPT_TIMEOUT : 1;                //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t BRAKE_RPT_TIMEOUT : 1;                //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t BRAKE_DECEL_RPT_TIMEOUT : 1;          //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CABIN_CLIMATE_RPT_TIMEOUT : 1;        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CABIN_FAN_SPEED_RPT_TIMEOUT : 1;      //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CABIN_TEMP_RPT_TIMEOUT : 1;           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CRUISE_CONTROL_RPT_TIMEOUT : 1;       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t DASH_LEFT_RPT_TIMEOUT : 1;            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t DASH_RIGHT_RPT_TIMEOUT : 1;           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENGINE_BRAKE_RPT_TIMEOUT : 1;         //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t HAZARD_LIGHTS_RPT_TIMEOUT : 1;        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t HEADLIGHT_RPT_TIMEOUT : 1;            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t HORN_RPT_TIMEOUT : 1;                 //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t MARKER_LAMP_RPT_TIMEOUT : 1;          //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t MEDIA_CONTROLS_RPT_TIMEOUT : 1;       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PARKING_BRAKE_RPT_TIMEOUT : 1;        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t REAR_PASS_DOOR_RPT_TIMEOUT : 1;       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SHIFT_RPT_TIMEOUT : 1;                //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SPRAYER_RPT_TIMEOUT : 1;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t STEERING_RPT_TIMEOUT : 1;             //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t TURN_RPT_TIMEOUT : 1;                 //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t WIPER_RPT_TIMEOUT : 1;                //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD1_SANITY_FAULT : 1;             //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD2_SANITY_FAULT : 1;             //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD3_SANITY_FAULT : 1;             //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI1_SANITY_FAULT : 1;            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI2_SANITY_FAULT : 1;            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI3_SANITY_FAULT : 1;            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD1_COMPONENT_RPT_TIMEOUT : 1;    //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD2_COMPONENT_RPT_TIMEOUT : 1;    //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD3_COMPONENT_RPT_TIMEOUT : 1;    //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI1_COMPONENT_RPT_TIMEOUT : 1;   //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI2_COMPONENT_RPT_TIMEOUT : 1;   //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI3_COMPONENT_RPT_TIMEOUT : 1;   //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD1_SYSTEM_PRESENT_FAULT : 1;     //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD2_SYSTEM_PRESENT_FAULT : 1;     //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD3_SYSTEM_PRESENT_FAULT : 1;     //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI1_SYSTEM_PRESENT_FAULT : 1;    //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI2_SYSTEM_PRESENT_FAULT : 1;    //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI3_SYSTEM_PRESENT_FAULT : 1;    //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t DRIVE_MODE_INVALID : 1;               //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t GLOBAL_CMD_SANITY_FAULT : 1;          //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t GLOBAL_CMD_TIMEOUT : 1;               //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t EXHAUST_BRAKE_RPT_TIMEOUT : 1;        //      Bits= 1

#else

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ACCEL_RPT_TIMEOUT;                    //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t BRAKE_RPT_TIMEOUT;                    //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t BRAKE_DECEL_RPT_TIMEOUT;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CABIN_CLIMATE_RPT_TIMEOUT;            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CABIN_FAN_SPEED_RPT_TIMEOUT;          //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CABIN_TEMP_RPT_TIMEOUT;               //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t CRUISE_CONTROL_RPT_TIMEOUT;           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t DASH_LEFT_RPT_TIMEOUT;                //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t DASH_RIGHT_RPT_TIMEOUT;               //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ENGINE_BRAKE_RPT_TIMEOUT;             //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t HAZARD_LIGHTS_RPT_TIMEOUT;            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t HEADLIGHT_RPT_TIMEOUT;                //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t HORN_RPT_TIMEOUT;                     //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t MARKER_LAMP_RPT_TIMEOUT;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t MEDIA_CONTROLS_RPT_TIMEOUT;           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PARKING_BRAKE_RPT_TIMEOUT;            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t REAR_PASS_DOOR_RPT_TIMEOUT;           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SHIFT_RPT_TIMEOUT;                    //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SPRAYER_RPT_TIMEOUT;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t STEERING_RPT_TIMEOUT;                 //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t TURN_RPT_TIMEOUT;                     //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t WIPER_RPT_TIMEOUT;                    //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD1_SANITY_FAULT;                 //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD2_SANITY_FAULT;                 //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD3_SANITY_FAULT;                 //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI1_SANITY_FAULT;                //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI2_SANITY_FAULT;                //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI3_SANITY_FAULT;                //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD1_COMPONENT_RPT_TIMEOUT;        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD2_COMPONENT_RPT_TIMEOUT;        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD3_COMPONENT_RPT_TIMEOUT;        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI1_COMPONENT_RPT_TIMEOUT;       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI2_COMPONENT_RPT_TIMEOUT;       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI3_COMPONENT_RPT_TIMEOUT;       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD1_SYSTEM_PRESENT_FAULT;         //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD2_SYSTEM_PRESENT_FAULT;         //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD3_SYSTEM_PRESENT_FAULT;         //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI1_SYSTEM_PRESENT_FAULT;        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI2_SYSTEM_PRESENT_FAULT;        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI3_SYSTEM_PRESENT_FAULT;        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t DRIVE_MODE_INVALID;                   //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t GLOBAL_CMD_SANITY_FAULT;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t GLOBAL_CMD_TIMEOUT;                   //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t EXHAUST_BRAKE_RPT_TIMEOUT;            //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} WATCHDOG_RPT_2_t;

// def @VIN_RPT_2 CAN Message (1058 0x422)
#define VIN_RPT_2_IDE (0U)
#define VIN_RPT_2_DLC (8U)
#define VIN_RPT_2_CANID (0x422)
#define VIN_RPT_2_CYC (1000U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  uint8_t VEHICLE_IDENTIFIER_SECTION_10;       //      Bits= 8 Unit:'ASCII'

  uint8_t VEHICLE_IDENTIFIER_SECTION_11;       //      Bits= 8 Unit:'ASCII'

  uint8_t VEHICLE_IDENTIFIER_SECTION_12;       //      Bits= 8 Unit:'ASCII'

  uint8_t VEHICLE_IDENTIFIER_SECTION_13;       //      Bits= 8 Unit:'ASCII'

  uint8_t VEHICLE_IDENTIFIER_SECTION_14;       //      Bits= 8 Unit:'ASCII'

  uint8_t VEHICLE_IDENTIFIER_SECTION_15;       //      Bits= 8 Unit:'ASCII'

  uint8_t VEHICLE_IDENTIFIER_SECTION_16;       //      Bits= 8 Unit:'ASCII'

  uint8_t VEHICLE_IDENTIFIER_SECTION_17;       //      Bits= 8 Unit:'ASCII'

#else

  uint8_t VEHICLE_IDENTIFIER_SECTION_10;       //      Bits= 8 Unit:'ASCII'

  uint8_t VEHICLE_IDENTIFIER_SECTION_11;       //      Bits= 8 Unit:'ASCII'

  uint8_t VEHICLE_IDENTIFIER_SECTION_12;       //      Bits= 8 Unit:'ASCII'

  uint8_t VEHICLE_IDENTIFIER_SECTION_13;       //      Bits= 8 Unit:'ASCII'

  uint8_t VEHICLE_IDENTIFIER_SECTION_14;       //      Bits= 8 Unit:'ASCII'

  uint8_t VEHICLE_IDENTIFIER_SECTION_15;       //      Bits= 8 Unit:'ASCII'

  uint8_t VEHICLE_IDENTIFIER_SECTION_16;       //      Bits= 8 Unit:'ASCII'

  uint8_t VEHICLE_IDENTIFIER_SECTION_17;       //      Bits= 8 Unit:'ASCII'

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} VIN_RPT_2_t;

// def @NOTIFICATION_RPT CAN Message (1059 0x423)
#define NOTIFICATION_RPT_IDE (0U)
#define NOTIFICATION_RPT_DLC (1U)
#define NOTIFICATION_RPT_CANID (0x423)
#define NOTIFICATION_RPT_CYC (250U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "OFF"
  //  1 : "ON"
  //  2 : "MUTED"
  // 
  uint8_t BUZZER_STATUS : 2;                 //      Bits= 2

  //  0 : "NO_BLINK"
  //  1 : "BLINK"
  // 
  uint8_t LIGHT_STATUS : 2;                  //      Bits= 2

  //  1 : "WHITE"
  //  2 : "RED"
  //  3 : "GREEN"
  //  4 : "BLUE"
  //  5 : "PURPLE"
  //  6 : "ORANGE"
  //  7 : "YELLOW"
  //  8 : "TEAL"
  //  9 : "PINK"
  //  10 : "OFF"
  // 
  uint8_t LIGHT_COLOR : 4;                   //      Bits= 4

#else

  //  0 : "OFF"
  //  1 : "ON"
  //  2 : "MUTED"
  // 
  uint8_t BUZZER_STATUS;                     //      Bits= 2

  //  0 : "NO_BLINK"
  //  1 : "BLINK"
  // 
  uint8_t LIGHT_STATUS;                      //      Bits= 2

  //  1 : "WHITE"
  //  2 : "RED"
  //  3 : "GREEN"
  //  4 : "BLUE"
  //  5 : "PURPLE"
  //  6 : "ORANGE"
  //  7 : "YELLOW"
  //  8 : "TEAL"
  //  9 : "PINK"
  //  10 : "OFF"
  // 
  uint8_t LIGHT_COLOR;                       //      Bits= 4

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} NOTIFICATION_RPT_t;

// def @AIR_PRESSURE_RPT CAN Message (1060 0x424)
#define AIR_PRESSURE_RPT_IDE (0U)
#define AIR_PRESSURE_RPT_DLC (7U)
#define AIR_PRESSURE_RPT_CANID (0x424)
#define AIR_PRESSURE_RPT_CYC (1000U)
// signal: @PNEUMATIC_SUPPLY_PRESSURE_ro
#define PACMOD12_PNEUMATIC_SUPPLY_PRESSURE_ro_CovFactor (8)
#define PACMOD12_PNEUMATIC_SUPPLY_PRESSURE_ro_toS(x) ( (uint16_t) ((x) / (8)) )
#define PACMOD12_PNEUMATIC_SUPPLY_PRESSURE_ro_fromS(x) ( ((x) * (8)) )
// signal: @BRAKE_CIRCUIT_1_PRESSURE_ro
#define PACMOD12_BRAKE_CIRCUIT_1_PRESSURE_ro_CovFactor (8)
#define PACMOD12_BRAKE_CIRCUIT_1_PRESSURE_ro_toS(x) ( (uint16_t) ((x) / (8)) )
#define PACMOD12_BRAKE_CIRCUIT_1_PRESSURE_ro_fromS(x) ( ((x) * (8)) )
// signal: @BRAKE_CIRCUIT_2_PRESSURE_ro
#define PACMOD12_BRAKE_CIRCUIT_2_PRESSURE_ro_CovFactor (8)
#define PACMOD12_BRAKE_CIRCUIT_2_PRESSURE_ro_toS(x) ( (uint16_t) ((x) / (8)) )
#define PACMOD12_BRAKE_CIRCUIT_2_PRESSURE_ro_fromS(x) ( ((x) * (8)) )
// signal: @PARK_TRAILER_AIR_PRESSURE_ro
#define PACMOD12_PARK_TRAILER_AIR_PRESSURE_ro_CovFactor (8)
#define PACMOD12_PARK_TRAILER_AIR_PRESSURE_ro_toS(x) ( (uint16_t) ((x) / (8)) )
#define PACMOD12_PARK_TRAILER_AIR_PRESSURE_ro_fromS(x) ( ((x) * (8)) )
// signal: @POWERTRAIN_AIR_PRESSURE_ro
#define PACMOD12_POWERTRAIN_AIR_PRESSURE_ro_CovFactor (8)
#define PACMOD12_POWERTRAIN_AIR_PRESSURE_ro_toS(x) ( (uint16_t) ((x) / (8)) )
#define PACMOD12_POWERTRAIN_AIR_PRESSURE_ro_fromS(x) ( ((x) * (8)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  uint16_t PNEUMATIC_SUPPLY_PRESSURE_ro;         //      Bits= 8 Factor= 8               Unit:'kPa'

#ifdef PACMOD12_USE_SIGFLOAT
  uint16_t PNEUMATIC_SUPPLY_PRESSURE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t BRAKE_CIRCUIT_1_PRESSURE_ro;          //      Bits= 8 Factor= 8               Unit:'kPa'

#ifdef PACMOD12_USE_SIGFLOAT
  uint16_t BRAKE_CIRCUIT_1_PRESSURE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t BRAKE_CIRCUIT_2_PRESSURE_ro;          //      Bits= 8 Factor= 8               Unit:'kPa'

#ifdef PACMOD12_USE_SIGFLOAT
  uint16_t BRAKE_CIRCUIT_2_PRESSURE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t PARK_TRAILER_AIR_PRESSURE_ro;         //      Bits= 8 Factor= 8               Unit:'kPa'

#ifdef PACMOD12_USE_SIGFLOAT
  uint16_t PARK_TRAILER_AIR_PRESSURE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t POWERTRAIN_AIR_PRESSURE_ro;           //      Bits= 8 Factor= 8               Unit:'kPa'

#ifdef PACMOD12_USE_SIGFLOAT
  uint16_t POWERTRAIN_AIR_PRESSURE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  //  0 : "NOT_ACTIVE"
  //  1 : "ACTIVE"
  //  2 : "ERROR"
  // 
  uint8_t AIR_COMPRESSOR_STATUS : 2;             //      Bits= 2

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t PNEUMATIC_SUPPLY_PRESSURE_AVAIL : 1;   //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t BRAKE_CIRCUIT_1_PRESSURE_AVAIL : 1;    //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t BRAKE_CIRCUIT_2_PRESSURE_AVAIL : 1;    //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t PARK_TRAILER_AIR_PRESSURE_AVAIL : 1;   //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t POWERTRAIN_AIR_PRESSURE_AVAIL : 1;     //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t AIR_COMPRESSOR_STATUS_AVAIL : 1;       //      Bits= 1

#else

  uint16_t PNEUMATIC_SUPPLY_PRESSURE_ro;         //      Bits= 8 Factor= 8               Unit:'kPa'

#ifdef PACMOD12_USE_SIGFLOAT
  uint16_t PNEUMATIC_SUPPLY_PRESSURE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t BRAKE_CIRCUIT_1_PRESSURE_ro;          //      Bits= 8 Factor= 8               Unit:'kPa'

#ifdef PACMOD12_USE_SIGFLOAT
  uint16_t BRAKE_CIRCUIT_1_PRESSURE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t BRAKE_CIRCUIT_2_PRESSURE_ro;          //      Bits= 8 Factor= 8               Unit:'kPa'

#ifdef PACMOD12_USE_SIGFLOAT
  uint16_t BRAKE_CIRCUIT_2_PRESSURE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t PARK_TRAILER_AIR_PRESSURE_ro;         //      Bits= 8 Factor= 8               Unit:'kPa'

#ifdef PACMOD12_USE_SIGFLOAT
  uint16_t PARK_TRAILER_AIR_PRESSURE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  uint16_t POWERTRAIN_AIR_PRESSURE_ro;           //      Bits= 8 Factor= 8               Unit:'kPa'

#ifdef PACMOD12_USE_SIGFLOAT
  uint16_t POWERTRAIN_AIR_PRESSURE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  //  0 : "NOT_ACTIVE"
  //  1 : "ACTIVE"
  //  2 : "ERROR"
  // 
  uint8_t AIR_COMPRESSOR_STATUS;                 //      Bits= 2

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t PNEUMATIC_SUPPLY_PRESSURE_AVAIL;       //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t BRAKE_CIRCUIT_1_PRESSURE_AVAIL;        //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t BRAKE_CIRCUIT_2_PRESSURE_AVAIL;        //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t PARK_TRAILER_AIR_PRESSURE_AVAIL;       //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t POWERTRAIN_AIR_PRESSURE_AVAIL;         //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t AIR_COMPRESSOR_STATUS_AVAIL;           //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} AIR_PRESSURE_RPT_t;

// def @ENGINE_RPT_2 CAN Message (1061 0x425)
#define ENGINE_RPT_2_IDE (0U)
#define ENGINE_RPT_2_DLC (3U)
#define ENGINE_RPT_2_CANID (0x425)
// signal: @FUEL_RATE_ro
#define PACMOD12_FUEL_RATE_ro_CovFactor (0.050000)
#define PACMOD12_FUEL_RATE_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.050000)) )
#define PACMOD12_FUEL_RATE_ro_fromS(x) ( (((x) * (0.050000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  uint16_t FUEL_RATE_ro;                     //      Bits=16 Factor= 0.050000        Unit:'L/h'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t FUEL_RATE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t FUEL_RATE_AVAIL : 1;               //      Bits= 1

#else

  uint16_t FUEL_RATE_ro;                     //      Bits=16 Factor= 0.050000        Unit:'L/h'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t FUEL_RATE_phys;
#endif // PACMOD12_USE_SIGFLOAT

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  //  0 : "NOT_AVAILABLE"
  //  1 : "AVAILABLE"
  // 
  uint8_t FUEL_RATE_AVAIL;                   //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} ENGINE_RPT_2_t;

// def @WHEEL_SPEED_RPT_2 CAN Message (1062 0x426)
#define WHEEL_SPEED_RPT_2_IDE (0U)
#define WHEEL_SPEED_RPT_2_DLC (8U)
#define WHEEL_SPEED_RPT_2_CANID (0x426)
// signal: @WHEEL_SPD_AXLE_3_LEFT_ro
#define PACMOD12_WHEEL_SPD_AXLE_3_LEFT_ro_CovFactor (0.010000)
#define PACMOD12_WHEEL_SPD_AXLE_3_LEFT_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.010000)) )
#define PACMOD12_WHEEL_SPD_AXLE_3_LEFT_ro_fromS(x) ( (((x) * (0.010000)) + (0.000000)) )
// signal: @WHEEL_SPD_AXLE_3_RIGHT_ro
#define PACMOD12_WHEEL_SPD_AXLE_3_RIGHT_ro_CovFactor (0.010000)
#define PACMOD12_WHEEL_SPD_AXLE_3_RIGHT_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.010000)) )
#define PACMOD12_WHEEL_SPD_AXLE_3_RIGHT_ro_fromS(x) ( (((x) * (0.010000)) + (0.000000)) )
// signal: @WHEEL_SPD_AXLE_4_LEFT_ro
#define PACMOD12_WHEEL_SPD_AXLE_4_LEFT_ro_CovFactor (0.010000)
#define PACMOD12_WHEEL_SPD_AXLE_4_LEFT_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.010000)) )
#define PACMOD12_WHEEL_SPD_AXLE_4_LEFT_ro_fromS(x) ( (((x) * (0.010000)) + (0.000000)) )
// signal: @WHEEL_SPD_AXLE_4_RIGHT_ro
#define PACMOD12_WHEEL_SPD_AXLE_4_RIGHT_ro_CovFactor (0.010000)
#define PACMOD12_WHEEL_SPD_AXLE_4_RIGHT_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.010000)) )
#define PACMOD12_WHEEL_SPD_AXLE_4_RIGHT_ro_fromS(x) ( (((x) * (0.010000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  int16_t WHEEL_SPD_AXLE_3_LEFT_ro;          //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t WHEEL_SPD_AXLE_3_LEFT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int16_t WHEEL_SPD_AXLE_3_RIGHT_ro;         //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t WHEEL_SPD_AXLE_3_RIGHT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int16_t WHEEL_SPD_AXLE_4_LEFT_ro;          //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t WHEEL_SPD_AXLE_4_LEFT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int16_t WHEEL_SPD_AXLE_4_RIGHT_ro;         //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t WHEEL_SPD_AXLE_4_RIGHT_phys;
#endif // PACMOD12_USE_SIGFLOAT

#else

  int16_t WHEEL_SPD_AXLE_3_LEFT_ro;          //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t WHEEL_SPD_AXLE_3_LEFT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int16_t WHEEL_SPD_AXLE_3_RIGHT_ro;         //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t WHEEL_SPD_AXLE_3_RIGHT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int16_t WHEEL_SPD_AXLE_4_LEFT_ro;          //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t WHEEL_SPD_AXLE_4_LEFT_phys;
#endif // PACMOD12_USE_SIGFLOAT

  int16_t WHEEL_SPD_AXLE_4_RIGHT_ro;         //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD12_USE_SIGFLOAT
  sigfloat_t WHEEL_SPD_AXLE_4_RIGHT_phys;
#endif // PACMOD12_USE_SIGFLOAT

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} WHEEL_SPEED_RPT_2_t;

// Not for new development.
// def @WATCHDOG_RPT CAN Message (1536 0x600)
#define WATCHDOG_RPT_IDE (0U)
#define WATCHDOG_RPT_DLC (8U)
#define WATCHDOG_RPT_CANID (0x600)
#define WATCHDOG_RPT_CYC (100U)

typedef struct
{
#ifdef PACMOD12_USE_BITS_SIGNAL

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t GLOBAL_ENABLE_FLAG : 1;                //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t GLOBAL_OVERRIDE_ACTIVE : 1;            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t GLOBAL_COMMAND_TIMEOUT_ERROR : 1;      //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t GLOBAL_PACMOD_SUBSYSTEM_TIMEOUT : 1;   //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t GLOBAL_VEHICLE_CAN_TIMEOUT : 1;        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t GLOBAL_PACMOD_SYS_FAULT_ACTIVE : 1;    //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t GLOBAL_CONFIG_FAULT_ACTIVE : 1;        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t GLOBAL_TIMEOUT : 1;                    //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ACCEL_ENABLED : 1;                     //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ACCEL_OVERRIDE_ACTIVE : 1;             //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ACCEL_COMMAND_OUTPUT_FAULT : 1;        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ACCEL_INPUT_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ACCEL_OUTPUT_REPORTED_FAULT : 1;       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ACCEL_PACMOD_FAULT : 1;                //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ACCEL_VEHICLE_FAULT : 1;               //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ACCEL_TIMEOUT : 1;                     //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t BRAKE_ENABLED : 1;                     //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t BRAKE_OVERRIDE_ACTIVE : 1;             //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t BRAKE_COMMAND_OUTPUT_FAULT : 1;        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t BRAKE_INPUT_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t BRAKE_OUTPUT_REPORTED_FAULT : 1;       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t BRAKE_PACMOD_FAULT : 1;                //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t BRAKE_VEHICLE_FAULT : 1;               //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t BRAKE_TIMEOUT : 1;                     //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SHIFT_ENABLED : 1;                     //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SHIFT_OVERRIDE_ACTIVE : 1;             //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SHIFT_COMMAND_OUTPUT_FAULT : 1;        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SHIFT_INPUT_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SHIFT_OUTPUT_REPORTED_FAULT : 1;       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SHIFT_PACMOD_FAULT : 1;                //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SHIFT_VEHICLE_FAULT : 1;               //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SHIFT_TIMEOUT : 1;                     //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t STEER_ENABLED : 1;                     //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t STEER_OVERRIDE_ACTIVE : 1;             //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t STEER_COMMAND_OUTPUT_FAULT : 1;        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t STEER_INPUT_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t STEER_OUTPUT_REPORTED_FAULT : 1;       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t STEER_PACMOD_FAULT : 1;                //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t STEER_VEHICLE_FAULT : 1;               //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t STEER_TIMEOUT : 1;                     //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD1_CONFIG_FAULT : 1;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD1_CAN_TIMEOUT : 1;               //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD1_COUNTER_FAULT : 1;             //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD2_CONFIG_FAULT : 1;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD2_CAN_TIMEOUT : 1;               //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD2_COUNTER_FAULT : 1;             //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD3_CONFIG_FAULT : 1;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD3_CAN_TIMEOUT : 1;               //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD3_COUNTER_FAULT : 1;             //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI1_RPT_TIMEOUT : 1;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI1_CONFIG_FAULT : 1;             //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI1_CAN_TIMEOUT : 1;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI1_COUNTER_FAULT : 1;            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI2_RPT_TIMEOUT : 1;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI2_CONFIG_FAULT : 1;             //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI2_CAN_TIMEOUT : 1;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI2_COUNTER_FAULT : 1;            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI3_RPT_TIMEOUT : 1;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI3_CONFIG_FAULT : 1;             //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI3_CAN_TIMEOUT : 1;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI3_COUNTER_FAULT : 1;            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD_SYSTEM_PRESENT_FAULT : 1;       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI_SYSTEM_PRESENT_FAULT : 1;      //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t GLOBAL_INT_POWER_SUPPLY_FAULT : 1;     //      Bits= 1

#else

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t GLOBAL_ENABLE_FLAG;                    //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t GLOBAL_OVERRIDE_ACTIVE;                //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t GLOBAL_COMMAND_TIMEOUT_ERROR;          //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t GLOBAL_PACMOD_SUBSYSTEM_TIMEOUT;       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t GLOBAL_VEHICLE_CAN_TIMEOUT;            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t GLOBAL_PACMOD_SYS_FAULT_ACTIVE;        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t GLOBAL_CONFIG_FAULT_ACTIVE;            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t GLOBAL_TIMEOUT;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ACCEL_ENABLED;                         //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ACCEL_OVERRIDE_ACTIVE;                 //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ACCEL_COMMAND_OUTPUT_FAULT;            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ACCEL_INPUT_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ACCEL_OUTPUT_REPORTED_FAULT;           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ACCEL_PACMOD_FAULT;                    //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ACCEL_VEHICLE_FAULT;                   //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t ACCEL_TIMEOUT;                         //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t BRAKE_ENABLED;                         //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t BRAKE_OVERRIDE_ACTIVE;                 //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t BRAKE_COMMAND_OUTPUT_FAULT;            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t BRAKE_INPUT_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t BRAKE_OUTPUT_REPORTED_FAULT;           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t BRAKE_PACMOD_FAULT;                    //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t BRAKE_VEHICLE_FAULT;                   //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t BRAKE_TIMEOUT;                         //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SHIFT_ENABLED;                         //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SHIFT_OVERRIDE_ACTIVE;                 //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SHIFT_COMMAND_OUTPUT_FAULT;            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SHIFT_INPUT_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SHIFT_OUTPUT_REPORTED_FAULT;           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SHIFT_PACMOD_FAULT;                    //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SHIFT_VEHICLE_FAULT;                   //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t SHIFT_TIMEOUT;                         //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t STEER_ENABLED;                         //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t STEER_OVERRIDE_ACTIVE;                 //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t STEER_COMMAND_OUTPUT_FAULT;            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t STEER_INPUT_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t STEER_OUTPUT_REPORTED_FAULT;           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t STEER_PACMOD_FAULT;                    //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t STEER_VEHICLE_FAULT;                   //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t STEER_TIMEOUT;                         //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD1_CONFIG_FAULT;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD1_CAN_TIMEOUT;                   //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD1_COUNTER_FAULT;                 //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD2_CONFIG_FAULT;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD2_CAN_TIMEOUT;                   //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD2_COUNTER_FAULT;                 //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD3_CONFIG_FAULT;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD3_CAN_TIMEOUT;                   //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD3_COUNTER_FAULT;                 //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI1_RPT_TIMEOUT;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI1_CONFIG_FAULT;                 //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI1_CAN_TIMEOUT;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI1_COUNTER_FAULT;                //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI2_RPT_TIMEOUT;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI2_CONFIG_FAULT;                 //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI2_CAN_TIMEOUT;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI2_COUNTER_FAULT;                //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI3_RPT_TIMEOUT;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI3_CONFIG_FAULT;                 //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI3_CAN_TIMEOUT;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI3_COUNTER_FAULT;                //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMOD_SYSTEM_PRESENT_FAULT;           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t PACMINI_SYSTEM_PRESENT_FAULT;          //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  // 
  uint8_t GLOBAL_INT_POWER_SUPPLY_FAULT;         //      Bits= 1

#endif // PACMOD12_USE_BITS_SIGNAL

#ifdef PACMOD12_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD12_USE_DIAG_MONITORS

} WATCHDOG_RPT_t;

// Function signatures

uint32_t Unpack_GLOBAL_RPT_pacmod12(GLOBAL_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_GLOBAL_RPT_pacmod12(GLOBAL_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_GLOBAL_RPT_pacmod12(GLOBAL_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_GLOBAL_RPT_2_pacmod12(GLOBAL_RPT_2_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_GLOBAL_RPT_2_pacmod12(GLOBAL_RPT_2_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_GLOBAL_RPT_2_pacmod12(GLOBAL_RPT_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_COMPONENT_RPT_00_pacmod12(COMPONENT_RPT_00_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_COMPONENT_RPT_00_pacmod12(COMPONENT_RPT_00_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_COMPONENT_RPT_00_pacmod12(COMPONENT_RPT_00_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_COMPONENT_RPT_01_pacmod12(COMPONENT_RPT_01_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_COMPONENT_RPT_01_pacmod12(COMPONENT_RPT_01_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_COMPONENT_RPT_01_pacmod12(COMPONENT_RPT_01_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_COMPONENT_RPT_02_pacmod12(COMPONENT_RPT_02_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_COMPONENT_RPT_02_pacmod12(COMPONENT_RPT_02_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_COMPONENT_RPT_02_pacmod12(COMPONENT_RPT_02_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_COMPONENT_RPT_03_pacmod12(COMPONENT_RPT_03_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_COMPONENT_RPT_03_pacmod12(COMPONENT_RPT_03_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_COMPONENT_RPT_03_pacmod12(COMPONENT_RPT_03_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_COMPONENT_RPT_04_pacmod12(COMPONENT_RPT_04_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_COMPONENT_RPT_04_pacmod12(COMPONENT_RPT_04_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_COMPONENT_RPT_04_pacmod12(COMPONENT_RPT_04_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_SAFETY_FUNC_RPT_pacmod12(SAFETY_FUNC_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_SAFETY_FUNC_RPT_pacmod12(SAFETY_FUNC_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_SAFETY_FUNC_RPT_pacmod12(SAFETY_FUNC_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_SAFETY_BRAKE_RPT_pacmod12(SAFETY_BRAKE_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_SAFETY_BRAKE_RPT_pacmod12(SAFETY_BRAKE_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_SAFETY_BRAKE_RPT_pacmod12(SAFETY_BRAKE_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_VEHICLE_FAULT_RPT_pacmod12(VEHICLE_FAULT_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_VEHICLE_FAULT_RPT_pacmod12(VEHICLE_FAULT_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_VEHICLE_FAULT_RPT_pacmod12(VEHICLE_FAULT_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_GLOBAL_CMD_pacmod12(GLOBAL_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_GLOBAL_CMD_pacmod12(GLOBAL_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_GLOBAL_CMD_pacmod12(GLOBAL_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_SUPERVISORY_CTRL_pacmod12(SUPERVISORY_CTRL_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_SUPERVISORY_CTRL_pacmod12(SUPERVISORY_CTRL_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_SUPERVISORY_CTRL_pacmod12(SUPERVISORY_CTRL_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_SAFETY_FUNC_CMD_pacmod12(SAFETY_FUNC_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_SAFETY_FUNC_CMD_pacmod12(SAFETY_FUNC_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_SAFETY_FUNC_CMD_pacmod12(SAFETY_FUNC_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_SAFETY_BRAKE_CMD_pacmod12(SAFETY_BRAKE_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_SAFETY_BRAKE_CMD_pacmod12(SAFETY_BRAKE_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_SAFETY_BRAKE_CMD_pacmod12(SAFETY_BRAKE_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_ACCEL_CMD_pacmod12(ACCEL_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_ACCEL_CMD_pacmod12(ACCEL_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_ACCEL_CMD_pacmod12(ACCEL_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_BRAKE_CMD_pacmod12(BRAKE_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_BRAKE_CMD_pacmod12(BRAKE_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BRAKE_CMD_pacmod12(BRAKE_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_CRUISE_CONTROL_BUTTONS_CMD_pacmod12(CRUISE_CONTROL_BUTTONS_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_CRUISE_CONTROL_BUTTONS_CMD_pacmod12(CRUISE_CONTROL_BUTTONS_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CRUISE_CONTROL_BUTTONS_CMD_pacmod12(CRUISE_CONTROL_BUTTONS_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_DASH_CONTROLS_LEFT_CMD_pacmod12(DASH_CONTROLS_LEFT_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_DASH_CONTROLS_LEFT_CMD_pacmod12(DASH_CONTROLS_LEFT_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DASH_CONTROLS_LEFT_CMD_pacmod12(DASH_CONTROLS_LEFT_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_DASH_CONTROLS_RIGHT_CMD_pacmod12(DASH_CONTROLS_RIGHT_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_DASH_CONTROLS_RIGHT_CMD_pacmod12(DASH_CONTROLS_RIGHT_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DASH_CONTROLS_RIGHT_CMD_pacmod12(DASH_CONTROLS_RIGHT_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_HAZARD_LIGHTS_CMD_pacmod12(HAZARD_LIGHTS_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_HAZARD_LIGHTS_CMD_pacmod12(HAZARD_LIGHTS_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_HAZARD_LIGHTS_CMD_pacmod12(HAZARD_LIGHTS_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_HEADLIGHT_CMD_pacmod12(HEADLIGHT_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_HEADLIGHT_CMD_pacmod12(HEADLIGHT_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_HEADLIGHT_CMD_pacmod12(HEADLIGHT_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_HORN_CMD_pacmod12(HORN_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_HORN_CMD_pacmod12(HORN_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_HORN_CMD_pacmod12(HORN_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_MEDIA_CONTROLS_CMD_pacmod12(MEDIA_CONTROLS_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_MEDIA_CONTROLS_CMD_pacmod12(MEDIA_CONTROLS_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_MEDIA_CONTROLS_CMD_pacmod12(MEDIA_CONTROLS_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_PARKING_BRAKE_CMD_pacmod12(PARKING_BRAKE_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_PARKING_BRAKE_CMD_pacmod12(PARKING_BRAKE_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_PARKING_BRAKE_CMD_pacmod12(PARKING_BRAKE_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_SHIFT_CMD_pacmod12(SHIFT_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_SHIFT_CMD_pacmod12(SHIFT_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_SHIFT_CMD_pacmod12(SHIFT_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_STEERING_CMD_pacmod12(STEERING_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_STEERING_CMD_pacmod12(STEERING_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_STEERING_CMD_pacmod12(STEERING_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_TURN_CMD_pacmod12(TURN_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_TURN_CMD_pacmod12(TURN_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_TURN_CMD_pacmod12(TURN_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_WIPER_CMD_pacmod12(WIPER_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_WIPER_CMD_pacmod12(WIPER_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_WIPER_CMD_pacmod12(WIPER_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_SPRAYER_CMD_pacmod12(SPRAYER_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_SPRAYER_CMD_pacmod12(SPRAYER_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_SPRAYER_CMD_pacmod12(SPRAYER_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_BRAKE_DECEL_CMD_pacmod12(BRAKE_DECEL_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_BRAKE_DECEL_CMD_pacmod12(BRAKE_DECEL_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BRAKE_DECEL_CMD_pacmod12(BRAKE_DECEL_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_REAR_PASS_DOOR_CMD_pacmod12(REAR_PASS_DOOR_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_REAR_PASS_DOOR_CMD_pacmod12(REAR_PASS_DOOR_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_REAR_PASS_DOOR_CMD_pacmod12(REAR_PASS_DOOR_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_ENGINE_BRAKE_CMD_pacmod12(ENGINE_BRAKE_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_ENGINE_BRAKE_CMD_pacmod12(ENGINE_BRAKE_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_ENGINE_BRAKE_CMD_pacmod12(ENGINE_BRAKE_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_EXHAUST_BRAKE_CMD_pacmod12(EXHAUST_BRAKE_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_EXHAUST_BRAKE_CMD_pacmod12(EXHAUST_BRAKE_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_EXHAUST_BRAKE_CMD_pacmod12(EXHAUST_BRAKE_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_MARKER_LAMP_CMD_pacmod12(MARKER_LAMP_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_MARKER_LAMP_CMD_pacmod12(MARKER_LAMP_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_MARKER_LAMP_CMD_pacmod12(MARKER_LAMP_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_CABIN_TEMP_CMD_pacmod12(CABIN_TEMP_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_CABIN_TEMP_CMD_pacmod12(CABIN_TEMP_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CABIN_TEMP_CMD_pacmod12(CABIN_TEMP_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_CABIN_FAN_SPEED_CMD_pacmod12(CABIN_FAN_SPEED_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_CABIN_FAN_SPEED_CMD_pacmod12(CABIN_FAN_SPEED_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CABIN_FAN_SPEED_CMD_pacmod12(CABIN_FAN_SPEED_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_CABIN_CLIMATE_CMD_pacmod12(CABIN_CLIMATE_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_CABIN_CLIMATE_CMD_pacmod12(CABIN_CLIMATE_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CABIN_CLIMATE_CMD_pacmod12(CABIN_CLIMATE_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_ACCEL_RPT_pacmod12(ACCEL_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_ACCEL_RPT_pacmod12(ACCEL_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_ACCEL_RPT_pacmod12(ACCEL_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_ACCEL_CMD_LIMIT_RPT_pacmod12(ACCEL_CMD_LIMIT_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_ACCEL_CMD_LIMIT_RPT_pacmod12(ACCEL_CMD_LIMIT_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_ACCEL_CMD_LIMIT_RPT_pacmod12(ACCEL_CMD_LIMIT_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_BRAKE_RPT_pacmod12(BRAKE_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_BRAKE_RPT_pacmod12(BRAKE_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BRAKE_RPT_pacmod12(BRAKE_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_BRAKE_CMD_LIMIT_RPT_pacmod12(BRAKE_CMD_LIMIT_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_BRAKE_CMD_LIMIT_RPT_pacmod12(BRAKE_CMD_LIMIT_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BRAKE_CMD_LIMIT_RPT_pacmod12(BRAKE_CMD_LIMIT_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_CRUISE_CONTROL_BUTTONS_RPT_pacmod12(CRUISE_CONTROL_BUTTONS_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_CRUISE_CONTROL_BUTTONS_RPT_pacmod12(CRUISE_CONTROL_BUTTONS_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CRUISE_CONTROL_BUTTONS_RPT_pacmod12(CRUISE_CONTROL_BUTTONS_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_DASH_CONTROLS_LEFT_RPT_pacmod12(DASH_CONTROLS_LEFT_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_DASH_CONTROLS_LEFT_RPT_pacmod12(DASH_CONTROLS_LEFT_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DASH_CONTROLS_LEFT_RPT_pacmod12(DASH_CONTROLS_LEFT_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_DASH_CONTROLS_RIGHT_RPT_pacmod12(DASH_CONTROLS_RIGHT_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_DASH_CONTROLS_RIGHT_RPT_pacmod12(DASH_CONTROLS_RIGHT_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DASH_CONTROLS_RIGHT_RPT_pacmod12(DASH_CONTROLS_RIGHT_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_HAZARD_LIGHTS_RPT_pacmod12(HAZARD_LIGHTS_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_HAZARD_LIGHTS_RPT_pacmod12(HAZARD_LIGHTS_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_HAZARD_LIGHTS_RPT_pacmod12(HAZARD_LIGHTS_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_HEADLIGHT_RPT_pacmod12(HEADLIGHT_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_HEADLIGHT_RPT_pacmod12(HEADLIGHT_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_HEADLIGHT_RPT_pacmod12(HEADLIGHT_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_HORN_RPT_pacmod12(HORN_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_HORN_RPT_pacmod12(HORN_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_HORN_RPT_pacmod12(HORN_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_MEDIA_CONTROLS_RPT_pacmod12(MEDIA_CONTROLS_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_MEDIA_CONTROLS_RPT_pacmod12(MEDIA_CONTROLS_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_MEDIA_CONTROLS_RPT_pacmod12(MEDIA_CONTROLS_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_PARKING_BRAKE_RPT_pacmod12(PARKING_BRAKE_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_PARKING_BRAKE_RPT_pacmod12(PARKING_BRAKE_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_PARKING_BRAKE_RPT_pacmod12(PARKING_BRAKE_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_SHIFT_RPT_pacmod12(SHIFT_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_SHIFT_RPT_pacmod12(SHIFT_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_SHIFT_RPT_pacmod12(SHIFT_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_STEERING_RPT_pacmod12(STEERING_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_STEERING_RPT_pacmod12(STEERING_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_STEERING_RPT_pacmod12(STEERING_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_STEERING_CMD_LIMIT_RPT_pacmod12(STEERING_CMD_LIMIT_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_STEERING_CMD_LIMIT_RPT_pacmod12(STEERING_CMD_LIMIT_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_STEERING_CMD_LIMIT_RPT_pacmod12(STEERING_CMD_LIMIT_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_TURN_RPT_pacmod12(TURN_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_TURN_RPT_pacmod12(TURN_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_TURN_RPT_pacmod12(TURN_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_WIPER_RPT_pacmod12(WIPER_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_WIPER_RPT_pacmod12(WIPER_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_WIPER_RPT_pacmod12(WIPER_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_SPRAYER_RPT_pacmod12(SPRAYER_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_SPRAYER_RPT_pacmod12(SPRAYER_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_SPRAYER_RPT_pacmod12(SPRAYER_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_BRAKE_DECEL_RPT_pacmod12(BRAKE_DECEL_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_BRAKE_DECEL_RPT_pacmod12(BRAKE_DECEL_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BRAKE_DECEL_RPT_pacmod12(BRAKE_DECEL_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_REAR_PASS_DOOR_RPT_pacmod12(REAR_PASS_DOOR_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_REAR_PASS_DOOR_RPT_pacmod12(REAR_PASS_DOOR_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_REAR_PASS_DOOR_RPT_pacmod12(REAR_PASS_DOOR_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_ENGINE_BRAKE_RPT_pacmod12(ENGINE_BRAKE_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_ENGINE_BRAKE_RPT_pacmod12(ENGINE_BRAKE_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_ENGINE_BRAKE_RPT_pacmod12(ENGINE_BRAKE_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_EXHAUST_BRAKE_RPT_pacmod12(EXHAUST_BRAKE_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_EXHAUST_BRAKE_RPT_pacmod12(EXHAUST_BRAKE_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_EXHAUST_BRAKE_RPT_pacmod12(EXHAUST_BRAKE_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_MARKER_LAMP_RPT_pacmod12(MARKER_LAMP_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_MARKER_LAMP_RPT_pacmod12(MARKER_LAMP_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_MARKER_LAMP_RPT_pacmod12(MARKER_LAMP_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_CABIN_TEMP_RPT_pacmod12(CABIN_TEMP_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_CABIN_TEMP_RPT_pacmod12(CABIN_TEMP_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CABIN_TEMP_RPT_pacmod12(CABIN_TEMP_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_CABIN_FAN_SPEED_RPT_pacmod12(CABIN_FAN_SPEED_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_CABIN_FAN_SPEED_RPT_pacmod12(CABIN_FAN_SPEED_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CABIN_FAN_SPEED_RPT_pacmod12(CABIN_FAN_SPEED_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_CABIN_CLIMATE_RPT_pacmod12(CABIN_CLIMATE_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_CABIN_CLIMATE_RPT_pacmod12(CABIN_CLIMATE_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CABIN_CLIMATE_RPT_pacmod12(CABIN_CLIMATE_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_ACCEL_AUX_RPT_pacmod12(ACCEL_AUX_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_ACCEL_AUX_RPT_pacmod12(ACCEL_AUX_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_ACCEL_AUX_RPT_pacmod12(ACCEL_AUX_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_BRAKE_AUX_RPT_pacmod12(BRAKE_AUX_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_BRAKE_AUX_RPT_pacmod12(BRAKE_AUX_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BRAKE_AUX_RPT_pacmod12(BRAKE_AUX_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_HEADLIGHT_AUX_RPT_pacmod12(HEADLIGHT_AUX_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_HEADLIGHT_AUX_RPT_pacmod12(HEADLIGHT_AUX_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_HEADLIGHT_AUX_RPT_pacmod12(HEADLIGHT_AUX_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_PARKING_BRAKE_AUX_RPT_pacmod12(PARKING_BRAKE_AUX_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_PARKING_BRAKE_AUX_RPT_pacmod12(PARKING_BRAKE_AUX_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_PARKING_BRAKE_AUX_RPT_pacmod12(PARKING_BRAKE_AUX_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_SHIFT_AUX_RPT_pacmod12(SHIFT_AUX_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_SHIFT_AUX_RPT_pacmod12(SHIFT_AUX_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_SHIFT_AUX_RPT_pacmod12(SHIFT_AUX_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_STEERING_AUX_RPT_pacmod12(STEERING_AUX_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_STEERING_AUX_RPT_pacmod12(STEERING_AUX_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_STEERING_AUX_RPT_pacmod12(STEERING_AUX_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_TURN_AUX_RPT_pacmod12(TURN_AUX_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_TURN_AUX_RPT_pacmod12(TURN_AUX_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_TURN_AUX_RPT_pacmod12(TURN_AUX_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_WIPER_AUX_RPT_pacmod12(WIPER_AUX_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_WIPER_AUX_RPT_pacmod12(WIPER_AUX_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_WIPER_AUX_RPT_pacmod12(WIPER_AUX_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_BRAKE_DECEL_AUX_RPT_pacmod12(BRAKE_DECEL_AUX_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_BRAKE_DECEL_AUX_RPT_pacmod12(BRAKE_DECEL_AUX_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BRAKE_DECEL_AUX_RPT_pacmod12(BRAKE_DECEL_AUX_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_ENGINE_BRAKE_AUX_RPT_pacmod12(ENGINE_BRAKE_AUX_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_ENGINE_BRAKE_AUX_RPT_pacmod12(ENGINE_BRAKE_AUX_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_ENGINE_BRAKE_AUX_RPT_pacmod12(ENGINE_BRAKE_AUX_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_VEHICLE_SPEED_RPT_pacmod12(VEHICLE_SPEED_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_VEHICLE_SPEED_RPT_pacmod12(VEHICLE_SPEED_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_VEHICLE_SPEED_RPT_pacmod12(VEHICLE_SPEED_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_BRAKE_MOTOR_RPT_1_pacmod12(BRAKE_MOTOR_RPT_1_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_BRAKE_MOTOR_RPT_1_pacmod12(BRAKE_MOTOR_RPT_1_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BRAKE_MOTOR_RPT_1_pacmod12(BRAKE_MOTOR_RPT_1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_BRAKE_MOTOR_RPT_2_pacmod12(BRAKE_MOTOR_RPT_2_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_BRAKE_MOTOR_RPT_2_pacmod12(BRAKE_MOTOR_RPT_2_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BRAKE_MOTOR_RPT_2_pacmod12(BRAKE_MOTOR_RPT_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_BRAKE_MOTOR_RPT_3_pacmod12(BRAKE_MOTOR_RPT_3_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_BRAKE_MOTOR_RPT_3_pacmod12(BRAKE_MOTOR_RPT_3_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BRAKE_MOTOR_RPT_3_pacmod12(BRAKE_MOTOR_RPT_3_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_STEERING_MOTOR_RPT_1_pacmod12(STEERING_MOTOR_RPT_1_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_STEERING_MOTOR_RPT_1_pacmod12(STEERING_MOTOR_RPT_1_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_STEERING_MOTOR_RPT_1_pacmod12(STEERING_MOTOR_RPT_1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_STEERING_MOTOR_RPT_2_pacmod12(STEERING_MOTOR_RPT_2_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_STEERING_MOTOR_RPT_2_pacmod12(STEERING_MOTOR_RPT_2_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_STEERING_MOTOR_RPT_2_pacmod12(STEERING_MOTOR_RPT_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_STEERING_MOTOR_RPT_3_pacmod12(STEERING_MOTOR_RPT_3_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_STEERING_MOTOR_RPT_3_pacmod12(STEERING_MOTOR_RPT_3_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_STEERING_MOTOR_RPT_3_pacmod12(STEERING_MOTOR_RPT_3_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_WHEEL_SPEED_RPT_pacmod12(WHEEL_SPEED_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_WHEEL_SPEED_RPT_pacmod12(WHEEL_SPEED_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_WHEEL_SPEED_RPT_pacmod12(WHEEL_SPEED_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_SOFTWARE_VERSION_RPT_00_pacmod12(SOFTWARE_VERSION_RPT_00_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_SOFTWARE_VERSION_RPT_00_pacmod12(SOFTWARE_VERSION_RPT_00_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_SOFTWARE_VERSION_RPT_00_pacmod12(SOFTWARE_VERSION_RPT_00_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_SOFTWARE_VERSION_RPT_01_pacmod12(SOFTWARE_VERSION_RPT_01_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_SOFTWARE_VERSION_RPT_01_pacmod12(SOFTWARE_VERSION_RPT_01_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_SOFTWARE_VERSION_RPT_01_pacmod12(SOFTWARE_VERSION_RPT_01_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_SOFTWARE_VERSION_RPT_02_pacmod12(SOFTWARE_VERSION_RPT_02_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_SOFTWARE_VERSION_RPT_02_pacmod12(SOFTWARE_VERSION_RPT_02_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_SOFTWARE_VERSION_RPT_02_pacmod12(SOFTWARE_VERSION_RPT_02_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_SOFTWARE_VERSION_RPT_03_pacmod12(SOFTWARE_VERSION_RPT_03_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_SOFTWARE_VERSION_RPT_03_pacmod12(SOFTWARE_VERSION_RPT_03_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_SOFTWARE_VERSION_RPT_03_pacmod12(SOFTWARE_VERSION_RPT_03_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_SOFTWARE_VERSION_RPT_04_pacmod12(SOFTWARE_VERSION_RPT_04_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_SOFTWARE_VERSION_RPT_04_pacmod12(SOFTWARE_VERSION_RPT_04_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_SOFTWARE_VERSION_RPT_04_pacmod12(SOFTWARE_VERSION_RPT_04_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_YAW_RATE_RPT_pacmod12(YAW_RATE_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_YAW_RATE_RPT_pacmod12(YAW_RATE_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_YAW_RATE_RPT_pacmod12(YAW_RATE_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_LAT_LON_HEADING_RPT_pacmod12(LAT_LON_HEADING_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_LAT_LON_HEADING_RPT_pacmod12(LAT_LON_HEADING_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_LAT_LON_HEADING_RPT_pacmod12(LAT_LON_HEADING_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_DATE_TIME_RPT_pacmod12(DATE_TIME_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_DATE_TIME_RPT_pacmod12(DATE_TIME_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DATE_TIME_RPT_pacmod12(DATE_TIME_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_ENGINE_RPT_pacmod12(ENGINE_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_ENGINE_RPT_pacmod12(ENGINE_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_ENGINE_RPT_pacmod12(ENGINE_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_DETECTED_OBJECT_RPT_pacmod12(DETECTED_OBJECT_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_DETECTED_OBJECT_RPT_pacmod12(DETECTED_OBJECT_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DETECTED_OBJECT_RPT_pacmod12(DETECTED_OBJECT_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_VEH_DYNAMICS_RPT_pacmod12(VEH_DYNAMICS_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_VEH_DYNAMICS_RPT_pacmod12(VEH_DYNAMICS_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_VEH_DYNAMICS_RPT_pacmod12(VEH_DYNAMICS_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_VIN_RPT_pacmod12(VIN_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_VIN_RPT_pacmod12(VIN_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_VIN_RPT_pacmod12(VIN_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_OCCUPANCY_RPT_pacmod12(OCCUPANCY_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_OCCUPANCY_RPT_pacmod12(OCCUPANCY_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_OCCUPANCY_RPT_pacmod12(OCCUPANCY_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_INTERIOR_LIGHTS_RPT_pacmod12(INTERIOR_LIGHTS_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_INTERIOR_LIGHTS_RPT_pacmod12(INTERIOR_LIGHTS_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INTERIOR_LIGHTS_RPT_pacmod12(INTERIOR_LIGHTS_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_DOOR_RPT_pacmod12(DOOR_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_DOOR_RPT_pacmod12(DOOR_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DOOR_RPT_pacmod12(DOOR_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_REAR_LIGHTS_RPT_pacmod12(REAR_LIGHTS_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_REAR_LIGHTS_RPT_pacmod12(REAR_LIGHTS_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_REAR_LIGHTS_RPT_pacmod12(REAR_LIGHTS_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_LINEAR_ACCEL_RPT_pacmod12(LINEAR_ACCEL_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_LINEAR_ACCEL_RPT_pacmod12(LINEAR_ACCEL_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_LINEAR_ACCEL_RPT_pacmod12(LINEAR_ACCEL_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_ANG_VEL_RPT_pacmod12(ANG_VEL_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_ANG_VEL_RPT_pacmod12(ANG_VEL_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_ANG_VEL_RPT_pacmod12(ANG_VEL_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_NOTIFICATION_CMD_pacmod12(NOTIFICATION_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_NOTIFICATION_CMD_pacmod12(NOTIFICATION_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_NOTIFICATION_CMD_pacmod12(NOTIFICATION_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_ESTOP_RPT_pacmod12(ESTOP_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_ESTOP_RPT_pacmod12(ESTOP_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_ESTOP_RPT_pacmod12(ESTOP_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_USER_NOTIFICATION_CMD_pacmod12(USER_NOTIFICATION_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_USER_NOTIFICATION_CMD_pacmod12(USER_NOTIFICATION_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_USER_NOTIFICATION_CMD_pacmod12(USER_NOTIFICATION_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_TIRE_PRESSURE_RPT_pacmod12(TIRE_PRESSURE_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_TIRE_PRESSURE_RPT_pacmod12(TIRE_PRESSURE_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_TIRE_PRESSURE_RPT_pacmod12(TIRE_PRESSURE_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_DRIVE_TRAIN_FEATURE_RPT_pacmod12(DRIVE_TRAIN_FEATURE_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_DRIVE_TRAIN_FEATURE_RPT_pacmod12(DRIVE_TRAIN_FEATURE_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DRIVE_TRAIN_FEATURE_RPT_pacmod12(DRIVE_TRAIN_FEATURE_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_SAFETY_FUNC_CRITICAL_STOP_RPT_pacmod12(SAFETY_FUNC_CRITICAL_STOP_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_SAFETY_FUNC_CRITICAL_STOP_RPT_pacmod12(SAFETY_FUNC_CRITICAL_STOP_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_SAFETY_FUNC_CRITICAL_STOP_RPT_pacmod12(SAFETY_FUNC_CRITICAL_STOP_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_WATCHDOG_RPT_2_pacmod12(WATCHDOG_RPT_2_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_WATCHDOG_RPT_2_pacmod12(WATCHDOG_RPT_2_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_WATCHDOG_RPT_2_pacmod12(WATCHDOG_RPT_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_VIN_RPT_2_pacmod12(VIN_RPT_2_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_VIN_RPT_2_pacmod12(VIN_RPT_2_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_VIN_RPT_2_pacmod12(VIN_RPT_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_NOTIFICATION_RPT_pacmod12(NOTIFICATION_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_NOTIFICATION_RPT_pacmod12(NOTIFICATION_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_NOTIFICATION_RPT_pacmod12(NOTIFICATION_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_AIR_PRESSURE_RPT_pacmod12(AIR_PRESSURE_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_AIR_PRESSURE_RPT_pacmod12(AIR_PRESSURE_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_AIR_PRESSURE_RPT_pacmod12(AIR_PRESSURE_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_ENGINE_RPT_2_pacmod12(ENGINE_RPT_2_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_ENGINE_RPT_2_pacmod12(ENGINE_RPT_2_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_ENGINE_RPT_2_pacmod12(ENGINE_RPT_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_WHEEL_SPEED_RPT_2_pacmod12(WHEEL_SPEED_RPT_2_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_WHEEL_SPEED_RPT_2_pacmod12(WHEEL_SPEED_RPT_2_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_WHEEL_SPEED_RPT_2_pacmod12(WHEEL_SPEED_RPT_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

uint32_t Unpack_WATCHDOG_RPT_pacmod12(WATCHDOG_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD12_USE_CANSTRUCT
uint32_t Pack_WATCHDOG_RPT_pacmod12(WATCHDOG_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_WATCHDOG_RPT_pacmod12(WATCHDOG_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD12_USE_CANSTRUCT

#ifdef __cplusplus
}
#endif
