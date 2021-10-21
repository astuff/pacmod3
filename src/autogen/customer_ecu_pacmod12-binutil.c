#include "pacmod3/autogen/customer_ecu_pacmod12-binutil.h"

#ifdef __DEF_CUSTOMER_ECU_PACMOD12__

customer_ecu_pacmod12_rx_t customer_ecu_pacmod12_rx;

customer_ecu_pacmod12_tx_t customer_ecu_pacmod12_tx;

#endif // __DEF_CUSTOMER_ECU_PACMOD12__

uint32_t customer_ecu_pacmod12_Receive(customer_ecu_pacmod12_rx_t* _m, const uint8_t* _d, uint32_t _id, uint8_t dlc_)
{
 uint32_t recid = 0;
 if ((_id >= 0x10U) && (_id < 0x32CU)) {
  if ((_id >= 0x10U) && (_id < 0x220U)) {
   if ((_id >= 0x10U) && (_id < 0x200U)) {
    if ((_id >= 0x10U) && (_id < 0x23U)) {
     if ((_id >= 0x10U) && (_id < 0x20U)) {
      if (_id == 0x10U) {
       recid = Unpack_GLOBAL_RPT_pacmod12(&(_m->GLOBAL_RPT), _d, dlc_);
      } else if (_id == 0x11U) {
       recid = Unpack_GLOBAL_RPT_2_pacmod12(&(_m->GLOBAL_RPT_2), _d, dlc_);
      }
     } else {
      if (_id == 0x20U) {
       recid = Unpack_COMPONENT_RPT_00_pacmod12(&(_m->COMPONENT_RPT_00), _d, dlc_);
      } else {
       if (_id == 0x21U) {
        recid = Unpack_COMPONENT_RPT_01_pacmod12(&(_m->COMPONENT_RPT_01), _d, dlc_);
       } else if (_id == 0x22U) {
        recid = Unpack_COMPONENT_RPT_02_pacmod12(&(_m->COMPONENT_RPT_02), _d, dlc_);
       }
      }
     }
    } else {
     if ((_id >= 0x23U) && (_id < 0x40U)) {
      if (_id == 0x23U) {
       recid = Unpack_COMPONENT_RPT_03_pacmod12(&(_m->COMPONENT_RPT_03), _d, dlc_);
      } else if (_id == 0x24U) {
       recid = Unpack_COMPONENT_RPT_04_pacmod12(&(_m->COMPONENT_RPT_04), _d, dlc_);
      }
     } else {
      if (_id == 0x40U) {
       recid = Unpack_SAFETY_FUNC_RPT_pacmod12(&(_m->SAFETY_FUNC_RPT), _d, dlc_);
      } else {
       if (_id == 0x41U) {
        recid = Unpack_SAFETY_BRAKE_RPT_pacmod12(&(_m->SAFETY_BRAKE_RPT), _d, dlc_);
       } else if (_id == 0x42U) {
        recid = Unpack_VEHICLE_FAULT_RPT_pacmod12(&(_m->VEHICLE_FAULT_RPT), _d, dlc_);
       }
      }
     }
    }
   } else {
    if ((_id >= 0x200U) && (_id < 0x20CU)) {
     if ((_id >= 0x200U) && (_id < 0x204U)) {
      if (_id == 0x200U) {
       recid = Unpack_ACCEL_RPT_pacmod12(&(_m->ACCEL_RPT), _d, dlc_);
      } else if (_id == 0x201U) {
       recid = Unpack_ACCEL_CMD_LIMIT_RPT_pacmod12(&(_m->ACCEL_CMD_LIMIT_RPT), _d, dlc_);
      }
     } else {
      if (_id == 0x204U) {
       recid = Unpack_BRAKE_RPT_pacmod12(&(_m->BRAKE_RPT), _d, dlc_);
      } else {
       if (_id == 0x205U) {
        recid = Unpack_BRAKE_CMD_LIMIT_RPT_pacmod12(&(_m->BRAKE_CMD_LIMIT_RPT), _d, dlc_);
       } else if (_id == 0x208U) {
        recid = Unpack_CRUISE_CONTROL_BUTTONS_RPT_pacmod12(&(_m->CRUISE_CONTROL_BUTTONS_RPT), _d, dlc_);
       }
      }
     }
    } else {
     if ((_id >= 0x20CU) && (_id < 0x214U)) {
      if (_id == 0x20CU) {
       recid = Unpack_DASH_CONTROLS_LEFT_RPT_pacmod12(&(_m->DASH_CONTROLS_LEFT_RPT), _d, dlc_);
      } else if (_id == 0x210U) {
       recid = Unpack_DASH_CONTROLS_RIGHT_RPT_pacmod12(&(_m->DASH_CONTROLS_RIGHT_RPT), _d, dlc_);
      }
     } else {
      if (_id == 0x214U) {
       recid = Unpack_HAZARD_LIGHTS_RPT_pacmod12(&(_m->HAZARD_LIGHTS_RPT), _d, dlc_);
      } else {
       if (_id == 0x218U) {
        recid = Unpack_HEADLIGHT_RPT_pacmod12(&(_m->HEADLIGHT_RPT), _d, dlc_);
       } else if (_id == 0x21CU) {
        recid = Unpack_HORN_RPT_pacmod12(&(_m->HORN_RPT), _d, dlc_);
       }
      }
     }
    }
   }
  } else {
   if ((_id >= 0x220U) && (_id < 0x244U)) {
    if ((_id >= 0x220U) && (_id < 0x230U)) {
     if ((_id >= 0x220U) && (_id < 0x228U)) {
      if (_id == 0x220U) {
       recid = Unpack_MEDIA_CONTROLS_RPT_pacmod12(&(_m->MEDIA_CONTROLS_RPT), _d, dlc_);
      } else if (_id == 0x224U) {
       recid = Unpack_PARKING_BRAKE_RPT_pacmod12(&(_m->PARKING_BRAKE_RPT), _d, dlc_);
      }
     } else {
      if (_id == 0x228U) {
       recid = Unpack_SHIFT_RPT_pacmod12(&(_m->SHIFT_RPT), _d, dlc_);
      } else {
       if (_id == 0x22CU) {
        recid = Unpack_STEERING_RPT_pacmod12(&(_m->STEERING_RPT), _d, dlc_);
       } else if (_id == 0x22DU) {
        recid = Unpack_STEERING_CMD_LIMIT_RPT_pacmod12(&(_m->STEERING_CMD_LIMIT_RPT), _d, dlc_);
       }
      }
     }
    } else {
     if ((_id >= 0x230U) && (_id < 0x238U)) {
      if (_id == 0x230U) {
       recid = Unpack_TURN_RPT_pacmod12(&(_m->TURN_RPT), _d, dlc_);
      } else if (_id == 0x234U) {
       recid = Unpack_WIPER_RPT_pacmod12(&(_m->WIPER_RPT), _d, dlc_);
      }
     } else {
      if (_id == 0x238U) {
       recid = Unpack_SPRAYER_RPT_pacmod12(&(_m->SPRAYER_RPT), _d, dlc_);
      } else {
       if (_id == 0x23CU) {
        recid = Unpack_BRAKE_DECEL_RPT_pacmod12(&(_m->BRAKE_DECEL_RPT), _d, dlc_);
       } else if (_id == 0x240U) {
        recid = Unpack_REAR_PASS_DOOR_RPT_pacmod12(&(_m->REAR_PASS_DOOR_RPT), _d, dlc_);
       }
      }
     }
    }
   } else {
    if ((_id >= 0x244U) && (_id < 0x254U)) {
     if ((_id >= 0x244U) && (_id < 0x248U)) {
      if (_id == 0x244U) {
       recid = Unpack_ENGINE_BRAKE_RPT_pacmod12(&(_m->ENGINE_BRAKE_RPT), _d, dlc_);
      } else if (_id == 0x245U) {
       recid = Unpack_EXHAUST_BRAKE_RPT_pacmod12(&(_m->EXHAUST_BRAKE_RPT), _d, dlc_);
      }
     } else {
      if (_id == 0x248U) {
       recid = Unpack_MARKER_LAMP_RPT_pacmod12(&(_m->MARKER_LAMP_RPT), _d, dlc_);
      } else {
       if (_id == 0x24CU) {
        recid = Unpack_CABIN_TEMP_RPT_pacmod12(&(_m->CABIN_TEMP_RPT), _d, dlc_);
       } else if (_id == 0x250U) {
        recid = Unpack_CABIN_FAN_SPEED_RPT_pacmod12(&(_m->CABIN_FAN_SPEED_RPT), _d, dlc_);
       }
      }
     }
    } else {
     if ((_id >= 0x254U) && (_id < 0x318U)) {
      if (_id == 0x254U) {
       recid = Unpack_CABIN_CLIMATE_RPT_pacmod12(&(_m->CABIN_CLIMATE_RPT), _d, dlc_);
      } else {
       if (_id == 0x300U) {
        recid = Unpack_ACCEL_AUX_RPT_pacmod12(&(_m->ACCEL_AUX_RPT), _d, dlc_);
       } else if (_id == 0x304U) {
        recid = Unpack_BRAKE_AUX_RPT_pacmod12(&(_m->BRAKE_AUX_RPT), _d, dlc_);
       }
      }
     } else {
      if (_id == 0x318U) {
       recid = Unpack_HEADLIGHT_AUX_RPT_pacmod12(&(_m->HEADLIGHT_AUX_RPT), _d, dlc_);
      } else {
       if (_id == 0x324U) {
        recid = Unpack_PARKING_BRAKE_AUX_RPT_pacmod12(&(_m->PARKING_BRAKE_AUX_RPT), _d, dlc_);
       } else if (_id == 0x328U) {
        recid = Unpack_SHIFT_AUX_RPT_pacmod12(&(_m->SHIFT_AUX_RPT), _d, dlc_);
       }
      }
     }
    }
   }
  }
 } else {
  if ((_id >= 0x32CU) && (_id < 0x410U)) {
   if ((_id >= 0x32CU) && (_id < 0x405U)) {
    if ((_id >= 0x32CU) && (_id < 0x400U)) {
     if ((_id >= 0x32CU) && (_id < 0x334U)) {
      if (_id == 0x32CU) {
       recid = Unpack_STEERING_AUX_RPT_pacmod12(&(_m->STEERING_AUX_RPT), _d, dlc_);
      } else if (_id == 0x330U) {
       recid = Unpack_TURN_AUX_RPT_pacmod12(&(_m->TURN_AUX_RPT), _d, dlc_);
      }
     } else {
      if (_id == 0x334U) {
       recid = Unpack_WIPER_AUX_RPT_pacmod12(&(_m->WIPER_AUX_RPT), _d, dlc_);
      } else {
       if (_id == 0x338U) {
        recid = Unpack_BRAKE_DECEL_AUX_RPT_pacmod12(&(_m->BRAKE_DECEL_AUX_RPT), _d, dlc_);
       } else if (_id == 0x344U) {
        recid = Unpack_ENGINE_BRAKE_AUX_RPT_pacmod12(&(_m->ENGINE_BRAKE_AUX_RPT), _d, dlc_);
       }
      }
     }
    } else {
     if ((_id >= 0x400U) && (_id < 0x402U)) {
      if (_id == 0x400U) {
       recid = Unpack_VEHICLE_SPEED_RPT_pacmod12(&(_m->VEHICLE_SPEED_RPT), _d, dlc_);
      } else if (_id == 0x401U) {
       recid = Unpack_BRAKE_MOTOR_RPT_1_pacmod12(&(_m->BRAKE_MOTOR_RPT_1), _d, dlc_);
      }
     } else {
      if (_id == 0x402U) {
       recid = Unpack_BRAKE_MOTOR_RPT_2_pacmod12(&(_m->BRAKE_MOTOR_RPT_2), _d, dlc_);
      } else {
       if (_id == 0x403U) {
        recid = Unpack_BRAKE_MOTOR_RPT_3_pacmod12(&(_m->BRAKE_MOTOR_RPT_3), _d, dlc_);
       } else if (_id == 0x404U) {
        recid = Unpack_STEERING_MOTOR_RPT_1_pacmod12(&(_m->STEERING_MOTOR_RPT_1), _d, dlc_);
       }
      }
     }
    }
   } else {
    if ((_id >= 0x405U) && (_id < 0x40AU)) {
     if ((_id >= 0x405U) && (_id < 0x407U)) {
      if (_id == 0x405U) {
       recid = Unpack_STEERING_MOTOR_RPT_2_pacmod12(&(_m->STEERING_MOTOR_RPT_2), _d, dlc_);
      } else if (_id == 0x406U) {
       recid = Unpack_STEERING_MOTOR_RPT_3_pacmod12(&(_m->STEERING_MOTOR_RPT_3), _d, dlc_);
      }
     } else {
      if (_id == 0x407U) {
       recid = Unpack_WHEEL_SPEED_RPT_pacmod12(&(_m->WHEEL_SPEED_RPT), _d, dlc_);
      } else {
       if (_id == 0x408U) {
        recid = Unpack_SOFTWARE_VERSION_RPT_00_pacmod12(&(_m->SOFTWARE_VERSION_RPT_00), _d, dlc_);
       } else if (_id == 0x409U) {
        recid = Unpack_SOFTWARE_VERSION_RPT_01_pacmod12(&(_m->SOFTWARE_VERSION_RPT_01), _d, dlc_);
       }
      }
     }
    } else {
     if ((_id >= 0x40AU) && (_id < 0x40DU)) {
      if (_id == 0x40AU) {
       recid = Unpack_SOFTWARE_VERSION_RPT_02_pacmod12(&(_m->SOFTWARE_VERSION_RPT_02), _d, dlc_);
      } else {
       if (_id == 0x40BU) {
        recid = Unpack_SOFTWARE_VERSION_RPT_03_pacmod12(&(_m->SOFTWARE_VERSION_RPT_03), _d, dlc_);
       } else if (_id == 0x40CU) {
        recid = Unpack_SOFTWARE_VERSION_RPT_04_pacmod12(&(_m->SOFTWARE_VERSION_RPT_04), _d, dlc_);
       }
      }
     } else {
      if (_id == 0x40DU) {
       recid = Unpack_YAW_RATE_RPT_pacmod12(&(_m->YAW_RATE_RPT), _d, dlc_);
      } else {
       if (_id == 0x40EU) {
        recid = Unpack_LAT_LON_HEADING_RPT_pacmod12(&(_m->LAT_LON_HEADING_RPT), _d, dlc_);
       } else if (_id == 0x40FU) {
        recid = Unpack_DATE_TIME_RPT_pacmod12(&(_m->DATE_TIME_RPT), _d, dlc_);
       }
      }
     }
    }
   }
  } else {
   if ((_id >= 0x410U) && (_id < 0x41CU)) {
    if ((_id >= 0x410U) && (_id < 0x416U)) {
     if ((_id >= 0x410U) && (_id < 0x413U)) {
      if (_id == 0x410U) {
       recid = Unpack_ENGINE_RPT_pacmod12(&(_m->ENGINE_RPT), _d, dlc_);
      } else if (_id == 0x411U) {
       recid = Unpack_DETECTED_OBJECT_RPT_pacmod12(&(_m->DETECTED_OBJECT_RPT), _d, dlc_);
      }
     } else {
      if (_id == 0x413U) {
       recid = Unpack_VEH_DYNAMICS_RPT_pacmod12(&(_m->VEH_DYNAMICS_RPT), _d, dlc_);
      } else {
       if (_id == 0x414U) {
        recid = Unpack_VIN_RPT_pacmod12(&(_m->VIN_RPT), _d, dlc_);
       } else if (_id == 0x415U) {
        recid = Unpack_OCCUPANCY_RPT_pacmod12(&(_m->OCCUPANCY_RPT), _d, dlc_);
       }
      }
     }
    } else {
     if ((_id >= 0x416U) && (_id < 0x418U)) {
      if (_id == 0x416U) {
       recid = Unpack_INTERIOR_LIGHTS_RPT_pacmod12(&(_m->INTERIOR_LIGHTS_RPT), _d, dlc_);
      } else if (_id == 0x417U) {
       recid = Unpack_DOOR_RPT_pacmod12(&(_m->DOOR_RPT), _d, dlc_);
      }
     } else {
      if (_id == 0x418U) {
       recid = Unpack_REAR_LIGHTS_RPT_pacmod12(&(_m->REAR_LIGHTS_RPT), _d, dlc_);
      } else {
       if (_id == 0x419U) {
        recid = Unpack_LINEAR_ACCEL_RPT_pacmod12(&(_m->LINEAR_ACCEL_RPT), _d, dlc_);
       } else if (_id == 0x41AU) {
        recid = Unpack_ANG_VEL_RPT_pacmod12(&(_m->ANG_VEL_RPT), _d, dlc_);
       }
      }
     }
    }
   } else {
    if ((_id >= 0x41CU) && (_id < 0x422U)) {
     if ((_id >= 0x41CU) && (_id < 0x41FU)) {
      if (_id == 0x41CU) {
       recid = Unpack_ESTOP_RPT_pacmod12(&(_m->ESTOP_RPT), _d, dlc_);
      } else if (_id == 0x41EU) {
       recid = Unpack_TIRE_PRESSURE_RPT_pacmod12(&(_m->TIRE_PRESSURE_RPT), _d, dlc_);
      }
     } else {
      if (_id == 0x41FU) {
       recid = Unpack_DRIVE_TRAIN_FEATURE_RPT_pacmod12(&(_m->DRIVE_TRAIN_FEATURE_RPT), _d, dlc_);
      } else {
       if (_id == 0x420U) {
        recid = Unpack_SAFETY_FUNC_CRITICAL_STOP_RPT_pacmod12(&(_m->SAFETY_FUNC_CRITICAL_STOP_RPT), _d, dlc_);
       } else if (_id == 0x421U) {
        recid = Unpack_WATCHDOG_RPT_2_pacmod12(&(_m->WATCHDOG_RPT_2), _d, dlc_);
       }
      }
     }
    } else {
     if ((_id >= 0x422U) && (_id < 0x425U)) {
      if (_id == 0x422U) {
       recid = Unpack_VIN_RPT_2_pacmod12(&(_m->VIN_RPT_2), _d, dlc_);
      } else {
       if (_id == 0x423U) {
        recid = Unpack_NOTIFICATION_RPT_pacmod12(&(_m->NOTIFICATION_RPT), _d, dlc_);
       } else if (_id == 0x424U) {
        recid = Unpack_AIR_PRESSURE_RPT_pacmod12(&(_m->AIR_PRESSURE_RPT), _d, dlc_);
       }
      }
     } else {
      if (_id == 0x425U) {
       recid = Unpack_ENGINE_RPT_2_pacmod12(&(_m->ENGINE_RPT_2), _d, dlc_);
      } else {
       if (_id == 0x426U) {
        recid = Unpack_WHEEL_SPEED_RPT_2_pacmod12(&(_m->WHEEL_SPEED_RPT_2), _d, dlc_);
       } else if (_id == 0x600U) {
        recid = Unpack_WATCHDOG_RPT_pacmod12(&(_m->WATCHDOG_RPT), _d, dlc_);
       }
      }
     }
    }
   }
  }
 }

 return recid;
}

