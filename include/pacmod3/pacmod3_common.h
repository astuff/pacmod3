#ifndef PACMOD3_COMMON_H
#define PACMOD3_COMMON_H

/*
* Unpublished Copyright (c) 2009-2017 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include <memory>
#include <mutex>

#include <pacmod3_core.h>
#include <ros/ros.h>

#include <pacmod_msgs/SystemCmdBool.h>
#include <pacmod_msgs/SystemCmdFloat.h>
#include <pacmod_msgs/SystemCmdInt.h>
#include <pacmod_msgs/GlobalRpt.h>
#include <pacmod_msgs/AccelAuxRpt.h>
#include <pacmod_msgs/AllSystemStatuses.h>
#include <pacmod_msgs/BrakeAuxRpt.h>
#include <pacmod_msgs/ComponentRpt.h>
#include <pacmod_msgs/DateTimeRpt.h>
#include <pacmod_msgs/DetectedObjectRpt.h>
#include <pacmod_msgs/DoorRpt.h>
#include <pacmod_msgs/HeadlightAuxRpt.h>
#include <pacmod_msgs/InteriorLightsRpt.h>
#include <pacmod_msgs/LatLonHeadingRpt.h>
#include <pacmod_msgs/MotorRpt1.h>
#include <pacmod_msgs/MotorRpt2.h>
#include <pacmod_msgs/MotorRpt3.h>
#include <pacmod_msgs/OccupancyRpt.h>
#include <pacmod_msgs/RearLightsRpt.h>
#include <pacmod_msgs/ShiftAuxRpt.h>
#include <pacmod_msgs/SteerAuxRpt.h>
#include <pacmod_msgs/SteeringPIDRpt1.h>
#include <pacmod_msgs/SteeringPIDRpt2.h>
#include <pacmod_msgs/SteeringPIDRpt3.h>
#include <pacmod_msgs/SteeringPIDRpt4.h>
#include <pacmod_msgs/SteerSystemCmd.h>
#include <pacmod_msgs/SystemRptBool.h>
#include <pacmod_msgs/SystemRptFloat.h>
#include <pacmod_msgs/SystemRptInt.h>
#include <pacmod_msgs/TurnAuxRpt.h>
#include <pacmod_msgs/VehicleDynamicsRpt.h>
#include <pacmod_msgs/VehicleSpecificRpt1.h>
#include <pacmod_msgs/VehicleSpeedRpt.h>
#include <pacmod_msgs/VinRpt.h>
#include <pacmod_msgs/WheelSpeedRpt.h>
#include <pacmod_msgs/WiperAuxRpt.h>
#include <pacmod_msgs/YawRateRpt.h>

#endif
