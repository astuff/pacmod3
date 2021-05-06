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

#ifndef PACMOD3_PACMOD3_COMMON_H
#define PACMOD3_PACMOD3_COMMON_H

#include <memory>
#include <mutex>

#include <pacmod3/pacmod3_core.h>
#include <ros/ros.h>

#include <pacmod3/AccelAuxRpt.h>
#include <pacmod3/AllSystemStatuses.h>
#include <pacmod3/AngVelRpt.h>
#include <pacmod3/BrakeAuxRpt.h>
#include <pacmod3/BrakeDecelAuxRpt.h>
#include <pacmod3/BrakeDecelCmd.h>
#include <pacmod3/CabinClimateCmd.h>
#include <pacmod3/CabinClimateRpt.h>
#include <pacmod3/ComponentRpt.h>
#include <pacmod3/DateTimeRpt.h>
#include <pacmod3/DetectedObjectRpt.h>
#include <pacmod3/DoorRpt.h>
#include <pacmod3/DriveTrainRpt.h>
#include <pacmod3/EngineRpt.h>
#include <pacmod3/EStopRpt.h>
#include <pacmod3/GlobalCmd.h>
#include <pacmod3/GlobalRpt.h>
#include <pacmod3/GlobalRpt2.h>
#include <pacmod3/HeadlightAuxRpt.h>
#include <pacmod3/InteriorLightsRpt.h>
#include <pacmod3/LatLonHeadingRpt.h>
#include <pacmod3/LinearAccelRpt.h>
#include <pacmod3/MotorRpt1.h>
#include <pacmod3/MotorRpt2.h>
#include <pacmod3/MotorRpt3.h>
#include <pacmod3/NotificationCmd.h>
#include <pacmod3/OccupancyRpt.h>
#include <pacmod3/OverrideCfgRpt.h>
#include <pacmod3/ParkingBrakeAuxRpt.h>
#include <pacmod3/RearLightsRpt.h>
#include <pacmod3/SafetyBrakeCmd.h>
#include <pacmod3/SafetyBrakeRpt.h>
#include <pacmod3/SafetyFuncCmd.h>
#include <pacmod3/SafetyFuncCriticalStopRpt.h>
#include <pacmod3/SafetyFuncRpt.h>
#include <pacmod3/ShiftAuxRpt.h>
#include <pacmod3/SoftwareVersionRpt.h>
#include <pacmod3/SteerAuxRpt.h>
#include <pacmod3/SteerSystemCmd.h>
#include <pacmod3/SteerCmdLimitRpt.h>
#include <pacmod3/SupervisoryCtrl.h>
#include <pacmod3/SystemCmdBool.h>
#include <pacmod3/SystemCmdFloat.h>
#include <pacmod3/SystemCmdInt.h>
#include <pacmod3/SystemCmdLimitRpt.h>
#include <pacmod3/SystemRptBool.h>
#include <pacmod3/SystemRptFloat.h>
#include <pacmod3/SystemRptInt.h>
#include <pacmod3/TirePressureRpt.h>
#include <pacmod3/TurnAuxRpt.h>
#include <pacmod3/VehDynamicsRpt.h>
#include <pacmod3/VehicleFaultRpt.h>
#include <pacmod3/VehicleSpeedRpt.h>
#include <pacmod3/VinRpt.h>
#include <pacmod3/VinRpt2.h>
#include <pacmod3/WatchdogRpt.h>
#include <pacmod3/WatchdogRpt2.h>
#include <pacmod3/WheelSpeedRpt.h>
#include <pacmod3/WiperAuxRpt.h>
#include <pacmod3/YawRateRpt.h>

#endif  // PACMOD3_PACMOD3_COMMON_H
