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
#include <pacmod_msgs/EngineRpt.h>
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

#endif  // PACMOD3_PACMOD3_COMMON_H
