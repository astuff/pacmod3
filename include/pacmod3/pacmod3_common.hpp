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

#ifndef PACMOD3__PACMOD3_COMMON_HPP_
#define PACMOD3__PACMOD3_COMMON_HPP_

#include "pacmod3/pacmod3_core.hpp"

#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <pacmod_msgs/msg/SystemCmdBool.hpp>
#include <pacmod_msgs/msg/SystemCmdFloat.hpp>
#include <pacmod_msgs/msg/SystemCmdInt.hpp>
#include <pacmod_msgs/msg/GlobalRpt.hpp>
#include <pacmod_msgs/msg/AccelAuxRpt.hpp>
#include <pacmod_msgs/msg/AllSystemStatuses.hpp>
#include <pacmod_msgs/msg/BrakeAuxRpt.hpp>
#include <pacmod_msgs/msg/ComponentRpt.hpp>
#include <pacmod_msgs/msg/DateTimeRpt.hpp>
#include <pacmod_msgs/msg/DetectedObjectRpt.hpp>
#include <pacmod_msgs/msg/DoorRpt.hpp>
#include <pacmod_msgs/msg/HeadlightAuxRpt.hpp>
#include <pacmod_msgs/msg/InteriorLightsRpt.hpp>
#include <pacmod_msgs/msg/LatLonHeadingRpt.hpp>
#include <pacmod_msgs/msg/MotorRpt1.hpp>
#include <pacmod_msgs/msg/MotorRpt2.hpp>
#include <pacmod_msgs/msg/MotorRpt3.hpp>
#include <pacmod_msgs/msg/OccupancyRpt.hpp>
#include <pacmod_msgs/msg/RearLightsRpt.hpp>
#include <pacmod_msgs/msg/ShiftAuxRpt.hpp>
#include <pacmod_msgs/msg/SteerAuxRpt.hpp>
#include <pacmod_msgs/msg/SteeringPIDRpt1.hpp>
#include <pacmod_msgs/msg/SteeringPIDRpt2.hpp>
#include <pacmod_msgs/msg/SteeringPIDRpt3.hpp>
#include <pacmod_msgs/msg/SteeringPIDRpt4.hpppp>
#include <pacmod_msgs/msg/SteerSystemCmd.hpp>
#include <pacmod_msgs/msg/SystemRptBool.hpp>
#include <pacmod_msgs/msg/SystemRptFloat.hpp>
#include <pacmod_msgs/msg/SystemRptInt.hpp>
#include <pacmod_msgs/msg/TurnAuxRpt.hpp>
#include <pacmod_msgs/msg/VehicleDynamicsRpt.hpp>
#include <pacmod_msgs/msg/VehicleSpecificRpt1.hpp>
#include <pacmod_msgs/msg/VehicleSpeedRpt.hpp>
#include <pacmod_msgs/msg/VinRpt.hpp>
#include <pacmod_msgs/msg/WheelSpeedRpt.hpp>
#include <pacmod_msgs/msg/WiperAuxRpt.hpp>
#include <pacmod_msgs/msg/YawRateRpt.hpp>

#endif  // PACMOD3__PACMOD3_COMMON_HPP_
