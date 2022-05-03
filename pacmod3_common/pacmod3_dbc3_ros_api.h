// Copyright (c) 2022 AutonomouStuff, LLC
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

#ifndef PACMOD3_DBC3_ROS_API_H
#define PACMOD3_DBC3_ROS_API_H

#include "pacmod3_dbc_ros_api.h"

#include <string>
#include <vector>
#include <memory>
#include <mutex>

namespace pacmod3_common
{

class Dbc3Api : public DbcApi
{
public:
  Dbc3Api();

  std::shared_ptr<void> ParseAccelAuxRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseAngVelRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseBrakeAuxRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseComponentRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseDateTimeRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseDetectedObjectRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseDoorRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseEngineRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseGlobalRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseHeadlightAuxRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseInteriorLightsRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseLatLonHeadingRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseLinearAccelRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseMotorRpt1(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseMotorRpt2(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseMotorRpt3(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseOccupancyRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseRearLightsRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseShiftAuxRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseSteeringAuxRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseSystemRptBool(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseSystemRptFloat(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseSystemRptInt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseTurnAuxRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseVehicleDynamicsRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseVehicleSpeedRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseVinRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseWheelSpeedRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseWiperAuxRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseYawRateRpt(const cn_msgs::Frame& can_msg) override;

  cn_msgs::Frame EncodeCmd(const pm_msgs::GlobalCmd& msg) override;
  cn_msgs::Frame EncodeCmd(const pm_msgs::NotificationCmd& msg) override;
  cn_msgs::Frame EncodeCmd(const pm_msgs::SteeringCmd& msg) override;
  cn_msgs::Frame EncodeCmd(const pm_msgs::SystemCmdBool& msg) override;
  cn_msgs::Frame EncodeCmd(const pm_msgs::SystemCmdFloat& msg) override;
  cn_msgs::Frame EncodeCmd(const pm_msgs::SystemCmdInt& msg) override;
};
}  // namespace pacmod3_common

#endif  // PACMOD3_DBC3_ROS_API_H
