#ifndef PACMOD3_ROS_MSG_HANDLER_H
#define PACMOD3_ROS_MSG_HANDLER_H

/*
* Unpublished Copyright (c) 2009-2017 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include <pacmod3_common.h>

namespace AS
{
namespace Drivers
{
namespace PACMod3
{
  class LockedData
  {
    public:
      LockedData();

      std::vector<unsigned char> getData() const;
      void setData(std::vector<unsigned char> new_data);

    private:
      std::vector<unsigned char> _data;
      mutable std::mutex _data_mut;
  };

  class Pacmod3TxRosMsgHandler
  {
    public:
      void fillAndPublish(const int64_t& can_id,
                          std::string frame_id,
                          ros::Publisher& pub,
                          std::shared_ptr<Pacmod3TxMsg>& parser_class);

    private:
      void fillSystemRptBool(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::SystemRptBool& new_msg, std::string frame_id);
      void fillSystemRptInt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::SystemRptInt& new_msg, std::string frame_id);
      void fillSystemRptFloat(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::SystemRptFloat& new_msg, std::string frame_id);
      void fillGlobalRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::GlobalRpt& new_msg, std::string frame_id);
      void fillComponentRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::ComponentRpt& new_msg, std::string frame_id);
      void fillAccelAuxRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::AccelAuxRpt& new_msg, std::string frame_id);
      void fillBrakeAuxRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::BrakeAuxRpt& new_msg, std::string frame_id);
      void fillDateTimeRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::DateTimeRpt& new_msg, std::string frame_id);
      void fillDetectedObjectRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::DetectedObjectRpt& new_msg, std::string frame_id);
      void fillDoorRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::DoorRpt& new_msg, std::string frame_id);
      void fillHeadlightAuxRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::HeadlightAuxRpt& new_msg, std::string frame_id);
      void fillInteriorLightsRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::InteriorLightsRpt& new_msg, std::string frame_id);
      void fillLatLonHeadingRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::LatLonHeadingRpt& new_msg, std::string frame_id);
      void fillMotorRpt1(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::MotorRpt1& new_msg, std::string frame_id);
      void fillMotorRpt2(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::MotorRpt2& new_msg, std::string frame_id);
      void fillMotorRpt3(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::MotorRpt3& new_msg, std::string frame_id);
      void fillOccupancyRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::OccupancyRpt& new_msg, std::string frame_id);
      void fillRearLightsRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::RearLightsRpt& new_msg, std::string frame_id);
      void fillShiftAuxRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::ShiftAuxRpt& new_msg, std::string frame_id);
      void fillSteerAuxRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::SteerAuxRpt& new_msg, std::string frame_id);
      void fillSteeringPIDRpt1(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::SteeringPIDRpt1& new_msg, std::string frame_id);
      void fillSteeringPIDRpt2(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::SteeringPIDRpt2& new_msg, std::string frame_id);
      void fillSteeringPIDRpt3(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::SteeringPIDRpt3& new_msg, std::string frame_id);
      void fillSteeringPIDRpt4(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::SteeringPIDRpt4& new_msg, std::string frame_id);
      void fillTurnAuxRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::TurnAuxRpt& new_msg, std::string frame_id);
      void fillVehicleSpecificRpt1(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::VehicleSpecificRpt1& new_msg, std::string frame_id);
      void fillVehicleDynamicsRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::VehicleDynamicsRpt& new_msg, std::string frame_id);
      void fillVehicleSpeedRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::VehicleSpeedRpt& new_msg, std::string frame_id);
      void fillVinRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::VinRpt& new_msg, std::string frame_id);
      void fillWheelSpeedRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::WheelSpeedRpt& new_msg, std::string frame_id);
      void fillWiperAuxRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::WiperAuxRpt& new_msg, std::string frame_id);
      void fillYawRateRpt(std::shared_ptr<Pacmod3TxMsg>& parser_class, pacmod_msgs::YawRateRpt& new_msg, std::string frame_id);
  };

  class Pacmod3RxRosMsgHandler
  {
    public:
      static std::vector<uint8_t> unpackAndEncode(const int64_t& can_id, const pacmod_msgs::SystemCmdBool::ConstPtr& msg);
      static std::vector<uint8_t> unpackAndEncode(const int64_t& can_id, const pacmod_msgs::SystemCmdFloat::ConstPtr& msg);
      static std::vector<uint8_t> unpackAndEncode(const int64_t& can_id, const pacmod_msgs::SystemCmdInt::ConstPtr& msg);
      static std::vector<uint8_t> unpackAndEncode(const int64_t& can_id, const pacmod_msgs::SteerSystemCmd::ConstPtr& msg);
  };
}
}
}

#endif
