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

#ifndef PACMOD3_PACMOD3_ROS_MSG_HANDLER_H
#define PACMOD3_PACMOD3_ROS_MSG_HANDLER_H


#include "pacmod3_dbc_ros_api.h"
#include "pacmod3_dbc12_ros_api.h"

#include <string>
#include <vector>
#include <memory>
#include <mutex>

#include <ros/ros.h>

namespace pacmod3
{
class LockedData
{
public:
  explicit LockedData(unsigned char data_length = 8U);

  std::vector<unsigned char> getData() const;
  void setData(std::vector<unsigned char> new_data);

private:
  std::vector<unsigned char> _data;
  mutable std::mutex _data_mut;
};

class Pacmod3RosMsgHandler
{
public:
  Pacmod3RosMsgHandler(uint32_t dbc_major_version);

  // Main parsing and publishing function, call this from the driver
  void ParseAndPublish(const can_msgs::Frame& can_msg, const ros::Publisher& pub);

  // Parse and publish a certain ROS msg type
  template <class RosMsgType>
  void ParseAndPublishType(const can_msgs::Frame& can_msg, const ros::Publisher& pub)
  {
    // Call generic fill function from common hybrid lib, cast void pointer return
    if (parse_functions.count(can_msg.id))
    {
      std::shared_ptr<RosMsgType> ros_msg = std::static_pointer_cast<RosMsgType>(parse_functions[can_msg.id](can_msg));

      ros_msg->header.frame_id = "pacmod3";
      ros_msg->header.stamp = can_msg.header.stamp;

      pub.publish(*ros_msg);
    }
  }

  // Parse a CAN msg into a ROS msg
  template <class RosMsgType>
  bool ParseType(const can_msgs::Frame& can_msg, RosMsgType& ros_msg)
  {
    // Call generic fill function from common hybrid lib, cast void pointer return
    if (parse_functions.count(can_msg.id))
    {
      ros_msg = *(std::static_pointer_cast<RosMsgType>(parse_functions[can_msg.id](can_msg)));

      ros_msg.header.frame_id = "pacmod3";
      ros_msg.header.stamp = can_msg.header.stamp;

      return true;
    }
    else
    {
      ros_msg = RosMsgType();
      return false;
    }
  }

  // Function for packing/encoding data into CAN messages. It's located in the header to force
  // the compiler to instatiate types required by the includer.
  template <class RosMsgType>
  can_msgs::Frame Encode(uint32_t can_id, const RosMsgType& msg)
  {
    can_msgs::Frame new_frame;
    new_frame = msg_api_->EncodeCmd(*msg);
    new_frame.id = can_id;
    return new_frame;
  }

private:
  std::unique_ptr<pacmod3_common::DbcApi> msg_api_;

  // List of functions for parsing CAN frames into ROS msgs
  std::map<uint32_t, std::function<std::shared_ptr<void>(const can_msgs::Frame&)>> parse_functions;

  // List of function for matching publishers with the type of message they are publishing
  std::map<uint32_t, std::function<void(const can_msgs::Frame&, const ros::Publisher&)>> pub_functions;
};

}  // namespace pacmod3

#endif  // PACMOD3_PACMOD3_ROS_MSG_HANDLER_H
