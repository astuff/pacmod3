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

#include <pacmod3_dbc_ros_api.h>

#include <string>

namespace pacmod3
{

DbcApi::PrintParseError(const std::string& msg_type)
{
  std::string full_msg = "Unable to parse " + msg_type + ", it is not supported by DBC version " + str(dbc_major_version_);

  #ifdef USE_ROS1
    ROS_WARN(full_msg);
  #endif  // USE_ROS1

  #ifdef USE_ROS2

  #endif  // USE_ROS2
}

DbcApi::PrintEncodeError(const std::string& msg_type)
{
  std::string full_msg = "Unable to encode " + msg_type + ", it is not supported by DBC version " + str(dbc_major_version_);

  #ifdef USE_ROS1
    ROS_WARN(full_msg);
  #endif  // USE_ROS1

  #ifdef USE_ROS2

  #endif  // USE_ROS2
}
