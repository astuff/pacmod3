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

void DbcApi::SetDbcVersion(uint32_t dbc_major_version)
{
  dbc_major_version_ = dbc_major_version;
}

uint32_t DbcApi::GetDbcVersion()
{
  return dbc_major_version_;
}

void DbcApi::PrintParseError(const std::string& msg_type)
{
  std::string full_msg = "Unable to parse " + msg_type + ", it is not supported by DBC version " + std::to_string(dbc_major_version_);

  #if ROS_VERSION==1
    ROS_WARN_STREAM(full_msg);
  #endif

  #if ROS_VERSION==2

  #endif
}

void DbcApi::PrintEncodeError(const std::string& msg_type)
{
  std::string full_msg = "Unable to encode " + msg_type + ", it is not supported by DBC version " + std::to_string(dbc_major_version_);

  #if ROS_VERSION==1
    ROS_WARN_STREAM(full_msg);
  #endif

  #if ROS_VERSION==2

  #endif
}

}  // namespace pacmod3
