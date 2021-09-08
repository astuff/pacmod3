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

#include <rclcpp/rclcpp.hpp>

#include <pacmod3_msgs/msg/system_cmd_bool.hpp>
#include <pacmod3_msgs/msg/system_cmd_float.hpp>
#include <pacmod3_msgs/msg/system_cmd_int.hpp>
#include <pacmod3_msgs/msg/global_rpt.hpp>
#include <pacmod3_msgs/msg/accel_aux_rpt.hpp>
#include <pacmod3_msgs/msg/all_system_statuses.hpp>
#include <pacmod3_msgs/msg/brake_aux_rpt.hpp>
#include <pacmod3_msgs/msg/component_rpt.hpp>
#include <pacmod3_msgs/msg/date_time_rpt.hpp>
#include <pacmod3_msgs/msg/detected_object_rpt.hpp>
#include <pacmod3_msgs/msg/door_rpt.hpp>
#include <pacmod3_msgs/msg/headlight_aux_rpt.hpp>
#include <pacmod3_msgs/msg/interior_lights_rpt.hpp>
#include <pacmod3_msgs/msg/lat_lon_heading_rpt.hpp>
#include <pacmod3_msgs/msg/motor_rpt1.hpp>
#include <pacmod3_msgs/msg/motor_rpt2.hpp>
#include <pacmod3_msgs/msg/motor_rpt3.hpp>
#include <pacmod3_msgs/msg/occupancy_rpt.hpp>
#include <pacmod3_msgs/msg/rear_lights_rpt.hpp>
#include <pacmod3_msgs/msg/shift_aux_rpt.hpp>
#include <pacmod3_msgs/msg/steering_aux_rpt.hpp>
#include <pacmod3_msgs/msg/steering_cmd.hpp>
#include <pacmod3_msgs/msg/system_rpt_bool.hpp>
#include <pacmod3_msgs/msg/system_rpt_float.hpp>
#include <pacmod3_msgs/msg/system_rpt_int.hpp>
#include <pacmod3_msgs/msg/turn_aux_rpt.hpp>
#include <pacmod3_msgs/msg/vehicle_dynamics_rpt.hpp>
#include <pacmod3_msgs/msg/vehicle_speed_rpt.hpp>
#include <pacmod3_msgs/msg/vin_rpt.hpp>
#include <pacmod3_msgs/msg/wheel_speed_rpt.hpp>
#include <pacmod3_msgs/msg/wiper_aux_rpt.hpp>
#include <pacmod3_msgs/msg/yaw_rate_rpt.hpp>

#endif  // PACMOD3__PACMOD3_COMMON_HPP_
