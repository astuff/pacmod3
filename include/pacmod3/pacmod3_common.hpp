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

#include <pacmod_msgs/msg/system_cmd_bool.hpp>
#include <pacmod_msgs/msg/system_cmd_float.hpp>
#include <pacmod_msgs/msg/system_cmd_int.hpp>
#include <pacmod_msgs/msg/global_rpt.hpp>
#include <pacmod_msgs/msg/accel_aux_rpt.hpp>
#include <pacmod_msgs/msg/all_system_statuses.hpp>
#include <pacmod_msgs/msg/brake_aux_rpt.hpp>
#include <pacmod_msgs/msg/component_rpt.hpp>
#include <pacmod_msgs/msg/date_time_rpt.hpp>
#include <pacmod_msgs/msg/detected_object_rpt.hpp>
#include <pacmod_msgs/msg/door_rpt.hpp>
#include <pacmod_msgs/msg/headlight_aux_rpt.hpp>
#include <pacmod_msgs/msg/interior_lights_rpt.hpp>
#include <pacmod_msgs/msg/lat_lon_heading_rpt.hpp>
#include <pacmod_msgs/msg/motor_rpt1.hpp>
#include <pacmod_msgs/msg/motor_rpt2.hpp>
#include <pacmod_msgs/msg/motor_rpt3.hpp>
#include <pacmod_msgs/msg/occupancy_rpt.hpp>
#include <pacmod_msgs/msg/rear_lights_rpt.hpp>
#include <pacmod_msgs/msg/shift_aux_rpt.hpp>
#include <pacmod_msgs/msg/steer_aux_rpt.hpp>
#include <pacmod_msgs/msg/steer_system_cmd.hpp>
#include <pacmod_msgs/msg/system_rpt_bool.hpp>
#include <pacmod_msgs/msg/system_rpt_float.hpp>
#include <pacmod_msgs/msg/system_rpt_int.hpp>
#include <pacmod_msgs/msg/turn_aux_rpt.hpp>
#include <pacmod_msgs/msg/vehicle_dynamics_rpt.hpp>
#include <pacmod_msgs/msg/vehicle_specific_rpt1.hpp>
#include <pacmod_msgs/msg/vehicle_speed_rpt.hpp>
#include <pacmod_msgs/msg/vin_rpt.hpp>
#include <pacmod_msgs/msg/wheel_speed_rpt.hpp>
#include <pacmod_msgs/msg/wiper_aux_rpt.hpp>
#include <pacmod_msgs/msg/yaw_rate_rpt.hpp>

#endif  // PACMOD3__PACMOD3_COMMON_HPP_
