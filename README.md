# PACMod3 (Platform Actuation and Control MODule) ROS Driver #

[![CircleCI](https://circleci.com/gh/astuff/pacmod3/tree/ros2_master.svg?style=svg)](https://circleci.com/gh/astuff/pacmod3/tree/ros2_master)

This ROS node is designed to allow the user to control a vehicle (see SUPPORTED VEHICLES below) with the PACMod drive-by-wire system, board revision 3.
The main purpose of the driver is to provide a common ROS API to PACMod devices regardless of vehicle type or specific PACMod version in use.

For access to the DBC file which defines the CAN interface for the PACMod, see the [pacmod_dbc](https://github.com/astuff/pacmod_dbc) repo.

## Installation 

Install pacmod3 using our debian repository:

```sh
sudo apt install apt-transport-https
sudo sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list'
sudo apt update
sudo apt install ros-$ROS_DISTRO-pacmod3
```

Note: Previously the pacmod3 driver was released via the ROS buildfarm. 
This has changed as of Ubuntu 20.04 (ROS2 Foxy and ROS1 Noetic) to keep old package versions available for download, which gives users greater control over their installed software and also allows downgrades if an upgrade breaks software dependencies.

## ROS API

The driver will automatically adapt the ROS API (published and subscribed topics) according to what data the PACMod system supports. 
Please consult the PACMod user manual you received with your vehicle in order to determine what types of data the PACMod system is able to send and receive.

### Launch Arguments

- **use_kvaser**: Set this to true if a Kvaser CAN device is being used with Kvaser canlib drivers to connect to the PACMod. Defaults to `false`.
- **kvaser_hardware_id**: The hardware id of the kvaser device, only applies if `use_kvaser` is true.
- **kvaser_circuit_id**: The circuit/channel id that the PACMod is plugged into on the kvaser device, only applies if `use_kvaser` is true.
- **use_socketcan**: Set this to true if Linux SocketCAN drivers are being used to connect to the PACMod. Defaults to `false`.
- **socketcan_device**: The device id of the SocketCAN channel the PACMod is plugged into, only applies if `use_socketcan` is true.
- **namespace**: The namespace of the PACMod driver, topics will be namespaced accordingly. Defaults to `pacmod`.

### Published Topics

Topics published on all platforms:

- `accel_rpt` ([pacmod3_msgs/SystemRptFloat](https://github.com/astuff/pacmod3_msgs/blob/master/msg/SystemRptFloat.msg))
- `accel_aux_rpt` ([pacmod3_msgs/AccelAuxRpt](https://github.com/astuff/pacmod3_msgs/blob/master/msg/AccelAuxRpt.msg))
- `brake_rpt` ([pacmod3_msgs/SystemRptFloat](https://github.com/astuff/pacmod3_msgs/blob/master/msg/SystemRptFloat.msg))
- `brake_aux_rpt` ([pacmod3_msgs/BrakeAuxRpt](https://github.com/astuff/pacmod3_msgs/blob/master/msg/BrakeAuxRpt.msg))
- `steering_rpt` ([pacmod3_msgs/SystemRptFloat](https://github.com/astuff/pacmod3_msgs/blob/master/msg/SystemRptFloat.msg))
- `steering_aux_rpt` ([pacmod3_msgs/SteeringAuxRpt](https://github.com/astuff/pacmod3_msgs/blob/master/msg/SteeringAuxRpt.msg))
- `shift_rpt` ([pacmod3_msgs/SystemRptInt](https://github.com/astuff/pacmod3_msgs/blob/master/msg/SystemRptInt.msg))
- `shift_aux_rpt` ([pacmod3_msgs/ShiftAuxRpt](https://github.com/astuff/pacmod3_msgs/blob/master/msg/ShiftAuxRpt.msg))
- `turn_rpt` ([pacmod3_msgs/SystemRptInt](https://github.com/astuff/pacmod3_msgs/blob/master/msg/SystemRptInt.msg))
- `turn_aux_rpt` ([pacmod3_msgs/TurnAuxRpt](https://github.com/astuff/pacmod3_msgs/blob/master/msg/TurnAuxRpt.msg))
- `vehicle_speed_rpt` ([pacmod3_msgs/VehicleSpeedRpt](https://github.com/astuff/pacmod3_msgs/blob/master/msg/VehicleSpeedRpt.msg))
- `vin_rpt` ([pacmod3_msgs/VinRpt](https://github.com/astuff/pacmod3_msgs/blob/master/msg/VinRpt.msg))
- `global_rpt` ([pacmod3_msgs/GlobalRpt](https://github.com/astuff/pacmod3_msgs/blob/master/msg/GlobalRpt.msg))
- `component_rpt` ([pacmod3_msgs/ComponentRpt](https://github.com/astuff/pacmod3_msgs/blob/master/msg/ComponentRpt.msg))
- `vehicle_speed` ([std_msgs/Float64](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html))
- `all_system_statuses` ([pacmod3_msgs/AllSystemStatuses](https://github.com/astuff/pacmod3_msgs/blob/master/msg/AllSystemStatuses.msg))
- `can_rx` ([can_msgs/Frame](http://docs.ros.org/en/noetic/api/can_msgs/html/msg/Frame.html))

Topics published on supported platforms only:

- `brake_motor_rpt_1` ([pacmod3_msgs/MotorRpt1](https://github.com/astuff/pacmod3_msgs/blob/master/msg/MotorRpt1.msg))
- `brake_motor_rpt_2` ([pacmod3_msgs/MotorRpt2](https://github.com/astuff/pacmod3_msgs/blob/master/msg/MotorRpt2.msg))
- `brake_motor_rpt_3` ([pacmod3_msgs/MotorRpt3](https://github.com/astuff/pacmod3_msgs/blob/master/msg/MotorRpt3.msg))
- `steering_motor_rpt_1` ([pacmod3_msgs/MotorRpt1](https://github.com/astuff/pacmod3_msgs/blob/master/msg/MotorRpt1.msg))
- `steering_motor_rpt_2` ([pacmod3_msgs/MotorRpt2](https://github.com/astuff/pacmod3_msgs/blob/master/msg/MotorRpt2.msg))
- `steering_motor_rpt_3` ([pacmod3_msgs/MotorRpt3](https://github.com/astuff/pacmod3_msgs/blob/master/msg/MotorRpt3.msg))
- `wiper_rpt` ([pacmod3_msgs/SystemRptInt](https://github.com/astuff/pacmod3_msgs/blob/master/msg/SystemRptInt.msg))
- `wiper_aux_rpt` ([pacmod3_msgs/WiperAuxRpt](https://github.com/astuff/pacmod3_msgs/blob/master/msg/WiperAuxRpt.msg))
- `headlight_rpt` ([pacmod3_msgs/SystemRptInt](https://github.com/astuff/pacmod3_msgs/blob/master/msg/SystemRptInt.msg))
- `headlight_aux_rpt` ([pacmod3_msgs/HeadlightAuxRpt](https://github.com/astuff/pacmod3_msgs/blob/master/msg/HeadlightAuxRpt.msg))
- `horn_rpt` ([pacmod3_msgs/SystemRptBool](https://github.com/astuff/pacmod3_msgs/blob/master/msg/SystemRptBool.msg))
- `wheel_speed_rpt` ([pacmod3_msgs/WheelSpeedRpt](https://github.com/astuff/pacmod3_msgs/blob/master/msg/WheelSpeedRpt.msg))
- `parking_brake_rpt` ([pacmod3_msgs/SystemRptBool](https://github.com/astuff/pacmod3_msgs/blob/master/msg/SystemRptBool.msg))
- `door_rpt` ([pacmod3_msgs/DoorRpt](https://github.com/astuff/pacmod3_msgs/blob/master/msg/DoorRpt.msg))
- `interior_lights_rpt` ([pacmod3_msgs/InteriorLightsRpt](https://github.com/astuff/pacmod3_msgs/blob/master/msg/InteriorLightsRpt.msg))
- `occupancy_rpt` ([pacmod3_msgs/OccupancyRpt](https://github.com/astuff/pacmod3_msgs/blob/master/msg/OccupancyRpt.msg))
- `rear_lights_rpt` ([pacmod3_msgs/RearLightsRpt](https://github.com/astuff/pacmod3_msgs/blob/master/msg/RearLightsRpt.msg))
- `hazard_lights_rpt` ([pacmod3_msgs/SystemRptBool](https://github.com/astuff/pacmod3_msgs/blob/master/msg/SystemRptBool.msg))
- `date_time_rpt` ([pacmod3_msgs/DateTimeRpt](https://github.com/astuff/pacmod3_msgs/blob/master/msg/DateTimeRpt.msg))
- `lat_lon_heading_rpt` ([pacmod3_msgs/LatLonHeadingRpt](https://github.com/astuff/pacmod3_msgs/blob/master/msg/LatLonHeadingRpt.msg))
- `yaw_rate_rpt` ([pacmod3_msgs/YawRateRpt](https://github.com/astuff/pacmod3_msgs/blob/master/msg/YawRateRpt.msg))
- `cruise_control_buttons_rpt` ([pacmod3_msgs/SystemRptInt](https://github.com/astuff/pacmod3_msgs/blob/master/msg/SystemRptInt.msg))
- `engine_brake_rpt` ([pacmod3_msgs/SystemRptInt](https://github.com/astuff/pacmod3_msgs/blob/master/msg/SystemRptInt.msg))
- `marker_lamp_rpt` ([pacmod3_msgs/SystemRptBool](https://github.com/astuff/pacmod3_msgs/blob/master/msg/SystemRptBool.msg))
- `sprayer_rpt` ([pacmod3_msgs/SystemRptBool](https://github.com/astuff/pacmod3_msgs/blob/master/msg/SystemRptBool.msg))
- `detected_object_rpt` ([pacmod3_msgs/DetectedObjectRpt](https://github.com/astuff/pacmod3_msgs/blob/master/msg/DetectedObjectRpt.msg))
- `vehicle_dynamics_rpt` ([pacmod3_msgs/VehicleDynamicsRpt](https://github.com/astuff/pacmod3_msgs/blob/master/msg/VehicleDynamicsRpt.msg))

### Subscribed Topics

Topics subscribed on all platforms:

- `accel_cmd` ([pacmod3_msgs/SystemCmdFloat](https://github.com/astuff/pacmod3_msgs/blob/master/msg/SystemCmdFloat.msg))
- `brake_cmd` ([pacmod3_msgs/SystemCmdFloat](https://github.com/astuff/pacmod3_msgs/blob/master/msg/SystemCmdFloat.msg))
- `steering_cmd` ([pacmod3_msgs/SystemCmdFloat](https://github.com/astuff/pacmod3_msgs/blob/master/msg/SystemCmdFloat.msg))
- `shift_cmd` ([pacmod3_msgs/SystemCmdInt](https://github.com/astuff/pacmod3_msgs/blob/master/msg/SystemCmdInt.msg))
- `turn_cmd` ([pacmod3_msgs/SystemCmdInt](https://github.com/astuff/pacmod3_msgs/blob/master/msg/SystemCmdInt.msg))

Topics subscribed on supported platforms only:

- `wiper_cmd` ([pacmod3_msgs/SystemCmdInt](https://github.com/astuff/pacmod3_msgs/blob/master/msg/SystemCmdInt.msg))
- `headlight_cmd` ([pacmod3_msgs/SystemCmdInt](https://github.com/astuff/pacmod3_msgs/blob/master/msg/SystemCmdInt.msg))
- `horn_cmd` ([pacmod3_msgs/SystemCmdBool](https://github.com/astuff/pacmod3_msgs/blob/master/msg/SystemCmdBool.msg))
- `hazard_lights_cmd` ([pacmod3_msgs/SystemCmdBool](https://github.com/astuff/pacmod3_msgs/blob/master/msg/SystemCmdBool.msg))
- `cruise_control_buttons_cmd` ([pacmod3_msgs/SystemCmdInt](https://github.com/astuff/pacmod3_msgs/blob/master/msg/SystemCmdInt.msg))
- `engine_brake_cmd` ([pacmod3_msgs/SystemCmdInt](https://github.com/astuff/pacmod3_msgs/blob/master/msg/SystemCmdInt.msg))
- `marker_lamp_cmd` ([pacmod3_msgs/SystemCmdBool](https://github.com/astuff/pacmod3_msgs/blob/master/msg/SystemCmdBool.msg))
- `sprayer_cmd` ([pacmod3_msgs/SystemCmdBool](https://github.com/astuff/pacmod3_msgs/blob/master/msg/SystemCmdBool.msg))
- `rear_pass_door_cmd` ([pacmod3_msgs/SystemCmdInt](https://github.com/astuff/pacmod3_msgs/blob/master/msg/SystemCmdInt.msg))

## Supported Vehicles ##

At the moment, this ROS2 pacmod3 driver only supports vehicles that use DBC version 3.
Vehicles using DBC verion 3 would be Lexus RX-450h vehicles purchased from AutonomouStuff in 2020 or earlier, as well as any PACMod-supported vehicle purchased from AutonomouStuff in 2018 or earlier.

If you are unsure about which DBC version your existing PACMod supports, please reach out to our support team: https://autonomoustuff.com/support
