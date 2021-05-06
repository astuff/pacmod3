cmake_minimum_required(VERSION 2.8.3)
project(pacmod3)

add_definitions(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roslint
  roscpp
  std_msgs
  can_msgs
  message_generation
)

add_message_files(DIRECTORY msg FILES
  "AccelAuxRpt.msg"
  "AllSystemStatuses.msg"
  "AngVelRpt.msg"
  "BrakeAuxRpt.msg"
  "BrakeDecelAuxRpt.msg"
  "BrakeDecelCmd.msg"
  "CabinClimateCmd.msg"
  "CabinClimateRpt.msg"
  "ComponentRpt.msg"
  "DateTimeRpt.msg"
  "DetectedObjectRpt.msg"
  "DoorRpt.msg"
  "DriveTrainRpt.msg"
  "EngineRpt.msg"
  "EStopRpt.msg"
  "GlobalCmd.msg"
  "GlobalRpt.msg"
  "GlobalRpt2.msg"
  "HeadlightAuxRpt.msg"
  "InteriorLightsRpt.msg"
  "KeyValuePair.msg"
  "LatLonHeadingRpt.msg"
  "LinearAccelRpt.msg"
  "MotorRpt1.msg"
  "MotorRpt2.msg"
  "MotorRpt3.msg"
  "NotificationCmd.msg"
  "OccupancyRpt.msg"
  "OverrideCfgRpt.msg"
  "PacmodCmd.msg"
  "ParkingBrakeAuxRpt.msg"
  "PositionWithSpeed.msg"
  "RearLightsRpt.msg"
  "SafetyBrakeCmd.msg"
  "SafetyBrakeRpt.msg"
  "SafetyFuncCmd.msg"
  "SafetyFuncCriticalStopRpt.msg"
  "SafetyFuncRpt.msg"
  "ShiftAuxRpt.msg"
  "SoftwareVersionRpt.msg"
  "SteerAuxRpt.msg"
  "SteerSystemCmd.msg"
  "SteerCmdLimitRpt.msg"
  "SupervisoryCtrl.msg"
  "SystemCmdBool.msg"
  "SystemCmdFloat.msg"
  "SystemCmdInt.msg"
  "SystemCmdLimitRpt.msg"
  "SystemRptBool.msg"
  "SystemRptFloat.msg"
  "SystemRptInt.msg"
  "TirePressureRpt.msg"
  "TurnAuxRpt.msg"
  "VehDynamicsRpt.msg"
  "VehicleFaultRpt.msg"
  "VehicleSpeedRpt.msg"
  "VinRpt.msg"
  "VinRpt2.msg"
  "WatchdogRpt.msg"
  "WatchdogRpt2.msg"
  "WheelSpeedRpt.msg"
  "WiperAuxRpt.msg"
  "YawRateRpt.msg"
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS
  roscpp
  std_msgs
  can_msgs
  message_runtime
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}
  src/pacmod3_ros_msg_handler.cpp
  src/pacmod3_node.cpp
  src/pacmod3_core.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

set(ROSLINT_CPP_OPTS "--filter=-build/c++11")
roslint_cpp()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY msg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg
)
