cmake_minimum_required(VERSION 3.5)
project(pacmod3)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(can_msgs REQUIRED)
find_package(pacmod3_msgs REQUIRED)
find_package(std_msgs REQUIRED)

###########
## Build ##
###########

include_directories(
  include/
  pacmod3_common/
)

# TODO: Change this to library for pacmod3_common
add_library(pacmod3_common SHARED
  pacmod3_common/pacmod3_dbc_ros_api.cpp
  pacmod3_common/pacmod3_dbc3_ros_api.cpp
  pacmod3_common/pacmod3_dbc4_ros_api.cpp
  pacmod3_common/autogen/pacmod3.c
  pacmod3_common/autogen/pacmod4.c
)
ament_target_dependencies(
  pacmod3_common
  "rclcpp"
  # "rclcpp_components"
  # "rclcpp_lifecycle"
  "can_msgs"
  "pacmod3_msgs"
  # "std_msgs"
)

add_library(${PROJECT_NAME}_node SHARED
  src/pacmod3_ros_msg_handler.cpp
  # pacmod3_common/pacmod3_dbc_ros_api.cpp
  # pacmod3_common/pacmod3_dbc3_ros_api.cpp
  # pacmod3_common/pacmod3_dbc4_ros_api.cpp
  src/pacmod3_core.cpp
  src/pacmod3_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  pacmod3_common
)

# TODO: build pacmod3_common librarby with USE_ROS1/USE_ROS2 set appropriately

target_compile_definitions(
  ${PROJECT_NAME}_node
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(
  ${PROJECT_NAME}_node
  "rclcpp"
  "rclcpp_components"
  "rclcpp_lifecycle"
  "can_msgs"
  "pacmod3_msgs"
  "std_msgs"
)
rclcpp_components_register_nodes(
  ${PROJECT_NAME}_node
  "pacmod3::PACMod3Node"
)

add_executable(
  ${PROJECT_NAME}_driver
  src/pacmod3_driver.cpp
)
ament_target_dependencies(
  ${PROJECT_NAME}_driver
  "rclcpp"
  "lifecycle_msgs"
)
target_link_libraries(
  ${PROJECT_NAME}_driver
  ${PROJECT_NAME}_node
)

ament_export_include_directories(include)
ament_export_interfaces(export_${PROJECT_NAME}_node HAS_LIBRARY_TARGET)
# ament_export_targets(
ament_export_libraries(${PROJECT_NAME}_node)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}_node pacmod3_common
  EXPORT export_${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS ${PROJECT_NAME}_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
