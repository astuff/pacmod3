cmake_minimum_required(VERSION 2.8.3)
project(pacmod3)

add_definitions(-std=c++14)

set(catkin_deps
  nodelet
  roscpp
  std_msgs
  pacmod3_msgs
  can_msgs
)

find_package(catkin REQUIRED ${catkin_deps} roslint)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${catkin_deps}
  LIBRARIES ${PROJECT_NAME}_nodelet
)

###########
## Build ##
###########

include_directories(
  include
  pacmod3_common/include
  ${catkin_INCLUDE_DIRS}
)

# Common Library
add_compile_definitions(ROS_VERSION=$ENV{ROS_VERSION})
add_library(pacmod3_common
  pacmod3_common/src/pacmod3_dbc_ros_api.cpp
  pacmod3_common/src/pacmod3_dbc3_ros_api.cpp
  pacmod3_common/src/pacmod3_dbc4_ros_api.cpp
  pacmod3_common/src/pacmod3_dbc5_ros_api.cpp
  pacmod3_common/src/pacmod3_dbc6_ros_api.cpp
  pacmod3_common/src/pacmod3_dbc7_ros_api.cpp
  pacmod3_common/src/pacmod3_dbc8_ros_api.cpp
  pacmod3_common/src/pacmod3_dbc9_ros_api.cpp
  pacmod3_common/src/pacmod3_dbc10_ros_api.cpp
  pacmod3_common/src/pacmod3_dbc11_ros_api.cpp
  pacmod3_common/src/pacmod3_dbc12_ros_api.cpp
  pacmod3_common/src/autogen/pacmod3.c
  pacmod3_common/src/autogen/pacmod4.c
  pacmod3_common/src/autogen/pacmod5.c
  pacmod3_common/src/autogen/pacmod6.c
  pacmod3_common/src/autogen/pacmod7.c
  pacmod3_common/src/autogen/pacmod8.c
  pacmod3_common/src/autogen/pacmod9.c
  pacmod3_common/src/autogen/pacmod10.c
  pacmod3_common/src/autogen/pacmod11.c
  pacmod3_common/src/autogen/pacmod12.c
)

# nodelets
add_library(${PROJECT_NAME}_nodelet
  src/pacmod3_nodelet.cpp
  src/pacmod3_ros_msg_handler.cpp
)

target_link_libraries(${PROJECT_NAME}_nodelet
  pacmod3_common
  ${catkin_LIBRARIES}
)

# nodes
add_executable(${PROJECT_NAME}_node
  src/pacmod3_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

# TODO: Re-enable after auto-formatting
# set(ROSLINT_CPP_OPTS "--filter=-build/include_subdir")
# roslint_cpp(
#   src/pacmod3_node.cpp
#   src/pacmod3_nodelet.cpp
#   src/pacmod3_ros_msg_handler.cpp
#   include/pacmod3/pacmod3_nodelet.h
#   include/pacmod3/pacmod3_ros_msg_handler.h
#   )

install(TARGETS
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
