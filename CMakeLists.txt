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
  pacmod3_common
  ${catkin_INCLUDE_DIRS}
)

# nodelets
add_library(${PROJECT_NAME}_nodelet
  src/pacmod3_nodelet.cpp
  src/pacmod3_core.cpp
  src/pacmod3_ros_msg_handler.cpp
  # src/pacmod3dbc12_ros_msg_handler.cpp
  pacmod3_common/pacmod3_dbc12_ros_api.cpp
)

target_link_libraries(${PROJECT_NAME}_nodelet
  ${catkin_LIBRARIES}
)

# nodes
add_executable(${PROJECT_NAME}_node
  src/pacmod3_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

set(ROSLINT_CPP_OPTS "--filter=-build/c++11")
roslint_cpp()

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
