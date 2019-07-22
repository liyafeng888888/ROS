cmake_minimum_required(VERSION 2.8.3)
project(pathgo_imu)

set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  serial
  std_msgs
  std_srvs
  tf
)

find_package(Boost REQUIRED COMPONENTS
    thread)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pathgo_imu
CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs serial std_msgs std_srvs tf
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(sync_imu_node src/sync_imu_node.cpp)


target_link_libraries(sync_imu_node 
   ${catkin_LIBRARIES}
)


