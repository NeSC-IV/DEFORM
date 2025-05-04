cmake_minimum_required(VERSION 3.0.2)
project(traj_opti)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(Eigen3 REQUIRED)
find_package(casadi REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES traj_opti
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS casadi
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(traj_opti
  src/traj_opti.cpp
)

target_link_libraries(traj_opti
  ${catkin_LIBRARIES}
  casadi
)

