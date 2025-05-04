cmake_minimum_required(VERSION 2.8.3)
project(plan_env2d)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
  message_filters
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES plan_env2d
 CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

if(NOT Eigen3_INCLUDE_DIRS)
    set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
endif()

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_library( plan_env2d
    src/grid_map.cpp 
    src/raycast.cpp
    )

target_include_directories(plan_env2d INTERFACE ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})
target_link_libraries( plan_env2d 
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    )  
