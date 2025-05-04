cmake_minimum_required(VERSION 2.8.3)
project(path_searching2d)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  plan_env2d
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES path_searching2d
 CATKIN_DEPENDS plan_env2d
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

add_library( path_searching2d
    src/dyn_a_star.cpp
    )
target_link_libraries( path_searching2d
    ${catkin_LIBRARIES} 
    )  
