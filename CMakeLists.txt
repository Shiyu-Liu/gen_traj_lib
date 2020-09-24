cmake_minimum_required(VERSION 3.0.2)
project(gen_traj_lib)
add_compile_options(-std=c++11)

#set(CMAKE_BUILD_TYPE Release)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslib
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs roslib
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/trajectory_base.cpp
  src/trajectory_3dspace.cpp
  src/trajectory_nd.cpp
)

## Unit testing via G-Test
if(CATKIN_ENABLE_TESTING)
  find_package(${PROJECT_NAME} REQUIRED)
  catkin_add_gtest(TestTrajectory test/src/test_trajectory_3d.cpp)
  target_link_libraries(TestTrajectory ${PROJECT_NAME} ${catkin_LIBRARIES} pthread)
endif(CATKIN_ENABLE_TESTING)