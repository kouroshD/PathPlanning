cmake_minimum_required(VERSION 2.8.3)
project(rrtstar_point)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  message_generation
  std_msgs
  rrtstar_msgs_point

)

#Is for add the .o files to devel/lib

catkin_package(  CATKIN_DEPENDS message_generation roscpp rospy std_msgs rrtstar_msgs_point geometry_msgs )

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#ADD_LIBRARY(endor /home/nasa/Documents/Endor/endor-master/aograph.cpp)

## Declare a C++ executable
# name  name.cpp
add_executable(rrtstar_point src/rrts_main.cpp src/system_single_integrator.cpp src/kdtree.c)

add_dependencies(rrtstar_point rrtstar_msgs_point_generate_messages_cpp geometry_msgs_generate_messages_cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(rrtstar_point
   ${catkin_LIBRARIES}
 )

