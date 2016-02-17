cmake_minimum_required(VERSION 2.8.3)
project(ci_example) 

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  YAML_CPP
)

catkin_package(
 LIBRARIES gains_configuration
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp std_msgs YAML_CPP 
)

include_directories( include ${catkin_INCLUDE_DIRS}  )

add_library(gains_configuration SHARED src/gains_configuration.cpp)
target_link_libraries(gains_configuration ${catkin_LIBRARIES} )

