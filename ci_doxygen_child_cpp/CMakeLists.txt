cmake_minimum_required(VERSION 2.8.3)
project(ci_doxygen_child_cpp) 

# this project is there to show how to create doxygen documentation
# which refers to code that is in another project in our workspace,
# in this case ci_example_cpp

# required to use std::shared_ptr in code
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")


find_package(catkin REQUIRED COMPONENTS
  ci_example_cpp
)

include_directories( include ${catkin_INCLUDE_DIRS}  )

add_library(doxygen_child SHARED src/doxygen_child.cpp)
target_link_libraries(doxygen_child ${catkin_LIBRARIES} )

catkin_package(
 LIBRARIES doxygen_child
 INCLUDE_DIRS include 
 CATKIN_DEPENDS ci_example_cpp
)

# building the documentation
# the documentation should not be build by default, but only 
# if the BUILD_DOCUMENTATION option is used, i.e. 'catkin_make -DBUILD_DOCUMENTATION=ON'
# (the continuous integration job will generate the doc this way)
##########################
# building documentation #
##########################
build_doxygen_documentation()