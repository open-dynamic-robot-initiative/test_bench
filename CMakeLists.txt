cmake_minimum_required(VERSION 2.8.3)
project(ci_example) 

# requred to use std::shared_ptr in code
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rostest
  std_msgs
  YAML_CPP
)

catkin_package(
 LIBRARIES gains_configuration
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp std_msgs YAML_CPP 
)

include_directories( include ${catkin_INCLUDE_DIRS}  )

# getting the path to the config folder and storing it in the CONFIG_PATH variable
# will be useful to set path to config files into the sources of demos and tests
get_filename_component(CONFIG_PATH config ABSOLUTE)


# creating the library 
add_library(basic_pid SHARED src/basic_pid.cpp)
target_link_libraries(basic_pid ${catkin_LIBRARIES} )


####################################################################################
#                         example of unit tests                                    #
####################################################################################

# to compile and run:
# after calling catkin_make, source ./devel/setup.bash
# then after typing "catkin_make run_", you may press "tab" for autocompletion
# selecting "catkin_make run_tests" will results in all found tests to be compiled
# and run, but you may select specific test

catkin_add_gtest(ci_example_ut
  tests/main.cpp
  tests/ci_example_ut.cpp
)
target_link_libraries(ci_example_ut basic_pid)
# one of the test requires to read a file in the config folder.
# here we ask the preprocessor to replace in the source TEST_YAML_FILE_PATH to 
# the absolute path to the file test_yaml_file.yaml in the catkin package config folder
set_target_properties(ci_example_ut PROPERTIES 
  COMPILE_DEFINITIONS TEST_YAML_FILE_PATH="${CONFIG_PATH}/test_pid_gains.yaml")


####################################################################################
#                 example of unit tests that use ROS                               #
####################################################################################

# unit test in ci_example_rostest.cpp needs the rosparameter server to run
# one just need to associate this test with a ros launch file
# (here ci_example_rostest.test)
# tests are run using same method as for regular unit-tests, as explained above

find_package(rostest REQUIRED)
add_rostest_gtest(ci_example_rostest
  tests/ci_example_rostest.test # launch file that will be called before starting the test
  tests/ci_example_rostest.cpp # source code of test that uses ros parameter server
  tests/main.cpp )
target_link_libraries(ci_example_rostest
  ${catkin_LIBRARIES}  basic_pid)

####################################################################################
#            example of demo supported by continuous integration                   #
####################################################################################

# any executable which names ends with "_demo" or "_example" will be run
# automatically by the continuous integration script. Nothing special required
add_executable(ci_example_demo demos/ci_example_demo.cpp)
target_link_libraries(ci_example_demo basic_pid)

# This demo requires to read a file in the config folder
# here we ask the preprocessor to replace in the source TEST_YAML_FILE_PATH to 
# the absolute path to the file test_yaml_file.yaml in the catkin package config folder
add_executable(ci_example_abs_path_config_demo demos/ci_example_abs_path_config_demo.cpp)
target_link_libraries(ci_example_abs_path_config_demo basic_pid)
set_target_properties(ci_example_abs_path_config_demo PROPERTIES 
  COMPILE_DEFINITIONS TEST_YAML_FILE_PATH="${CONFIG_PATH}/test_pid_gains.yaml")



