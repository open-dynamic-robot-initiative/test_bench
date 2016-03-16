cmake_minimum_required(VERSION 2.8.3)
project(ci_example) 

# requred to use std::shared_ptr in code
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")


####################################################################################
#                 ensuring path to libraries are set during install                #
####################################################################################
# if some demo are required to be run from the install folder
# (i.e. the folder created when "catkin_make install" is called)
# the commands below are required to make sure the binaries may still link against
# their required libraries after install 
# note: has to be called before add_executable
# see somewhere below for demo requiring installation
set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)                                                
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rostest
  std_msgs
  YAML_CPP
)

####################################################################################
#                           dealing with CPP                                       #
####################################################################################

catkin_package(
 LIBRARIES gains_configuration # exporting the gains_configuration library (libs and executable built in other catkin package will be able to link against them)
 INCLUDE_DIRS include # exporting the include file: source file hosted by other catkin package will be able to include the related header files)
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
#                         dealing with Python                                      #
####################################################################################

# more information can be found here: http://docs.ros.org/jade/api/catkin/html/user_guide/setup_dot_py.html
# and setup.py in same folder as this file
# what this does: after calling catkin_make and sourcing devel/setup.bash, ci_example.pid will be in python path
catkin_python_setup()

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
  COMPILE_DEFINITIONS TEST_PID_GAINS_YAML_FILE_PATH="${CONFIG_PATH}/test_pid_gains.yaml")

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
#                 how to name your demo executables ?                              #
####################################################################################

# Please note that all the demos that are to be executed by the continuous integration script have a name that:
# - starts with the name of the current catkin package (ci_example_...)
# - ends with "_demo".
# This is *required*

####################################################################################
#                           example of python demo                                 #
####################################################################################

# the python script is set as executable, named *_demo (in this case ci_example_python_demo),
# placed in demos and copies to install/demo folder when 'catkin_make install' is called 
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/demos/ci_example_ros_demo" DESTINATION ${CMAKE_INSTALL_PREFIX}/demos )

####################################################################################
#            example of demo supported by continuous integration                   #
####################################################################################

### simple example ###
# any executable which names ends with "_demo" or "_example" will be run
# automatically by the continuous integration script. Nothing special required
add_executable(ci_example_demo demos/ci_example_demo.cpp)
target_link_libraries(ci_example_demo basic_pid)

### requires path to a config file ###
# This demo requires to read a file in the config folder
# here we ask the preprocessor to replace in the source TEST_PID_GAINS_YAML_FILE_PATH to 
# the absolute path to the file test_yaml_file.yaml in the catkin package config folder
add_executable(ci_example_abs_path_config_demo demos/ci_example_abs_path_config_demo.cpp)
target_link_libraries(ci_example_abs_path_config_demo basic_pid)
set_target_properties(ci_example_abs_path_config_demo PROPERTIES 
  COMPILE_DEFINITIONS TEST_PID_GAINS_YAML_FILE_PATH="${CONFIG_PATH}/test_pid_gains.yaml")

### requires a complex setup ###
# if a demo requires a special setup, then the executable is not start directly,
# but start via a script that performs the setup/cleaning.
# in this case ci_example_ros_demo script starts ros, place info in the parameter
# server, call the demo and kill roscore
add_executable(ci_example_ros demos/ci_example_ros.cpp)
target_link_libraries(ci_example_ros basic_pid)
# the sh script and the executable have to find themselves in the same folder.
# for this, we configure installation, which will occurs when "catkin_make install" is called
# then both the script and the executable binaries will find themselves in ./install/demos
install(TARGETS ci_example_ros basic_pid
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/demos 
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
) 
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/demos/ci_example_python_ros.py" DESTINATION ${CMAKE_INSTALL_PREFIX}/demos )