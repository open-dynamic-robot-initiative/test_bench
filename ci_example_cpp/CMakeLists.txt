cmake_minimum_required(VERSION 2.8.3)
project(ci_example_cpp) 

# required to use std::shared_ptr in code
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


# these catkin packages must be in the catkin workspace for 
# this package to compile. roscpp, rostest and std_msgs are standart ros package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rostest
  std_msgs
  mpi_cmake_modules
  pybind11_catkin
)

# Find some none-catkin dependencies
add_required_dependency("yaml-cpp >= 0.5")

# the find_package command created the ${catkin_INCLUDE_DIRS} cmake variable
# it contains the path to all header folders exported by the package found
# by adding it to include_directories, the source of these package will have access
# to the header files of all these packages.
# in turn, this package exports its own include header folder that other catkin project
# may find. See the catkin_package command somewhere below.
include_directories( include ${catkin_INCLUDE_DIRS}  )

# creating the library and linking it against the libraries this project depends on
# the ${catkin_LIBRARIES} is a cmake variable that was created by the find_package command
# above. It contains all the libraries exported by the package found.
# in turn, this package exports the library it created so that other catkin project
# may link against it. See the catkin_package command somewhere below.
add_library( basic_pid SHARED
  src/gains_configuration.cpp
  src/default_configuration.cpp
  src/file_configuration.cpp
  src/rosparameters_configuration.cpp
  src/pid.cpp )
target_link_libraries(basic_pid ${catkin_LIBRARIES} )

# this package exports its include folder and its libraries for other catkin packages to enjoy
catkin_package(
 LIBRARIES basic_pid # exporting the basic_pid library (libs and executable built in other catkin package will be able to link against them)
 INCLUDE_DIRS include # exporting the include file: source file hosted by other catkin package will be able to include the related header files)
 CATKIN_DEPENDS roscpp std_msgs 
)


# getting the path to the config folder and storing it in the CONFIG_PATH variable
# will be useful to set path to config files into the sources of demos and tests
get_filename_component(CONFIG_PATH config ABSOLUTE)

####################################################################################
#                       example of unit tests                                      #
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
# Use the none-catkin dependencies
pkg_config_use_dependency(ci_example_ut yaml-cpp)
# one of the test requires to read a file in the config folder.
# here we ask the preprocessor to replace in the source TEST_YAML_FILE_PATH to 
# the absolute path to the file test_yaml_file.yaml in the catkin package config folder
set_target_properties(ci_example_ut PROPERTIES 
  COMPILE_DEFINITIONS TEST_PID_GAINS_YAML_FILE_PATH="${CONFIG_PATH}/test_pid_gains.yaml")

####################################################################################
#               example of unit tests that use ROS (c++)                           #
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
# Use the none-catkin dependencies
pkg_config_use_dependency(ci_example_rostest yaml-cpp)

####################################################################################
#                 how to name your demo executables ?                              #
####################################################################################

# Please note that all the demos that are to be executed by the continuous integration script have a name that:
# - starts with the name of the current catkin package (ci_example_...)
# - ends with "_demo".
# This is *required*

####################################################################################
#         examples of executables              
####################################################################################

### simple example ###
add_executable(ci_example_demo src/demos/ci_example_demo.cpp)
target_link_libraries(ci_example_demo basic_pid)
# Use the none-catkin dependencies
pkg_config_use_dependency(ci_example_demo yaml-cpp)

### requires path to a config file ###
# This demo requires to read a file in the config folder
# here we ask the preprocessor to replace in the source TEST_PID_GAINS_YAML_FILE_PATH to 
# the absolute path to the file test_yaml_file.yaml in the catkin package config folder
add_executable(ci_example_abs_path_config_demo src/demos/ci_example_abs_path_config_demo.cpp)
target_link_libraries(ci_example_abs_path_config_demo basic_pid)
# Use the none-catkin dependencies
pkg_config_use_dependency(ci_example_abs_path_config_demo yaml-cpp)
set_target_properties(ci_example_abs_path_config_demo PROPERTIES 
  COMPILE_DEFINITIONS TEST_PID_GAINS_YAML_FILE_PATH="${CONFIG_PATH}/test_pid_gains.yaml")

##########################
# building documentation #
##########################
build_doxygen_documentation()

# typically, a simple way to create documentation is to 
# have a Doxygen file, and type "doxygen" in a terminal
# but in order to manage dependencies between packages, we do something more involved
# for example the documentation of the package ci_doxygen_child (in same git repo) 
# will refer to code built in this ci_example_cpp project
# see ci_doxygen_child for more info


################################
# python bindings with pybind11#
################################
#pybind11_add_module(ci_example src/python_bindings.cpp)
#target_link_libraries(ci_example PRIVATE basic_pid ${catkin_LIBRARIES})
#set_target_properties(ci_example PROPERTIES
#  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION})
#install(TARGETS ci_example DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})
