cmake_minimum_required(VERSION 3.10.2)
project(ci_example) 

# Using C++17
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


################
# Dependencies #
################

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(yaml_utils REQUIRED)
find_package(mpi_cmake_modules REQUIRED)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)
# ??? only not ament-cmake dependencies ???
ament_export_dependencies(mpi_cmake_modules yaml_utils pybind11)


# prepare to export all needed targets
set(all_targets)
set(all_target_exports)


###########
# Library #
###########

add_library( ${PROJECT_NAME} SHARED
  src/gains_configuration.cpp
  src/default_configuration.cpp
  src/file_configuration.cpp
  #src/rosparameters_configuration.cpp
  src/pid.cpp )
# ??? why so complicated ???
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# ??? shared_memory uses target_link_libraries againt Threads::Threads ???
ament_target_dependencies(${PROJECT_NAME} yaml_utils)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
list(APPEND all_targets ${PROJECT_NAME})
list(APPEND all_target_exports export_${PROJECT_NAME})


###################
# Python wrappers #
###################

add_library(${PROJECT_NAME}_cpp_bindings MODULE srcpy/wrappers.cpp)
# ??? trying this instead of target_link_library, but no success ???
#ament_target_dependencies(${PROJECT_NAME}
#  pybind11_vendor)
target_link_libraries(${PROJECT_NAME}_cpp_bindings PRIVATE pybind11::module)
target_link_libraries(${PROJECT_NAME}_cpp_bindings PRIVATE ${PYTHON_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_cpp_bindings PRIVATE ${PROJECT_NAME})
# ??? 
set_target_properties(${PROJECT_NAME}_cpp_bindings
                      PROPERTIES PREFIX "" SUFFIX "${PYTHON_MODULE_EXTENSION}")
target_include_directories(
  ${PROJECT_NAME}_cpp_bindings
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include> SYSTEM
  PUBLIC ${PYTHON_INCLUDE_DIRS})
_ament_cmake_python_get_python_install_dir()
install(TARGETS ${PROJECT_NAME}_cpp_bindings DESTINATION ${PYTHON_INSTALL_DIR})


######################
# Python Native code #
######################

# ??? does not work ???
# also: is the setup.py still required ?
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python/${PROJECT_NAME})


#################
# documentation #
#################

add_documentation()


######################
# Install and export #
######################\

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS ${all_targets}
  EXPORT ${all_target_exports}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)
ament_package()


##################
# cpp unit tests #
##################

if(BUILD_TESTING)
  find_package(ament_cmake_gtest)
  ament_add_gtest(test_${PROJECT_NAME} tests/main.cpp tests/unit_tests.cpp)
  # ??? why not ament_target_dependencies ???
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
  set_target_properties(test_${PROJECT_NAME} PROPERTIES 
    COMPILE_DEFINITIONS
    TEST_PID_GAINS_YAML_FILE_PATH="${CONFIG_PATH}/test_pid_gains.yaml")
endif()

      
#catkin_python_setup()

# this package exports its include folder and its libraries for other catkin packages to enjoy
#catkin_package(
# LIBRARIES ci_example # exporting the ci_example library (libs and executable built in other catkin package will be able to link against them)
# INCLUDE_DIRS include # exporting the include file: source file hosted by other catkin package will be able to include the related header files)
# CATKIN_DEPENDS #roscpp std_msgs 
#)

# getting the path to the config folder and storing it in the CONFIG_PATH variable
# will be useful to set path to config files into the sources of demos and tests
#get_filename_component(CONFIG_PATH config ABSOLUTE)


####################################################################################
#                       example of unit tests                                      #
####################################################################################

# to compile and run:
# after calling catkin_make, source ./devel/setup.bash
# then after typing "catkin_make run_", you may press "tab" for autocompletion
# selecting "catkin_make run_tests" will results in all found tests to be compiled
# and run, but you may select specific test

#catkin_add_gtest(ci_example_ut
#  tests/main.cpp
#  tests/ci_example_ut.cpp
#)
#target_link_libraries(ci_example_ut ci_example)
# Use the none-catkin dependencies
#pkg_config_use_dependency(ci_example_ut yaml-cpp)
# one of the test requires to read a file in the config folder.
# here we ask the preprocessor to replace in the source TEST_YAML_FILE_PATH to 
# the absolute path to the file test_yaml_file.yaml in the catkin package config folder
#set_target_properties(ci_example_ut PROPERTIES 
#  COMPILE_DEFINITIONS TEST_PID_GAINS_YAML_FILE_PATH="${CONFIG_PATH}/test_pid_gains.yaml")


# regular unit tests (test is the name of the folder in which all the tests are)
#catkin_add_nosetests(tests)


# test that requires roscore to run:
# tests are started via *.test files which are ros launch files
# consequently, these tests may run under the assumption roscore is running
#find_package(rostest REQUIRED)
#add_rostest(tests/test_pid_ros.test)


####################################################################################
#               example of unit tests that use ROS (c++)                           #
####################################################################################

# unit test in ci_example_rostest.cpp needs the rosparameter server to run
# one just need to associate this test with a ros launch file
# (here ci_example_rostest.test)
# tests are run using same method as for regular unit-tests, as explained above

#find_package(rostest REQUIRED)
#add_rostest_gtest(ci_example_rostest
#  tests/ci_example_rostest.test # launch file that will be called before starting the test
#  tests/ci_example_rostest.cpp # source code of test that uses ros parameter server
#  tests/main.cpp )
#target_link_libraries(ci_example_rostest
#  ${catkin_LIBRARIES}  ci_example)
# Use the none-catkin dependencies
#pkg_config_use_dependency(ci_example_rostest yaml-cpp)

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

# executable will be found in <workspace>/devel/lib/ci_example_cpp/ after
# calling catkin_make

### simple example ###
#add_executable(demo_pid demos/demo_pid.cpp)
#target_link_libraries(demo_pid ci_example)
# Use the none-catkin dependencies
#pkg_config_use_dependency(demo_pid yaml-cpp)

### requires path to a config file ###
# This demo requires to read a file in the config folder
# here we ask the preprocessor to replace in the source TEST_PID_GAINS_YAML_FILE_PATH to 
# the absolute path to the file test_yaml_file.yaml in the catkin package config folder
#add_executable(demo_pid_load_from_file demos/demo_pid_load_from_file.cpp)
#target_link_libraries(demo_pid_load_from_file ci_example)
# Use the none-catkin dependencies
#pkg_config_use_dependency(demo_pid_load_from_file yaml-cpp)
#set_target_properties(demo_pid_load_from_file PROPERTIES 
#  COMPILE_DEFINITIONS TEST_PID_GAINS_YAML_FILE_PATH="${CONFIG_PATH}/test_pid_gains.yaml")


##########################
# building documentation #
##########################
#build_doxygen_documentation()

# typically, a simple way to create documentation is to 
# have a Doxygen file, and type "doxygen" in a terminal
# but in order to manage dependencies between packages, we do something more involved
# compile with : catkin_make -DBUILD_DOCUMENTATION=ON,
# the documentation will be created in <workspace>/devel/share/ci_example_cpp/doc/


################################
# python bindings with pybind11#
################################
#pybind11_add_module(basic_pid srcpy/python_bindings.cpp)
#target_link_libraries(basic_pid PRIVATE ci_example ${catkin_LIBRARIES})
#pkg_config_use_dependency(basic_pid yaml-cpp)
#set_target_properties(basic_pid PROPERTIES
#  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION})
#install(TARGETS ci_example DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})

# this will create a module of name "basic_pid". After calling catkin_make
# and sourcing <workspace>/devel/setup.bash, the following should work in python:

# import basic_pid
# p = basic_pid.PID()
# p.compute(1,1,1,1)
# p.reset_integral()
