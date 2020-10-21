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
  src/pid.cpp )
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} yaml_utils)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
list(APPEND all_targets ${PROJECT_NAME})
list(APPEND all_target_exports export_${PROJECT_NAME})


###################
# Python wrappers #
###################

add_library(${PROJECT_NAME}_cpp_bindings MODULE srcpy/wrappers.cpp)
target_link_libraries(${PROJECT_NAME}_cpp_bindings PRIVATE pybind11::module)
target_link_libraries(${PROJECT_NAME}_cpp_bindings PRIVATE ${PYTHON_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_cpp_bindings PRIVATE ${PROJECT_NAME})
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
  ament_add_gtest(test_${PROJECT_NAME}_cpp tests/main.cpp tests/unit_tests.cpp)
  target_link_libraries(test_${PROJECT_NAME}_cpp ${PROJECT_NAME})
  set_target_properties(test_${PROJECT_NAME}_cpp PROPERTIES 
    COMPILE_DEFINITIONS
    TEST_PID_GAINS_YAML_FILE_PATH="${CONFIG_PATH}/test_pid_gains.yaml")

  #find_package(ament_cmake_nose)
  #ament_add_nose_test(test_${PROJECT_NAME}_python tests/test_pid.py)
  
endif()

      
