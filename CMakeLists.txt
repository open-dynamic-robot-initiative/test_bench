cmake_minimum_required(VERSION 3.10.2)
project(test_bench) 

# Using C++17
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

#
# Dependencies
#

find_package(ament_cmake REQUIRED)
find_package(mpi_cmake_modules REQUIRED)
find_package(real_time_tools REQUIRED)
find_package(mim_msgs REQUIRED)
find_package(master_board_sdk REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(dynamic_graph_manager QUIET)

#
# prepare to export all needed targets
#
ament_export_dependencies(ament_cmake mpi_cmake_modules yaml_utils master_board_sdk)
ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
set(all_targets)

#
# Library.
#

add_library( ${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}.cpp)
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} real_time_tools::real_time_tools)
target_link_libraries(${PROJECT_NAME} master_board_sdk::master_board_sdk)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
list(APPEND all_targets ${PROJECT_NAME})

if(${dynamic_graph_manager_FOUND})
  find_package(ament_index_cpp REQUIRED)
  find_package(yaml_utils REQUIRED)

  add_library(dgm_${PROJECT_NAME} SHARED src/dgm_${PROJECT_NAME}.cpp)
  target_include_directories(
    dgm_${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(dgm_${PROJECT_NAME} ${PROJECT_NAME})
  target_link_libraries(dgm_${PROJECT_NAME} dynamic_graph_manager::dynamic_graph_manager)
  target_link_libraries(dgm_${PROJECT_NAME} yaml_utils::yaml_utils)
    list(APPEND all_targets dgm_${PROJECT_NAME})

  add_executable(dg_main_${PROJECT_NAME} src/dg_main_${PROJECT_NAME}.cpp)
  target_include_directories(
    dg_main_${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(dg_main_${PROJECT_NAME} dgm_${PROJECT_NAME})
  target_link_libraries(dg_main_${PROJECT_NAME} ament_index_cpp::ament_index_cpp)
  list(APPEND all_targets dg_main_${PROJECT_NAME})  

endif()


##############
# unit tests #
##############

if(BUILD_TESTING)
  # Nothing yet.
endif()

#
# Documentation.
#

add_documentation()

#
# Install and export.
#

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install resource files
install(DIRECTORY
  resource
  DESTINATION share/${PROJECT_NAME}/
)

# Install libraries and executables
install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS ${all_targets}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES
  DESTINATION include)
ament_package()
