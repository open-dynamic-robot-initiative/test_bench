/**
 * @file ci_example_rostest.cpp
 * @author Vincent Berenz
 * license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellshaft.
 * @date 2019-05-22
 * 
 * @brief example of unit tests that use ROS
 * @see https://git-amd.tuebingen.mpg.de/amd-clmc/ci_example/wikis/catkin:-how-to-use-ros-in-unit-tests
 */

#include "ci_example_cpp/rosparameters_configuration.hpp"
#include <gtest/gtest.h>



TEST(RosParam_config_test, read_parameters_test){

  // need to call init before being able to create nodes
  int argc = 1;
  char name[] = "ci_example_rosparam_test";
  char* argv[1];
  argv[0] = name;
  ros::init(argc,argv,"ci_example_rosparam_test");

  // putting gains in parameter server
  // ROSPARAM_KX are declared in gains_configuration.h
  ros::NodeHandle nh;
  nh.setParam(ROSPARAM_KP,1);
  nh.setParam(ROSPARAM_KD,2);
  nh.setParam(ROSPARAM_KI,3);

  // getting corresponding configuration instance
  ci_example_cpp::RosParameters_configuration config;
  ASSERT_EQ(1,config.get_kp());
  ASSERT_EQ(2,config.get_kd());
  ASSERT_EQ(3,config.get_ki());
  

}




