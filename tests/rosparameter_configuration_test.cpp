#include "ci_example/gains_configuration.h"
#include <gtest/gtest.h>

TEST(RosParameters_configuration_test, basicTest){

  // putting gains in parameter server
  // ROSPARAM_KX are declared in gains_configuration.h
  ros::NodeHandle nh;
  nh.setParam(ROSPARAM_KP,1);
  nh.setParam(ROSPARAM_KD,2);
  nh.setParam(ROSPARAM_KI,3);

  // getting corresponding configuration instance
  RosParameters_configuration config;
  ASSERT_EQ(1,config.get_kp())
  ASSERT_EQ(2,config.get_kd())
  ASSERT_EQ(3,config.get_ki())
  

}




