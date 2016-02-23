#include "ci_example/gains_configuration.h"
#include "gtest/gtest.h"


// more info: http://www.ibm.com/developerworks/aix/library/au-googletestingframework.html

// quick reference: http://www.cheezyworld.com/wp-content/uploads/2010/12/PlainGoogleQuickTestReferenceGuide1.pdf


#define YAML_CONFIG_FILE "ci_example_unit_test.yaml"


/* ******************************* setup of test ******************************* */

// in setup for test, we write the yaml file that will be used
// to test File_configuration. In TearDown, we delete the file.
class PID_tests : public ::testing::Test {
protected:
  void SetUp() {
    YAML::Node node;
    node["kp"] = DEFAULT_KP;
    node["kd"] = DEFAULT_KD;
    node["ki"] = DEFAULT_KI;
    std::ofstream fout(YAML_CONFIG_FILE);
    fout<<node;
  }
  void TearDown() {
    std::remove(YAML_CONFIG_FILE);
  }
};


/* ******************************* testing Default_configuration ******************************* */

TEST_F(PID_tests, default_configuration_test){
  ci_example::Default_configuration config;
  ASSERT_EQ(config.get_kp(),DEFAULT_KP); 
  ASSERT_EQ(config.get_kd(),DEFAULT_KD); 
  ASSERT_EQ(config.get_ki(),DEFAULT_KI); 
  ASSERT_EQ(config.has_error(),false); 
}

/* ******************************* testing File_configuration ******************************* */

TEST_F(PID_tests, file_configuration_ok_test){
  ci_example::File_configuration config(YAML_CONFIG_FILE); // set Setup function above to see file creation
  ASSERT_EQ(config.get_kp(),DEFAULT_KP); 
  ASSERT_EQ(config.get_kd(),DEFAULT_KD); 
  ASSERT_EQ(config.get_ki(),DEFAULT_KI); 
}

TEST_F(PID_tests, file_configuration_fail_test){
  ci_example::File_configuration config("None existing file"); 
  ASSERT_EQ(config.has_error(),true); 
}


/* ******************************* testing File_configuration ******************************* */
/* ******************************* with default configuration file ******************************* */

// most of the time it will be non practical to write the test config file at runtime as it
// was done above in the SetUp method. Here is an example of using the config file stored in 
// the "config" folder of this catkin package. 
// look at the CMakeLists.txt to see why TEST_YAML_FILE_PATH is replaced during compilation
// by the absolute path to the file /config/test_pid_gains.yaml
TEST_F(PID_tests, read_config_file_test){
  ci_example::File_configuration config(TEST_YAML_FILE_PATH); 
  ASSERT_EQ(config.has_error(),false);
}


/* ******************************* sanity check: using default or file configuration ******************************* */
/* ******************************* should have same result  ******************************* */

TEST_F(PID_tests, configurations_same_results_test){

  // random data for testing
  double position = 1;
  double velocity = 1;
  double position_target=2;
  double delta_time=0.01;

  // will be used to host the configurations
  // note: a shared pointer is same thing as a pointer, except 
  // that is manages the memory automatically. It does not need to be deleted.
  std::shared_ptr<ci_example::Gains_configuration> config;

  // because config is a pointer to a Gains_configuration, and an instance 
  // of Default_configuration is, through inheritance, also an instance of
  // Gains_configuration, this works 
  config.reset(new ci_example::Default_configuration());
  ci_example::PID controller_default(config);
  double force_default = controller_default.compute(position,velocity,position_target,delta_time);

  // an instance of File_configuration is also an instance of Gains_configuration
  config.reset(new ci_example::File_configuration(YAML_CONFIG_FILE));
  ci_example::PID controller_file(config);
  double force_file = controller_file.compute(position,velocity,position_target,delta_time);

  ASSERT_EQ(force_default,force_file); 

}

/* ******************************* testing of PID controller ******************************* */

// arguably, these tests are way insufficient. E.g. stability of the controller
// is not tested. Ideally, some simulation should be used for better testing.
// Use your best judgement to evaluate to which extend investing time and effort is worth the gain.
// But these considerations should not emped in the implementation of simpler and basic test
// such as the one presented here. 
// *** Such test take 5 min to implement **, 
// yet they can be very useful by catching at least some of the main error that may occure

// does integral integrates ?
TEST_F(PID_tests, integral){

  // random data for testing
  double position = 1;
  double velocity = 1;
  double position_target=2;
  double delta_time=0.01;

  std::shared_ptr<ci_example::PID> controller = ci_example::get_default_pid();
  double force_1 = controller->compute(position,velocity,position_target,delta_time);
  double force_2 = controller->compute(position,velocity,position_target,delta_time);

  ASSERT_NE(force_1,force_2);

}

// does reset of integral work ?
TEST_F(PID_tests, reset_integral){

  // random data for testing
  double position = 1;
  double velocity = 1;
  double position_target=2;
  double delta_time=0.01;

  // running pid and integrating
  std::shared_ptr<ci_example::PID> controller = ci_example::get_default_pid();
  double force_1 = controller->compute(position,velocity,position_target,delta_time);
  double force_2 = controller->compute(position,velocity,position_target,delta_time);

  // reset integral
  controller->reset_integral();

  // run controller again
  double force_reset = controller->compute(position,velocity,position_target,delta_time);

  ASSERT_EQ(force_1,force_reset);

}

// generated force is zero if already at target ?
TEST_F(PID_tests, zero_force_at_target){

  // random data for testing
  double position = 1;
  double velocity = 0;
  double position_target=position;
  double delta_time=0.01;

  std::shared_ptr<ci_example::PID> controller = ci_example::get_default_pid();
  double force = controller->compute(position,velocity,position_target,delta_time);

  ASSERT_EQ(force,0);

}

// does the controller push to the right direction ?
TEST_F(PID_tests, right_direction){

  // random data for testing
  double position = 0;
  double velocity = 0;
  double position_target=1;
  double delta_time=0.01;

  std::shared_ptr<ci_example::PID> controller = ci_example::get_default_pid();
  double force = controller->compute(position,velocity,position_target,delta_time);
  ASSERT_GT(force,0);

  controller->reset_integral();
  position_target=-1;
  force = controller->compute(position,velocity,position_target,delta_time);
  ASSERT_LT(force,0);

  controller->reset_integral();
  position_target=position;
  velocity=-1;
  force = controller->compute(position,velocity,position_target,delta_time);
  ASSERT_GT(force,0);
    
  controller->reset_integral();
  velocity=1;
  force = controller->compute(position,velocity,position_target,delta_time);
  ASSERT_LT(force,0);
  

}



/* ******************************* testing RosParameters_configuration ? ******************************* */

// because it uses rostest, RosParameters_configuration is tested in a separated file