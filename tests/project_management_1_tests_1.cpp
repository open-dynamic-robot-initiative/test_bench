#include "ci_example/gains_configuration.h"
#include "gtest/gtest.h"


// more info: http://www.ibm.com/developerworks/aix/library/au-googletestingframework.html


#define YAML_CONFIG_FILE "ci_example_unit_test.yaml"


/* ******************************* setup of test ******************************* */

// in setup for test, we write the yaml file that will be used
// to test File_configuration. In TearDown, we delete the file.
class Gains_configuration_tests : public ::testing::Test {
protected:
  virtual void SetUp() {
    YAML::Node node;
    node["kp"] = DEFAULT_KP;
    node["kd"] = DEFAULT_KD;
    node["ki"] = DEFAULT_KI;
    std::ofstream fout(YAML_CONFIG_FILE);
    fout<<node
  }
  virtual void TearDown() {
    std::remove(YAML_CONFIG_FILE);
  }
};


/* ******************************* testing Default_configuration ******************************* */

TEST_F(Gains_configuration_tests, default_configuration_test){
  Default_configuration config;
  ASSERT_EQ(config.get_kp(),DEFAULT_KP); 
  ASSERT_EQ(config.get_kd(),DEFAULT_KD); 
  ASSERT_EQ(config.get_ki(),DEFAULT_KI); 
}

/* ******************************* testing File_configuration ******************************* */

TEST_F(Gains_configuration_tests, file_configuration_ok_test){
  File_configuration config(YAML_CONFIG_FILE); // set Setup function above to see file creation
  ASSERT_EQ(config.get_kp(),DEFAULT_KP); 
  ASSERT_EQ(config.get_kd(),DEFAULT_KD); 
  ASSERT_EQ(config.get_ki(),DEFAULT_KI); 
}

TEST_F(Gains_configuration_tests, file_configuration_fail_test){
  File_configuration config("None existing file"); // set Setup function above to see file creation
  ASSERT_EQ(config.has_error(),true); 
}

/* ******************************* testing RosParameters_configuration ? ******************************* */

// because it uses rostest, RosParameters_configuration is tested in a separated file
