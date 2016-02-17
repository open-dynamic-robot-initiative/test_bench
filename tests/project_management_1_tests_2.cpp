#include "gtest/gtest.h"

class Project1Test2 : public ::testing::Test {
protected:
  virtual void SetUp() {}
  virtual void TearDown() {}
};


TEST_F(Project1Test2, success_test_2){
  ASSERT_EQ(1,1); 
}

TEST_F(Project1Test2, failure_test_2){
  ASSERT_EQ(1,2); 
}
