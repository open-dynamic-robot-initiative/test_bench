#include "gtest/gtest.h"

class Project1Test1 : public ::testing::Test {
protected:
  virtual void SetUp() {}
  virtual void TearDown() {}
};


TEST_F(Project1Test1, success_test_1){
  ASSERT_EQ(1,1); 
}

TEST_F(Project1Test1, failure_test_1){
  ASSERT_EQ(1,2); 
}
