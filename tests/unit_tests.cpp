/**
 * @file ci_example_ut.cpp
 * @author Vincent Berenz
 * license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 *
 * @brief example of unit tests
 * @see
 * https://git-amd.tuebingen.mpg.de/amd-clmc/ci_example/wikis/catkin:-how-to-implement-unit-tests
 */

#include <fstream>
#include <iostream>
#include "ci_example/file_configuration.hpp"
#include "ci_example/pid.hpp"
#include "gtest/gtest.h"

// more info:
// http://www.ibm.com/developerworks/aix/library/au-googletestingframework.html

// quick reference:
// http://www.cheezyworld.com/wp-content/uploads/2010/12/PlainGoogleQuickTestReferenceGuide1.pdf

#define YAML_CONFIG_FILE "ci_example_unit_test.yaml"

/* ******************************* setup of test *******************************
 */

// in setup for test, we write the yaml file that will be used
// to test File_configuration. In TearDown, we delete the file.
class PID_tests : public ::testing::Test
{
protected:
    void SetUp()
    {
        YAML::Node node;
        node["kp"] = DEFAULT_KP;
        node["kd"] = DEFAULT_KD;
        node["ki"] = DEFAULT_KI;
        std::ofstream fout(YAML_CONFIG_FILE);
        fout << node;
    }
    void TearDown()
    {
        std::remove(YAML_CONFIG_FILE);
    }
};

/* ******************************* testing DefaultConfiguration
 * ******************************* */

TEST_F(PID_tests, default_configuration_test)
{
    ci_example::DefaultConfiguration config;
    ASSERT_EQ(config.get_kp(), DEFAULT_KP);
    ASSERT_EQ(config.get_kd(), DEFAULT_KD);
    ASSERT_EQ(config.get_ki(), DEFAULT_KI);
    ASSERT_EQ(config.has_error(), false);
}

/* ******************************* testing File_configuration
 * ******************************* */

TEST_F(PID_tests, file_configuration_ok_test)
{
    // see CMakeLists.txt to see how this change to valid path
    ci_example::File_configuration config(YAML_CONFIG_FILE);
    ASSERT_EQ(config.get_kp(), DEFAULT_KP);
    ASSERT_EQ(config.get_kd(), DEFAULT_KD);
    ASSERT_EQ(config.get_ki(), DEFAULT_KI);
}

TEST_F(PID_tests, file_configuration_fail_test)
{
    ci_example::File_configuration config("None existing file");
    ASSERT_EQ(config.has_error(), true);
}

/* ******************************* testing File_configuration
 * ******************************* */
/* ******************************* with default configuration file
 * ******************************* */

TEST_F(PID_tests, read_config_file_test)
{
    ci_example::File_configuration config(YAML_CONFIG_FILE);
    ASSERT_EQ(config.has_error(), false);
}

TEST_F(PID_tests, configurations_same_results_test)
{
    // random data for testing
    double position = 1;
    double velocity = 1;
    double position_target = 2;
    double delta_time = 0.01;

    ci_example::DefaultConfiguration default_config;
    ci_example::PID controller_default(default_config);
    double force_default = controller_default.compute(
        position, velocity, position_target, delta_time);

    ci_example::File_configuration file_config(YAML_CONFIG_FILE);
    ci_example::PID controller_file(file_config);
    double force_file = controller_file.compute(
        position, velocity, position_target, delta_time);

    ASSERT_EQ(force_default, force_file);
}

// does integral integrates ?
TEST_F(PID_tests, integral)
{
    // random data for testing
    double position = 1;
    double velocity = 1;
    double position_target = 2;
    double delta_time = 0.01;

    ci_example::PID& controller = ci_example::get_default_pid();
    double force_1 =
        controller.compute(position, velocity, position_target, delta_time);
    double force_2 =
        controller.compute(position, velocity, position_target, delta_time);

    ASSERT_NE(force_1, force_2);
}

// does reset of integral work ?
TEST_F(PID_tests, reset_integral)
{
    // random data for testing
    double position = 1;
    double velocity = 1;
    double position_target = 2;
    double delta_time = 0.01;

    // running pid and integrating
    ci_example::PID& controller = ci_example::get_default_pid();
    double force_1 =
        controller.compute(position, velocity, position_target, delta_time);

    // reset integral
    controller.reset_integral();

    // run controller again
    double force_reset =
        controller.compute(position, velocity, position_target, delta_time);

    ASSERT_EQ(force_1, force_reset);
}

// generated force is zero if already at target ?
TEST_F(PID_tests, zero_force_at_target)
{
    // random data for testing
    double position = 1;
    double velocity = 0;
    double position_target = position;
    double delta_time = 0.01;

    ci_example::PID& controller = ci_example::get_default_pid();
    double force =
        controller.compute(position, velocity, position_target, delta_time);

    ASSERT_EQ(force, 0);
}

// does the controller push to the right direction ?
TEST_F(PID_tests, right_direction)
{
    // random data for testing
    double position = 0;
    double velocity = 0;
    double position_target = 1;
    double delta_time = 0.01;

    ci_example::PID& controller = ci_example::get_default_pid();
    double force =
        controller.compute(position, velocity, position_target, delta_time);
    ASSERT_GT(force, 0);

    controller.reset_integral();
    position_target = -1;
    force = controller.compute(position, velocity, position_target, delta_time);
    ASSERT_LT(force, 0);

    controller.reset_integral();
    position_target = position;
    velocity = -1;
    force = controller.compute(position, velocity, position_target, delta_time);
    ASSERT_GT(force, 0);

    controller.reset_integral();
    velocity = 1;
    force = controller.compute(position, velocity, position_target, delta_time);
    ASSERT_LT(force, 0);
}
