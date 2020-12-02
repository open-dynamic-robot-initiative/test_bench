/**
 * @file
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellshaft.
 *
 * @brief Defines the drivers of the test_bench
 */

#pragma once

#include <Eigen/Eigen>

#include "master_board_sdk/master_board_interface.h"

namespace Eigen
{
typedef Eigen::Matrix<double, 12, 1> Vector12d;
}

namespace test_bench
{
typedef std::array<double, 12> Array12d;

struct TestBenchData
{
    /*
     * Joint data
     */

    /** @brief Joint positions [rad]. */
    Eigen::Vector12d joint_positions;

    /** @brief Joint velocities [rad/s]. */
    Eigen::Vector12d joint_velocities;

    /** @brief Desired motor current. */
    Eigen::Vector12d desired_motor_current;

    /*
     * Hardware data
     */

    /** @brief Joint gear ratios. */
    Eigen::Vector12d joint_gear_ratios;

    /** @brief motor_torque_constants_ are the motor torque constants. */
    Eigen::Vector12d motor_torque_constants;

    /** @brief Max motor current. */
    Eigen::Vector12d max_motor_current;

    /** Joint rotation direction. negative means we invert it. */
    Eigen::Vector12d polarities;

    /**
     * Additional data
     */

    /** @brief slider_positions_ is the position of the linear potentiometer.
     * Can be used as a joystick input.
     */
    Eigen::Vector12d slider_positions_;
};

enum TestBenchState
{
    initial,
    getting_ready,
    ready
};

class TestBench
{
public:
    /**
     * @brief DGMTeststand is the constructor.
     */
    TestBench();

    /**
     * @brief ~DGMTeststand is the destructor.
     */
    ~TestBench(){};

    /**
     * @brief initialize is the function that initialize the hardware.
     */
    void initialize(const std::string& network_id);

    /**
     * @brief Fills in the internal sensors data.
     */
    void acquire_sensors();

    /**
     * @brief send_target_torques sends the target currents to the motors
     */
    void send_target_joint_torque(
        Eigen::Ref<const Eigen::Vector12d> target_joint_torque);

    /**
     * @brief Check if everything is setup (motor aligned, no timeout, etc).
     *
     * @return true
     * @return false
     */
    bool ready();

    /**
     * @brief get_joint_positions
     * WARNING !!!! The method acquire_sensors() has to be called prior to
     * any getter to have up to date data.
     *
     * @return  the joint angle of each module
     */
    const TestBenchData& get_data()
    {
        return data_;
    }

private:
    /**
     * @brief Calibrate the robot joint position
     *
     * @param soft or mechanical calibration?
     * @param zero_to_index_angle is the angle between the theoretical zero and
     * the next positive angle.
     * @param index_angle is the positition of the next index.
     */
    void calibrate_joint_position(bool mechanical_calibration,
                                  Array12d& zero_to_index_angle,
                                  Array12d& index_angle);

    /**
     * Entries for the real hardware.
     */

    /** @brief Robot driver sdk object. Allowing communication with the robot.
     */
    std::shared_ptr<MasterBoardInterface> robot_drivers;

    /** @brief Sensor and internal robot data. */
    TestBenchData data_;
    
    /** @brief State of the state machine managing the initialization procedure.
     */
    TestBenchState state_;
};

}  // namespace test_bench
