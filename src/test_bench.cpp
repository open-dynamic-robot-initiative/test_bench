/**
 * @file
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 *
 * @brief Implement of the TestBench class.
 */

#include "test_bench/test_bench.hpp"

#include "real_time_tools/spinner.hpp"

namespace test_bench
{
TestBench::TestBench()
{
    // initialize internal data.
    data_.joint_positions.setZero();
    data_.joint_velocities.setZero();
    data_.desired_motor_current.setZero();

    // Constants.
    data_.joint_gear_ratios.fill(1.0);
    data_.motor_torque_constants.fill(0.025);
    data_.max_motor_current.fill(2.0);
    data_.polarities.fill(1.0);

    // set the pointers to nullptr
    robot_drivers = nullptr;

    // Start in initial state.
    state_ = TestBenchState::initial;
}

void TestBench::initialize(const std::string& network_id)
{
    robot_drivers = std::make_shared<MasterBoardInterface>(network_id);
    robot_drivers->Init();
    // Initialisation, send the init commands
    for (int i = 0; i < 6 /*nb motor drivers*/; i++)
    {
        robot_drivers->motor_drivers[i].motor1->SetCurrentReference(0.0);
        robot_drivers->motor_drivers[i].motor2->SetCurrentReference(0.0);
        robot_drivers->motor_drivers[i].motor1->Enable();
        robot_drivers->motor_drivers[i].motor2->Enable();
        robot_drivers->motor_drivers[i].EnablePositionRolloverError();
        robot_drivers->motor_drivers[i].SetTimeout(5);
        robot_drivers->motor_drivers[i].Enable();
    }
}

bool TestBench::ready()
{
    bool ready = !robot_drivers->IsTimeout();
    for (int i = 0; i < 6; i++)
    {
        if (!robot_drivers->motor_drivers[i / 2].is_connected)
            continue;  // ignoring the motors of a disconnected slave

        ready &= robot_drivers->motors[i].IsEnabled() &&
                 robot_drivers->motors[i].IsReady();
    }
    return ready;
}

void TestBench::acquire_sensors()
{
    robot_drivers->ParseSensorData();
    for (Eigen::Index i = 0; i < data_.joint_positions.size(); i++)
    {
        data_.joint_positions(i) = robot_drivers->motors[i].get_position();
    }
    data_.joint_positions = data_.joint_positions.cwiseProduct(data_.polarities)
                                .cwiseQuotient(data_.joint_gear_ratios);

    for (Eigen::Index i = 0; i < data_.joint_velocities.size(); i++)
    {
        data_.joint_velocities(i) = robot_drivers->motors[i].get_velocity();
    }
    data_.joint_velocities =
        data_.joint_velocities.cwiseProduct(data_.polarities)
            .cwiseQuotient(data_.joint_gear_ratios);
}

void TestBench::send_target_joint_torque(
    Eigen::Ref<const Eigen::Vector12d> target_joint_torque)
{
    switch (state_)
    {
        case TestBenchState::initial:
            if (!robot_drivers->IsTimeout() &&
                !robot_drivers->IsAckMsgReceived())
            {
                robot_drivers->SendInit();
            }
            else
            {
                state_ = TestBenchState::getting_ready;
            }
            break;
        case TestBenchState::getting_ready:
            if (!ready())
            {
                data_.desired_motor_current.fill(0.0);
                robot_drivers->SendCommand();
            }
            else
            {
                state_ = TestBenchState::ready;
            }
            break;
        case TestBenchState::ready:
            data_.desired_motor_current =
                data_.polarities.cwiseProduct(target_joint_torque)
                    .cwiseQuotient(data_.joint_gear_ratios)
                    .cwiseQuotient(data_.motor_torque_constants);

            // Current clamping.
            data_.desired_motor_current =
                data_.desired_motor_current.cwiseMin(data_.max_motor_current)
                    .cwiseMax(-data_.max_motor_current);

            for (int i = 0; i < 12; i++)
            {
                robot_drivers->motors[i].set_current_ref(
                    data_.desired_motor_current(i));
            }
            robot_drivers->SendCommand();
            break;
        default:
            break;
    }
}

}  // namespace test_bench
