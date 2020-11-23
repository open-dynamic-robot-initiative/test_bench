/**
 * @file
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 *
 * @brief Implement of the TestBench class.
 */

#include "test_bench/test_bench.hpp"

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
    data_.motor_torque_constants.fill();
    data_.max_motor_current.fill();
    data_.polarities.fill();

    // set the pointers to nullptr
    robot_drivers = nullptr;
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

    std::chrono::time_point<std::chrono::system_clock> last =
        std::chrono::system_clock::now();
    while (!robot_drivers->IsTimeout() && !robot_drivers->IsAckMsgReceived())
    {
        if (((std::chrono::duration<double>)(std::chrono::system_clock::now() -
                                             last))
                .count() > dt)
        {
            last = std::chrono::system_clock::now();
            robot_drivers->SendInit();
        }
    }
}

bool TestBench::ready()
{
    bool ready = robot_drivers->IsTimeout();
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

    for (size_t i = 0; i < data_.joint_positions.size(); i++)
    {
        data_.joint_positions(i) = robot_drivers->motors[i]->GetPosition();
    }
    positions = positions.cwiseProduct(data_.polarities)
                    .cwiseQuotient(data.gear_ratios);

    for (size_t i = 0; i < data_.joint_velocities.size(); i++)
    {
        data_.joint_velocities(i) = robot_drivers->motors[i]->GetVelocity();
    }
    velocities = velocities.cwiseProduct(data_.polarities)
                     .cwiseQuotient(data.gear_ratios);
}

bool TestBench::send_target_joint_torque(
    const Eigen::Ref<Eigen::Vector12d> target_joint_torque)
{
    data_.desired_motor_current =
        polarities_.cwiseProduct(data_.target_joint_torque)
            .cwiseQuotient(data_.joint_gear_ratios)
            .cwiseQuotient(data_.motor_torque_constants);

    // Current clamping.
    data_.desired_motor_current =
        desired_motor_current.cwiseMin(data_.max_motor_current)
            .cwiseMax(-data_.max_motor_current);

    for (int i = 0; i < 12; i++)
    {
        robot_drivers->motors[i]->SetCurrentReference(
            data_.desired_motor_current(i));
    }
}

}  // namespace test_bench
