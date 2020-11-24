/**
 * @file
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellshaft.
 *
 * @brief DGM wrapper around the test bench containing 12 motors.
 */

#include "test_bench/dgm_test_bench.hpp"
#include "dynamic_graph_manager/ros.hpp"
#include "yaml_utils/yaml_cpp_fwd.hpp"

namespace test_bench
{
DGMTestBench::DGMTestBench()
{
}

DGMTestBench::~DGMTestBench()
{
}

void DGMTestBench::initialize_hardware_communication_process()
{
    std::string network_id = YAML::ReadParameter<std::string>(
        params_["hardware_communication"], "network_id");

    robot_.initialize(network_id);
}

void DGMTestBench::get_sensors_to_map(dynamic_graph_manager::VectorDGMap& map)
{
    robot_.acquire_sensors();
    const TestBenchData& robot_data = robot_.get_data();

    /**
     * Joint data
     */
    map.at("joint_positions") = robot_data.joint_positions;
    map.at("joint_velocities") = robot_data.joint_velocities;
}

void DGMTestBench::set_motor_controls_from_map(
    const dynamic_graph_manager::VectorDGMap& map)
{
    robot_.send_target_joint_torque(map.at("ctrl_joint_torques"));
}

}  // namespace test_bench
