/**
 * @file
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellshaft.
 *
 * @brief Execute the main program to control the test bench.
 */

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "test_bench/dgm_test_bench.hpp"

int main(int, char* [])
{
    // Get the dynamic_graph_manager config file.
    std::string share_path =
        ament_index_cpp::get_package_share_directory("test_bench");
    std::string yaml_path =
        share_path + "/resource/dynamic_graph_manager/test_bench.yaml";
    std::cout << "Loading parameters from " << yaml_path << std::endl;
    YAML::Node param = YAML::LoadFile(yaml_path);

    // Create the dgm.
    test_bench::DGMTestBench dgm;

    // Initialize and run it.
    dgm.initialize(param);
    dgm.run();

    // Wait until ROS is shutdown.
    std::cout << "Wait for shutdown, press CTRL+C to close." << std::endl;
    dynamic_graph_manager::ros_spin();
    dynamic_graph_manager::ros_shutdown();
}
