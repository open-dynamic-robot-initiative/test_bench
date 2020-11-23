/**
 * @file dgm_teststandx.hpp
 * @author Manuel Wuthrich
 * @author Maximilien Naveau
 * @author Julian Viereck
 * @author Johannes Pfleging
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellshaft.
 */

#ifndef DGM_TESTSTAND_HH
#define DGM_TESTSTAND_HH

#include "test_bench/test_bench.hpp"
#include <dynamic_graph_manager/dynamic_graph_manager.hh>


namespace test_bench
{
class DGMTestBench : public dynamic_graph::DynamicGraphManager
{
public:
    /**
     * @brief DGMTeststand is the constructor.
     */
    DGMTestBench();

    /**
     * @brief ~DGMTeststand is the destructor.
     */
    ~DGMTestBench();

    /**
     * @brief initialize_hardware_communication_process is the function that
     * initialize the hardware.
     */
    void initialize_hardware_communication_process();

    /**
     * @brief get_sensors_to_map acquires the sensors data and feeds it to the
     * input/output map
     * @param[in][out] map is the sensors data filled by this function.
     */
    void get_sensors_to_map(dynamic_graph::VectorDGMap& map);

    /**
     * @brief set_motor_controls_from_map reads the input map that contains the
     * controls and send these controls to the hardware.
     * @param map
     */
    void set_motor_controls_from_map(const dynamic_graph::VectorDGMap& map);

private:

    TestBench robot_;
};

}  // namespace test_bench
