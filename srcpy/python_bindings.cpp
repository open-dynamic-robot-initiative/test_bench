/**
 * @file python_bindings.cpp
 * license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 */

#include <pybind11/pybind11.h>
#include "ci_example_cpp/pid.hpp"


using namespace ci_example_cpp;

PYBIND11_MODULE(basic_pid,m) {

  pybind11::class_<PID>(m,"PID")
    .def(pybind11::init<>())
    .def("compute",&PID::compute)
    .def("reset_integral",&PID::reset_integral);

}
