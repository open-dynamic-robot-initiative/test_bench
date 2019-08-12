#include <pybind11/pybind11.h>
#include "ci_example/basic_pid.h"

using namespace ci_example;

PYBIND11_MODULE(ci_example,m) {

  /*
  pybind11::class_<File_configuration>(m,"File_configuration")
    .def(pybind11::init<std::string>())
    .def("get_kp",&File_configuration::get_kp)
    .def("get_kd",&File_configuration::get_kd)
    .def("get_ki",&File_configuration::get_ki)
    .def("has_error",&File_configuration::has_error)
    .def("get_error",&File_configuration::get_error);
  */

  pybind11::class_<PID>(m,"PID")
    .def(pybind11::init<>())
    .def("compute",&PID::compute)
    .def("reset_integral",&PID::reset_integral);

  pybind11::class_<Biniou>(m,"BINIOU")
    .def(pybind11::init<int>())
    .def("play",&Biniou::play);
  
}
