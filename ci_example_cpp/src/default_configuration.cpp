#include "ci_example_cpp/default_configuration.hpp"


namespace ci_example_cpp {


  double DefaultConfiguration::get_kp() const {
    return DEFAULT_KP;
  }

  double DefaultConfiguration::get_kd() const {
    return DEFAULT_KD;
  }

  double DefaultConfiguration::get_ki() const {
    return DEFAULT_KI;
  }

  bool DefaultConfiguration::has_error() const {
    return false;
  }

  std::string DefaultConfiguration::get_error() const {
    return std::string("no error");
  }

}
