#include "ci_example/default_configuration.h"


namespace ci_example {


  double Default_configuration::get_kp() const {
    return DEFAULT_KP;
  }

  double Default_configuration::get_kd() const {
    return DEFAULT_KD;
  }

  double Default_configuration::get_ki() const {
    return DEFAULT_KI;
  }

  bool Default_configuration::has_error() const {
    return false;
  }

  std::string Default_configuration::get_error() const {
    return std::string("no error");
  }

}
