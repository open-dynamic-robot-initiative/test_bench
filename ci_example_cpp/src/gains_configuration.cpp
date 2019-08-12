#include "ci_example/gains_configuration.h"


namespace ci_example {

  
  void print_configuration(const Gains_configuration& configuration){
    std::cout << "kp: " << configuration.get_kp() << std::endl;
    std::cout << "kd: " << configuration.get_kd() << std::endl;
    std::cout << "ki: " << configuration.get_ki() << std::endl;
  }


}
