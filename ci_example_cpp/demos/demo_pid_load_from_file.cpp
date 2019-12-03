/**
 * @file demo_pid_load_from_file.cpp
 * @author Vincent Berenz
 * license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellshaft.
 * @date 2019-05-22
 * 
 * @brief example of a demo that requires to read a config file
 * @see https://git-amd.tuebingen.mpg.de/amd-clmc/ci_example/wikis/catkin:-how-to-implement-a-demo
 */

#include "ci_example_cpp/pid.hpp"
#include "ci_example_cpp/file_configuration.hpp" 
#include <stdexcept>

/**
 * @brief Run some demo using a YAML file as configuration for the PID
 * controller.
 */
void run_demo(){

  /* displaying what this demo is about */
  std::cout << 
    "This demo shows how to create an executable run by the continuous integration\n" <<
    "which depends on a configuration file. In the solution showed here, the absolute path\n" <<
    "to the configuration file is set during pre-compilation. See code in /demos/demo_pid_load_from_file.cpp\n" <<
    "for details\n\n";
  
  /* reading gains (kp,kd,ki) from yaml config */

  // (look at the CMakeLists.txt to see why TEST_PID_GAINS_YAML_FILE_PATH is replaced by correct abs path  during compilation)
  std::string config_file_path = TEST_PID_GAINS_YAML_FILE_PATH;

  // Gains_configuration is the base class for all configuration, including
  // the one read from yaml file, as done here. 
  ci_example_cpp::File_configuration gains = ci_example_cpp::File_configuration(config_file_path);

  // printing to standard output the gains
  std::cout << "gains read from configuration file:" << std::endl;
  ci_example_cpp::print_configuration(gains);

  // checking reading the config file when fine
  // if not, throwing corresponding error
  if (gains.has_error()){
    throw std::runtime_error(gains.get_error());
  }

  /* creating and running the controller */

  // PID controller creation
  ci_example_cpp::PID controller(gains);
  
  // example of force computation
  double current_position=1;
  double current_velocity=1;
  double delta_time=0.01;
  double target_position=2;
  double force = controller.compute(current_position,current_velocity,target_position,delta_time);
  std::cout<< "computed force: " << force << std::endl;

  // resetting integral of the controller
  controller.reset_integral();
  
}

/**
 * @brief Run the demo in a safe environment.
 */
int main(){
  
  try {
    run_demo();
  } catch(const std::runtime_error& e){
    std::cout << "demo failed !\nerror message:\n" << e.what() << std::endl;
    return 1; // informs continuous integration that this demo did not run successfully
  }

  return 0; // informs continuous integration that this demo did run successfully

}
