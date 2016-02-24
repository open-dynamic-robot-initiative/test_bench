#include "ci_example/basic_pid.h" 

void run_demo(){

  /* displaying what this demo is about */
  std::cout << 
    "This demo shows how to run a demo that requires complex setup, in this case turning on ROS\n" <<
    "and putting data in ros parameter server.\n"<<
    "The executable related to this file is not started directly, but via a related executable sh script 'ci_example_ros_demo' \n" <<
    "which does all the setup work\n"<<
    "For equivalent unit-tests implementation, see /tests/ci_example_rostest.cpp\n\n"; 

  // code below assume ros is on and gains data in ros parameter server
  std::shared_ptr<ci_example::Gains_configuration> configuration(new ci_example::RosParameters_configuration());

  // throwing error if failing to read params from rosparameter server
  if (configuration->has_error()){
    throw std::runtime_error(configuration->get_error());
  }

  // running pid 
  ci_example::PID controller(configuration);
  
  // example of force computation
  double current_position=1;
  double current_velocity=1;
  double delta_time=0.01;
  double target_position=2;
  double force = controller.compute(current_position,current_velocity,target_position,delta_time);
  std::cout<< "computed force: " << force << std::endl;

  // resetting integral of the controller
  // (useless here because we do not reuse it)
  controller.reset_integral();
  
}


int main(){
  
  try {
    run_demo();
  } catch(const std::exception& e){
    std::cout << "demo failed !\nerror message:\n" << e.what() << std::endl;
    return 1; // informs continuous integration that this demo did not run successfully
  }

  return 0; // informs continuous integration that this demo did run successfully

}
