#include "ci_example/rosparameters_configuration.h"


namespace ci_example {

    static bool get_parameter(const ros::NodeHandle &nh, const std::string &parameter, double &get_value){
    ros::Rate wait(10);
    bool success = false;
    while (ros::ok()){
      success = nh.getParam(parameter,get_value);
      if (success) return true;
      wait.sleep();
    }
    return false;
  }
  
  RosParameters_configuration::RosParameters_configuration(){
    this->error = false;
    this->error_message="no error";
    ros::NodeHandle nh;
    std::vector<std::string> parameters = {ROSPARAM_KP,ROSPARAM_KD,ROSPARAM_KI};
    std::vector<double*> gains = {&(this->kp),&(this->kd),&(this->ki)};
    for(unsigned int i=0;i<parameters.size();i++){
      bool success = get_parameter(nh,parameters[i],*(gains[i]));
      if (!success) {
	this->error = true;
	this->error_message = "roscore shut down before parameter "+parameters[i]+" could be read";
      }
    }
  }

  double RosParameters_configuration::get_kp() const { 
    return this->kp;
  }


  double RosParameters_configuration::get_kd() const { 
    return this->kd;
  }


  double RosParameters_configuration::get_ki() const { 
    return this->ki;
  }

  bool RosParameters_configuration::has_error() const { 
    return this->error;
  }

  std::string RosParameters_configuration::get_error() const { 
    return this->error_message;
  }
  

}
