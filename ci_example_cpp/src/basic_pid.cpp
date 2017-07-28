#include "ci_example/basic_pid.h" // where DEFAULT_KP,KD and KI are declared


// you may notice that there is no doxygen friendly documentation here.
// all documentation should be in the header files.
// (but comments are welcomed)

// note the usage of "this->" to access class members.
// this is a nice way to be unambiguous on the origin of variables


namespace ci_example {

 this is an invalid statement in c++, put here with the purpose to test continuous integration


  // --------------------------- File Configuration --------------------------- // 

  File_configuration::File_configuration(std::string yaml_file){

    try {

      YAML::Node node = YAML::LoadFile(yaml_file);
      this->kp = node["kp"].as<double>();
      this->kd = node["kd"].as<double>();
      this->ki = node["ki"].as<double>();

      this->error = false;

    } catch( const  std::exception& e ){

      this->error = true;
      this->error_message = e.what();

    }

  }


  double File_configuration::get_kp() { 
    return this->kp;
  }


  double File_configuration::get_kd() { 
    return this->kd;
  }


  double File_configuration::get_ki() { 
    return this->ki;
  }

  bool File_configuration::has_error() { 
    return this->error;
  }

  std::string File_configuration::get_error() { 
    return this->error_message;
  }


  // --------------------------- Default Configuration --------------------------- // 

  
  double Default_configuration::get_kp(){
    return DEFAULT_KP;
  }

  double Default_configuration::get_kd(){
    return DEFAULT_KD;
  }

  double Default_configuration::get_ki(){
    return DEFAULT_KI;
  }

  bool Default_configuration::has_error(){
    return false;
  }

  std::string Default_configuration::get_error(){
    return std::string("no error");
  }


  // --------------------------- RosParameters_configuration --------------------------- // 

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
    for(int i=0;i<parameters.size();i++){
      bool success = get_parameter(nh,parameters[i],*(gains[i]));
      if (!success) {
	this->error = true;
	this->error_message = "roscore shut down before parameter "+parameters[i]+" could be read";
      }
    }
  }


  double RosParameters_configuration::get_kp() { 
    return this->kp;
  }


  double RosParameters_configuration::get_kd() { 
    return this->kd;
  }


  double RosParameters_configuration::get_ki() { 
    return this->ki;
  }

  bool RosParameters_configuration::has_error() { 
    return this->error;
  }

  std::string RosParameters_configuration::get_error() { 
    return this->error_message;
  }


  // --------------------------- Functions --------------------------- // 

  void print_configuration(const std::shared_ptr<Gains_configuration> configuration){
    std::cout << "kp: " << configuration->get_kp() << std::endl;
    std::cout << "kd: " << configuration->get_kd() << std::endl;
    std::cout << "ki: " << configuration->get_ki() << std::endl;
  }


  // --------------------------- PID controller --------------------------- // 

  PID::PID(std::shared_ptr<Gains_configuration> configuration){
    this->integral=0;
    this->configuration = configuration;
  }
  
  double PID::compute(const double position, const double velocity, const double position_target, const double delta_time){
    double position_error = position_target-position;
    this->integral += delta_time * position_error;
    double f = position_error*this->configuration->get_kp() - velocity*this->configuration->get_kd() + integral*this->configuration->get_ki();
    return f;
  }
  
  void PID::reset_integral(){
    this->integral=0;
  }

  // --------------------------- PID factories --------------------------- // 

  std::shared_ptr<PID> get_default_pid(){
    std::shared_ptr<Gains_configuration> configuration(new Default_configuration());
    std::shared_ptr<PID> controller(new PID(configuration));
    return controller;
  }


}


