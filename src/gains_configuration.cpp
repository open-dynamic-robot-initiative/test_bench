#include <fstream>
#include "ci_example/gains_configuration.h" // where DEFAULT_KP,KD and KI are declared


// you may notice that there is no doxygen friendly documentation here.
// all documentation should be in the header files.
// but comments are welcomed

namespace ci_example {


  // --------------------------- File Configuration --------------------------- // 

  File_configuration::File_configuration(std::string yaml_file){

    try {

      YAML::Node node = YAML::LoadFile(yaml_file);
      this->kp = node["kp"].as<double>();
      this->kd = node["kd"].as<double>();
      this->ki = node["ki"].as<double>();

    } catch( const  std::exception& e ){

      this->error = true;
      this->error_message = e.what();

    }

    this->error = false;

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


  // --------------------------- Functions --------------------------- // 

  void console_configuration(const std::shared_ptr<Gains_configuration> configuration);
    std::cout << "kp: " << configuration->get_kp() << std::endl;
    std::cout << "kd: " << configuration->get_kd() << std::endl;
    std::cout << "ki: " << configuration->get_ki() << std::endl;
  }

