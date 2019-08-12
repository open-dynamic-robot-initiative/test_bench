#include "ci_example/file_configuration.h"


namespace ci_example {

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


  double File_configuration::get_kp() const { 
    return this->kp;
  }


  double File_configuration::get_kd() const { 
    return this->kd;
  }


  double File_configuration::get_ki() const { 
    return this->ki;
  }

  bool File_configuration::has_error() const { 
    return this->error;
  }

  std::string File_configuration::get_error() const { 
    return this->error_message;
  }
  

}
