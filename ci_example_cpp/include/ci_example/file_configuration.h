#pragma once


#include "ci_example/gains_configuration.h"
#include "yaml-cpp/yaml.h"


namespace ci_example {

  
  /*! Reading configuration from yaml file */
  class File_configuration : public Gains_configuration {
    
  public:
    
    /**
     * returns error encountered when reading configuration
     * @param yaml_file absolute path to configuration yaml file. 
     *        The file is expected to have parameters "kp", "kd" and "ki"
     * @see has_error()
     */
    File_configuration(std::string yaml_file);
    
    double get_kp() const;
    double get_kd() const;
    double get_ki() const;
    bool has_error() const;
    std::string get_error() const;
    
  private:
    double kp,kd,ki;
    std::string error_message;
    bool error;
    
  };

}
