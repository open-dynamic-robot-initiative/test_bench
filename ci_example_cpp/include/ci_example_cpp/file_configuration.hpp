#pragma once


#include "ci_example_cpp/gains_configuration.hpp"
#include "yaml-cpp/yaml.h"


namespace ci_example_cpp {

  
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
    
    /*! Get the proportional gain */
    double get_kp() const;
    /*! Get the derivative gain */
    double get_kd() const;
    /*! Get the integral gain */
    double get_ki() const;
    /*! Check if there are internal errors */
    bool has_error() const;
    /*! Get the error messages */
    std::string get_error() const;
    
  private:
    double kp_; /*!< Proportinal gain. */
    double kd_; /*!< Derivative gain. */
    double ki_; /*!< Integral gain. */
    std::string error_message_; /*!< Internal error message. */
    bool error_; /*!< True is an error occured. */
    
  };

}
