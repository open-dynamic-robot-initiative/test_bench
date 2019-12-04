#pragma once


#include "ci_example_cpp/gains_configuration.hpp"
#include "yaml-cpp/yaml.h"


namespace ci_example_cpp {

  
  /*! @brief Reading configuration from yaml file. */
  class File_configuration : public Gains_configuration {
    
  public:
    
    /**
     * Returns error encountered when reading configuration
     * @param yaml_file absolute path to configuration yaml file. 
     *        The file is expected to have parameters "kp", "kd" and "ki"
     * @see has_error()
     */
    File_configuration(std::string yaml_file);

    /** @copy 
     */
    double get_kp() const;

    /**
     * @brief Get the derivative gain.
     * 
     * @return double 
     */
    double get_kd() const;

    /**
     * @brief Get the integral gain.
     * 
     * @return double 
     */
    double get_ki() const;

    /**
     * @brief Check if there are internal errors.
     * 
     * @return true if an error occurred.
     * @return false otherwise.
     */
    bool has_error() const;

    /**
     * @brief Get the error messages.
     * 
     * @return std::string internal error message.
     */
    std::string get_error() const;
    
  private:
    /** @brief  Proportinal gain. */
    double kp_;
    /** @brief  Derivative gain. */
    double kd_;
    /** @brief  Integral gain. */
    double ki_;
    /** @brief  Internal error message. */
    std::string error_message_;
    /** @brief  True if an error occured. */
    bool error_;
    
  };

}
