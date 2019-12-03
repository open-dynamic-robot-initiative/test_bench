#pragma once

#include <string>
#include <iostream>

namespace ci_example_cpp {

  
  /*! virtual object describing the function concrete configuration objects should present */
  class Gains_configuration {
    
  public:

    virtual ~Gains_configuration(){}
    
    virtual double get_kp() const =0; /**< returns desired kp */
    virtual double get_kd() const =0; /**< returns desired kd */
    virtual double get_ki() const =0; /**< returns desired ki */
    
    /**
     * for enquiring in an error was encountered while reading the configuration
     * @see get_error()
     * @return true if an error has been encountered, false otherwise
     */
    virtual bool has_error() const =0;
    
    /**
     * returns error encountered when reading configuration
     * @see has_error()
     */
    virtual std::string get_error() const =0;
    
  };


  /*! print values encapsulated by the provided configuration console on the standard output */
  void print_configuration(const Gains_configuration& configuration);


}
