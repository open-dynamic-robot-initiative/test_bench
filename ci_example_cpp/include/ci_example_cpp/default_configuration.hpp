#pragma once

#include "ci_example_cpp/gains_configuration.hpp"


#define DEFAULT_KP 1.0
#define DEFAULT_KD 1.0
#define DEFAULT_KI 1.0


namespace ci_example_cpp {

  /*! Returning default configuration: kp: DEFAULT_KP, kd: DEFAULT_KD, ki: DEFAULT_KI */
  class Default_configuration : public Gains_configuration {
    
  public:
    /** Here we use the default destructor. */
    ~Default_configuration(){}
    
    double get_kp() const; /*!< Always returns DEFAULT_KP. */
    double get_kd() const; /*!< Always returns DEFAULT_KD. */
    double get_ki() const; /*!< Always returns DEFAULT_KI. */
    bool has_error() const; /*!< Always returns false. */
    std::string get_error() const; /*!< Always returns "no error". */
    
  };



};
