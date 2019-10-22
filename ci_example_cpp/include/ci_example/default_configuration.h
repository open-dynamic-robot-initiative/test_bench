#pragma once

#include "ci_example/gains_configuration.h"


#define DEFAULT_KP 1.0
#define DEFAULT_KD 1.0
#define DEFAULT_KI 1.0


namespace ci_example {

  /*! Returning default configuration: kp: DEFAULT_KP, kd: DEFAULT_KD, ki: DEFAULT_KI */
  class Default_configuration : public Gains_configuration {
    
  public:

    ~Default_configuration(){}
    
    double get_kp() const;
    double get_kd() const;
    double get_ki() const;
    bool has_error() const; /** always returns false */
    std::string get_error() const; /** always returns "no error" */
    
  };



};
