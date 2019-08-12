#pragma once

#include "ci_example/gains_configuration.h"
#include "ros/ros.h"
#include "ros/master.h"


#define ROSPARAM_KP "gains_kp"
#define ROSPARAM_KD "gains_kd"
#define ROSPARAM_KI "gains_ki"



namespace ci_example {


  /*! Read gains configuration from the ros parameter server*/
  class RosParameters_configuration : public Gains_configuration {

  public:

    /**
     * Attempt to get the gains from the parameter server ("gains_kp","gains_kd","gains_ki" parameters)
     * If roscore is running, calls to this constructor will be blocking until all the gains are read
     * or roscore is turned off. If roscore is turned off before gains are read, has_error() will return true
     * @see has_error()
     */
    RosParameters_configuration();

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
