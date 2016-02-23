#include "ros/ros.h"
#include "ros/master.h"
#include <memory>
#include <iostream>
#include <fstream>
#include "yaml-cpp/yaml.h"


#define DEFAULT_KP 1.0
#define DEFAULT_KD 1.0
#define DEFAULT_KI 1.0

#define ROSPARAM_KP "gains_kp"
#define ROSPARAM_KD "gains_kd"
#define ROSPARAM_KI "gains_ki"

using namespace std;

namespace ci_example {

  /*! virtual object describing the function concrete configuration objects should present */
  class Gains_configuration {
  public:
    virtual double get_kp()=0; /**< returns desired kp */
    virtual double get_kd()=0; /**< returns desired kd */
    virtual double get_ki()=0; /**< returns desired ki */
    /**
     * for enquiring in an error was encountered while reading the configuration
     * @see get_error()
     * @return true if an error has been encountered, false otherwise
     */
    virtual bool has_error()=0; 
    /**
     * returns error encountered when reading configuration
     * @see has_error()
     */
    virtual std::string get_error()=0; 
  };

  /*! Reading configuration from yaml file */
  class File_configuration : public Gains_configuration {
  public:
    /**
     * returns error encountered when reading configuration
     * @param yaml_file absolute path to configuration yaml file. The file is expected to have parameters "kp", "kd" and "ki"
     * @see has_error()
     */
    File_configuration(std::string yaml_file);
    double get_kp();
    double get_kd();
    double get_ki();
    bool has_error();
    std::string get_error();
  private:
    double kp,kd,ki;
    std::string error_message;
    bool error;
  };

  /*! Returning default configuration: kp: DEFAULT_KP, kd: DEFAULT_KD, ki: DEFAULT_KI */
  class Default_configuration : public Gains_configuration {
  public:
    double get_kp();
    double get_kd();
    double get_ki();
    bool has_error(); /** always returns false */
    std::string get_error(); /** always returns "no error" */
  };



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
    double get_kp();
    double get_kd();
    double get_ki();
    bool has_error();
    std::string get_error();
  private:
    double kp,kd,ki;
    std::string error_message;
    bool error;
  };


  /*! print values encapsulated by the provided configuration console on the standard output */
  void print_configuration(const std::shared_ptr<Gains_configuration> configuration);

  /*! simple 1D pid controller */
  class PID {

  public:
    PID(std::shared_ptr<Gains_configuration> configuration);
    /**
     * compute the force related to the pid controller. 
     * \warning this function is not stateless, as it performs integration. call reset_pid() to reset the integral part. 
     * @param position current position
     * @param velocity current velocity
     * @param position_target target position
     * @param delta_time time passed since last measurement. Used for integral computation
     * @return computed force
   */
    double compute(const double position, const double velocity, const double position_target, const double delta_time);
    /*! reset integral part of the PID*/
    void reset_integral();

  private:
    std::shared_ptr<Gains_configuration> configuration;
    double integral;

  };

  
  /**
   * convenience factory for getting default controller, i.e. same as PID(std::shared_ptr<Default_configuration> configuration)
   * @see Default_configuration
   */
  std::shared_ptr<PID> get_default_pid();



}
