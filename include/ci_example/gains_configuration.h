#include <memory>
#include <iostream>
#include "yaml-cpp/yaml.h"

#define DEFAULT_KP 1.0
#define DEFAULT_KD 1.0
#define DEFAULT_KI 1.0


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
  class File_configuration : Gains_configuration {
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
  class Default_configuration : Gains_configuration {
  public:
    double get_kp();
    double get_kd();
    double get_ki();
    bool has_error(); /** always returns false */
    std::string get_error(); /** always returns "no error" */
  };

  /*! print values encapsulated by the provided configuration console on the standard output */
  void console_configuration(const std::shared_ptr<Gains_configuration> configuration);



}
