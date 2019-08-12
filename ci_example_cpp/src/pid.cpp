#include "ci_example/pid.h"

namespace ci_example {

  PID::PID()
    : integral_(0) {
    configuration_ = new Default_configuration();
    private_configuration_ = true;
  }

  
  PID::PID(const Gains_configuration &configuration)
    :configuration_(&configuration),private_configuration_(false),integral_(0) {}

  
  PID::~PID(){
    if(private_configuration_){
      delete configuration_;
    }
  }

  
  double PID::compute( const double position,
		       const double velocity,
		       const double position_target,
		       const double delta_time ) {
    double position_error = position_target-position;
    integral_ += delta_time * position_error;
    double f = position_error*configuration_->get_kp()
      - velocity*configuration_->get_kd()
      + integral_*configuration_->get_ki();
    return f;
  }

  
  void PID::reset_integral(){
    this->integral_=0;
  }
  

  
  class Default_pid_factory {

  public:
    static std::vector< std::shared_ptr<Gains_configuration> > configs_;
    static std::vector< std::shared_ptr<PID> > controllers_;
    static PID& get(){
      std::shared_ptr<Gains_configuration> configuration(new Default_configuration());
      std::shared_ptr<PID> controller(new PID(*configuration));
      configs_.push_back(configuration);
      controllers_.push_back(controller);
      return *controller;
    }
    
  };

  
  std::vector< std::shared_ptr<Gains_configuration> > Default_pid_factory::configs_;
  std::vector< std::shared_ptr<PID> > Default_pid_factory::controllers_;


  
  PID& get_default_pid(){
    return Default_pid_factory::get();
  }

  

}
