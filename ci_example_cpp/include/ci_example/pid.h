#pragma once

#include "ci_example/default_configuration.h"
#include <memory>
#include <vector>

namespace ci_example {


  /*! simple 1D pid controller */
  class PID {

  public:
    
    PID();
    PID(const Gains_configuration& configuration);
    ~PID();
    
    /**
     * compute the force related to the pid controller. 
     * \warning this function is not stateless, as it performs integration. 
     * Call reset_pid() to reset the integral part. 
     * @param position current position
     * @param velocity current velocity
     * @param position_target target position
     * @param delta_time time passed since last measurement. Used for integral computation
     * @return computed force
   */
    double compute( const double position,
		    const double velocity,
		    const double position_target,
		    const double delta_time );
    
    /*! reset integral part of the PID*/
    void reset_integral();

  private:
    
    const Gains_configuration *configuration_;
    bool private_configuration_;
    double integral_;

  };

  
  /**
   * convenience factory for getting default controller,
   *  i.e. same as PID(std::shared_ptr<Default_configuration> configuration)
   * @see Default_configuration
   */
  PID& get_default_pid();


}
