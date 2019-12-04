#pragma once

#include "ci_example_cpp/default_configuration.hpp"
#include <memory>
#include <vector>

namespace ci_example_cpp {


  /*! @brief Simple 1D pid controller. */
  class PID {

  public:
    
    /**
     * @brief Construct a default PID object using the DefaultConfiguration
     */
    PID();
    /**
     * @brief Construct a new PID object using a user provided configuration.
     * 
     * @param configuration
     */
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
