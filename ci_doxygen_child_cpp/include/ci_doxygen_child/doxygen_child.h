/**
 * @file doxygen_child.h
 * @author Vincent Berenz
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellshaft.
 * @date 2019-05-22
 */

#include "ci_example/basic_pid.h"



namespace ci_doxygen_child {

  /* dummy class using the ci_example API */
  class Dummy_class {

  public:

    /** 
     * example of a function referring to ci_example API.
     * the documentation of this function should link with success to 
     * ci_example::PID. See CMakeLists.txt for details
     * @param pid PID controller
     */
    void dummy_function_using_ci_example_api (ci_example::PID &pid);

  };

}

