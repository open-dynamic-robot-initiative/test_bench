#!/usr/bin/env python

""" @namespace Demos of the ci_example_python.pid.PID controller.

@file demo_pid.py
@copyright Copyright (c) 2017-2019,
           New York University and Max Planck Gesellschaft,
           License BSD-3-Clause

@example demo_pid.py
In order to run this demos the `ci_example_python/python/ci_example_python` path
needs to be added in the `PYTHONPATH`. This is done after:
- 1. Building the workspace by executing `catkin_make` from the root of the
     catkin workspace.
- 2. "source ./devel/setup.bash" is called from the root of the catkin
     workspace.
- 3. Run the demo by either:
    - 3.1. `rosrun ci_example_python demo_pid.py`
    - 3.3. `cd /path/to/ci_example_python/` ; `./demos/demo_pid.py`

Notice the use of the double ## for the comments. This allow Doxygen
to parse you code and for you to explain in details the content of the demo.
"""


# Python 3 compatibility, has to be called just after the hashbang.
from __future__ import print_function, division
from ci_example_python.pid import PID, get_default_pid, get_config_file_pid


if __name__ == "__main__":
    
    ## Input position for the pid controller
    current_position = 1
    ## Input velcoity for the pid controller
    current_velocity = 0.1
    ## Input target position for the pid controller
    position_target = 0
    ## Input smapling period for the pid controller
    delta_time = 0.001

    # basic example of PID usage
    print("basic pid usage:")
    
    class Configuration:
        """ This a small shell that contains the PID gains.
        It mocks the load of a yaml files.
        Attributes:
            kp: Proportional gain.
            kd: Derivative gain.
            ki: Integral gain.
        """
        
        def __init__(self, kp, kd, ki):
            """  Create the 3 gains
            Args:
                kp: Proportional gain.
                kd: Derivative gain.
                ki: Integral gain.
            """
            self.kp = kp
            self.kd = kd
            self.ki = ki

    ## Configuration of the PID controller
    config = Configuration(1,1,1)

    ## Example of creation of the PID using a config class.
    pid = PID(config)
    print(pid)
    ## Compute the control from the current input.
    force = pid.compute(current_position,current_velocity,position_target,delta_time)
    print("force:",force)

    ## Example of creation of the PID using default gains.
    print ("pid using default gains:")
    pid = get_default_pid()
    print (pid)

    force = pid.compute(current_position,current_velocity,position_target,delta_time)
    print ("force:",force)

    # Example of creation of the PID using config files read from config file.
    print ("pid using gains read from config file:")
    pid = get_config_file_pid(verbose=True)
    print (pid)

    force = pid.compute(current_position,current_velocity,position_target,delta_time)
    print ("force:",force)
