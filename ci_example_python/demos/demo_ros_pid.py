#!/usr/bin/env python


""" @namespace Demos of the ci_example_python.pid.PID controller using ROS param.

@file demo_ros_pid.py
@copyright Copyright (c) 2017-2019,
           New York University and Max Planck Gesellschaft,
           License BSD-3-Clause

@example demo_ros_pid.py
In order to run this demos the `ci_example_python/python/ci_example_python` path
needs to be added in the `PYTHONPATH`. This is done after:
- 1. "catkin_make" is called from the root of the catkin workspace
- 2. "source ./devel/setup.bash" is called from the root of the catkin workspace
- 3. roscore is called in an additional terminal
- 4. Finally run `rosrun ci_example_python demo_ros_pid.py`

Notice the use of the double ## for the comments. This allow Doxygen
to parse you code and for you to explain in details the content of the demo.
"""


# Python 3 compatibility, has to be called just after the hashbang.
from __future__ import print_function, division
import rospy
from ci_example_python.pid import get_ros_params_pid


if __name__ == "__main__":
    
    # here we set the parameters to ROS.
    rospy.set_param('kp', 1.0)
    rospy.set_param('kd', 1.0)
    rospy.set_param('ki', 1.0)

    ## Input position for the pid controller
    current_position = 1
    ## Input velcoity for the pid controller
    current_velocity = 0.1
    ## Input target position for the pid controller
    position_target = 0
    ## Input smapling period for the pid controller
    delta_time = 0.001

    print ("pid using ros parameter server")

    ## Example of creation of PID using default gains.
    pid = get_ros_params_pid()
    print (pid)

    ## Compute the control from the current input.
    control = pid.compute(current_position,current_velocity,position_target,delta_time)
    print ("control: ",control)
