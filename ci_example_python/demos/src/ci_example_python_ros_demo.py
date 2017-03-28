#! /bin/sh

# this demo can not be directly run, it needs roscore up and some
# parameters put in ros parameter server.
# this is why the continuous integration will run it using the script ci_example_ros_demo
# in the folder above the one this script is in

""":"
exec python $0 ${1+"$@"}
"""

from ci_example_python.pid import get_ros_params_pid

current_position = 1
current_velocity = 0.1
position_target = 0
delta_time = 0.001

# example of creation of PID using default gains
print "pid using ros parameter server"
pid = get_ros_params_pid()
print pid
force = pid.compute(current_position,current_velocity,position_target,delta_time)
print "force:",force

