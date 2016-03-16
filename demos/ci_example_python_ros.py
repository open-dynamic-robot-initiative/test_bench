
# this script expects roscore to run and gains_kp, gains_kd and gains_ki written in the parameter server
# see: ci_example_ros_demo in this folder

from ci_example.pid import get_ros_params_pid

current_position = 1
current_velocity = 0.1
position_target = 0
delta_time = 0.001

# example of creation of PID reading from ros parameter server
print "pid reading from ros parameter:"
pid = get_ros_params_pid(verbose=True)
print pid
force = pid.compute(current_position,current_velocity,position_target,delta_time)
print "\tforce:",force

