#!/usr/bin/env python

import unittest
import rospy
import ci_example_python.pid as PID


# see CMakeLists.txt in root folder to see how to activate these tests
# for continuous integration

## tests for creation of instances of PID using configuration from ros parameter server
class PID_ROS_TESTCASE(unittest.TestCase):

    # tests here assumes roscore is running
    # this is ok because tests here are not started directly, but
    # via test_pid_ros.test, which is a ros launch file which starts
    # roscore. See CMakeLists.txt file in root folder to see activation of these tests

    ## writing gains values in ros parameter server
    def setUp(self):
        rospy.init_node('test_pid_ros',anonymous=True)
        rospy.set_param("kp",1)
        rospy.set_param("kd",1)
        rospy.set_param("ki",1)
    
    ## deleting the file created above when we leave the tests
    def tearDown(self):
        rospy.delete_param("kp")
        rospy.delete_param("kd")
        rospy.delete_param("ki")

    ## testing creating a pid controller ros servo
    def test_ros_param_server_factory(self):
        pid = PID.get_ros_params_pid(verbose=False)
        gains = pid.get_gains()
        self.assertEqual(gains["kp"],1)
        self.assertEqual(gains["kd"],1)
        self.assertEqual(gains["ki"],1)



if __name__ == "__main__":
    import rostest
    rostest.rosrun("ci_example_python","test_pid_ros",PID_ROS_TESTCASE)
