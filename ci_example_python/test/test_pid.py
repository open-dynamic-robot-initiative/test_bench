"""@package
Unit-tests for PID and related factories
"""


import unittest
import yaml
import os
import ci_example_python.pid as PID


# see CMakeLists.txt in root folder to see how to activate these tests
# for continuous integration



## set of unit-tests for PID and related factories
class PID_TESTCASE(unittest.TestCase):

    YAML_CONFIG_FILE = "pid_config_test.yaml"

    ## 
    # testing the function get_config_file_pid will require a yaml config file
    # creating this file here, which will be called before running all tests
    def setUp(self):
        with open(self.YAML_CONFIG_FILE,"w+") as f:
            f.write(yaml.dump(PID.Default_configuration))
    
    ## deleting the file created above when we leave the tests
    def tearDown(self):
        os.remove(self.YAML_CONFIG_FILE)

    ## testing creating a pid controller from file works as expected
    def test_config_file_factory(self):
        pid = PID.get_config_file_pid(config_file_path=self.YAML_CONFIG_FILE,verbose=False)
        gains = pid.get_gains()
        self.assertEqual(gains["kp"],PID.Default_configuration.kp)
        self.assertEqual(gains["kd"],PID.Default_configuration.kd)
        self.assertEqual(gains["ki"],PID.Default_configuration.ki)

    ## testing creating a pid controller from default config file 
    def test_config_file_factory(self):
        pid = PID.get_config_file_pid(verbose=False)
   
    ## testing creation using default config
    def test_default_factory(self):
        pid = PID.get_default_pid()
        gains = pid.get_gains()
        self.assertEqual(gains["kp"],PID.Default_configuration.kp)
        self.assertEqual(gains["kd"],PID.Default_configuration.kd)
        self.assertEqual(gains["ki"],PID.Default_configuration.ki)

    ## testing creating a pid controller from a non existing file raises an exception
    def test_exception_on_non_existing_config_file(self):
        with self.assertRaises(Exception):
            pid = PID.get_config_file_pid(config_file_path="non_existing_path",verbose=False)
        
    ## testing integral integrates, except if ki is zero
    def test_integral(self):
        class config:
            kp,kd,ki = 1,1,1
        position = 1
        velocity = 1
        position_target = 2
        delta_time = 0.1
        pid = PID.PID(config)
        force_1 = pid.compute(position,velocity,position_target,delta_time)
        force_2 = pid.compute(position,velocity,position_target,delta_time)
        self.assertNotEqual(force_1,force_2)
        config.ki = 0
        force_3 = pid.compute(position,velocity,position_target,delta_time)
        force_4 = pid.compute(position,velocity,position_target,delta_time)
        self.assertEqual(force_3,force_4)


        # testing force is zero if already at target
        class config:
            kp,kd,ki = 1,1,1
        position = 1
        velocity = 0
        position_target = 1
        delta_time = 0.1
        pid = PID.PID(config)
        force = pid.compute(position,velocity,position_target,delta_time)
        self.assertEqual(force,0)

        # testing the controller pushes in the right direction
        class config:
            kp,kd,ki = 1,1,0
        position = 0
        velocity = 0
        target_position = 1
        delta_time = 0.1
        pid = PID.PID(config)
        force = pid.compute(position,velocity,target_position,delta_time)
        self.assertTrue(force>0)
        target_position = -1
        force = pid.compute(position,velocity,target_position,delta_time)
        self.assertTrue(force<0)
        target_position = 0
        velocity=1.0
        force = pid.compute(position,velocity,target_position,delta_time)
        self.assertTrue(force<0)
