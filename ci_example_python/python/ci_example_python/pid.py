#! /usr/bin/python

##@package ci_example_python
# 
# @file pid.py
# @author Vincent Berenz
# @author Maximilien Naveau
# license License BSD-3-Clause
# @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
# @date 2019-05-22
# 
# @brief Simple 1D PID controller, along with convenience factory.
#


import os


## 
# Configuration object with default values for kp, kd and ki
# can be used as input argument to create an instance of PID
# @see PID
class Default_configuration:
    ## proportional gain
    kp=1
    ## derivative gain
    kd=1
    ## integral gain
    ki=1

## Configuration: name of ros parameter server keys in which gains should be stored
class ROS_configuration:
    ## key for reading kp gain
    ROSPARAM_KP = "kp"
    ## key for reading kd gain
    ROSPARAM_KD = "kd"
    ## key for reading ki gain
    ROSPARAM_KI = "ki"

## Configuration: path to default configuration file, relative to the pid package
class Config_file_configuration:
    ## relative path to the default configuration fole
    relative_path = os.path.join("..", "..", "config", "test_pid_gains.yaml")

## code for simple 1D PID controller
class PID:
    """
    Simple 1D PID controller
    """
    
    ##
    #@param configuration any object with "kp", "kd" and "ki" attributes (as float)
    def __init__(self,configuration):
        self._configuration = configuration
        self._integral = 0

    ##
    #@return dictionary of gains, keys: "kp", "kd" and "ki"
    def get_gains(self):
        return {"kp":self._configuration.kp,"kd":self._configuration.kp,"ki":self._configuration.ki}

    ## resert integral part of the PID 
    def reset_integral(self):
        self._integral = 0

    ##
    #compute the force related to the pid controller. 
    #this function is not stateless, as it performs integration. call reset_pid() to reset the integral part. 
    #@param position current position
    #@param velocity current velocity
    #@param position_target target position
    #@param delta_time time passed since last measurement. Used for integral computation
    #@return computed force
    def compute(self,position,velocity,position_target,delta_time):
        position_error = position_target - position
        self._integral += delta_time * position_error
        return position_error*self._configuration.kp-velocity*self._configuration.kd+self._integral*self._configuration.ki
    ## Convert the object into a string
    def __str__(self):
        return "PID controller: kp:"+str(self._configuration.kp)+" kd:"+str(self._configuration.kd)+" ki:"+str(self._configuration.ki)



# convenience function for reading pid configuration from yaml file
def _read_yaml_config_file(file_path):
    # importing yaml and reading yaml file
    import yaml
    with open(file_path,"r") as f:
        config_dict = yaml.load(f)
    # checking the yaml file had the excepted entries
    expected_attributes = ["kp","kd","ki"]
    for attribute in expected_attributes:
        if not attribute in config_dict.keys():
            raise Exception("Configuration file "+str(file_path)+" is expected to have the "+str(attribute)+" entry")
    # creating a config object with correct attributes
    class config(object): pass
    c = config();
    for attribute in expected_attributes:
        try : 
            setattr(config,attribute,float(config_dict[attribute]))
        except Exception as e: 
            raise Exception("failed to convert "+attribute+"("+str(config_dict[attribute])+") to float (file: "+str(file_path)+")")
    # constructing and returning controller
    return PID(config)


### factories for getting already configured pid controllers

## 
#return PID based on default configuration
#@see PID
#@see Default_configuration
def get_default_pid():
    return PID(Default_configuration)

##
#return PID based on gains read from the ROS parameter server. 
#assumes roscore is running and suitable parameters have been written in the server.
#@param verbose if true, print the ros parameters it reads to standard output
#@see PID
#@see ROS_configuration
def get_ros_params_pid(verbose=True):
    # importing ros and checking roscore is running
    import rospy
    if rospy.is_shutdown():
        raise Exception("failed to read ros parameters: ros is shutdown")
    # placeholder for the config
    class config:
        kp=None
        kd=None
        ki=None
    # reading the gains from ros parameter server
    parameters = [ROS_configuration.ROSPARAM_KP,ROS_configuration.ROSPARAM_KD,ROS_configuration.ROSPARAM_KI]
    gains = ["kp","kd","ki"]
    # if requested, printing the parameters it is about to read
    if verbose:
        print "reading ros parameters: "+", ".join(parameters)
    for parameter,gain in zip(parameters,gains):
        if not rospy.has_param(parameter):
            raise Exception("ros parameter server does not have the requested parameter: "+str(parameter)+" (current parameters: "+", ".join(rospy.get_param_names())+")")
        try:
            value = rospy.get_param(parameter)
            setattr(config,gain,value)
        except Exception as e:
            raise Exception("failed to read ros parameter "+str(parameter)+": "+str(e))
    # constructing and returning controller    
    return PID(config)

##
#return PID based on gains read from default configuration file 
#Path to configuration file relative to the script where this function is 
#defined is specified in the Config_file_configuration object.
#@param config_file_path if None, uses default config file (in config folder), else used specified path
#@param verbose if True, print path to config file used to standard output
#@see PID
#@see Config_file_configuration
def get_config_file_pid(config_file_path=None,verbose=True):
    if config_file_path is None:
        # getting abs path to this script
        abs_path_script = os.path.realpath(__file__)
        # getting name of this file
        script = os.path.basename(abs_path_script)
        # getting abs path of folder in which this script is
        abs_path =  abs_path_script[:-len(script)]
        # getting abs path to config file
        abs_path_config = os.path.abspath(abs_path+os.sep+Config_file_configuration.relative_path)
    else : abs_path_config = config_file_path
    # checking file exists
    if not os.path.isfile(abs_path_config):
        raise Exception("failed to find configuration file: "+str(abs_path_config))
    # printing path to config file if asked
    if verbose:
        print "reading pid gains from: ",os.path.abspath(abs_path_config)
    # constructing and returning the controller
    return _read_yaml_config_file(abs_path_config)
