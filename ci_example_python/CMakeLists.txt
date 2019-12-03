cmake_minimum_required(VERSION 2.8.3)
project(ci_example_python) 

# This allows you use the catkin cmake macros
find_package(catkin REQUIRED COMPONENTS mpi_cmake_modules)

# the only annoying thing with python is the need to have python libraries
# either installed or with their path added to PYTHONPATH
# catkin helps with this using the following approach:
# 1. each (python) catkin package contains a setup.py file which specify which
#    package/module it exports
# 2. the CMakeLists.txt files (e.g. the file your are reading) calls
#    "catkin_python_setup()"
# 3. when catkin_make is called from the catkin workspace root instructions for
#    updating the python path are created
# 4. after "source <path_to>/workspace/devel/setup.bash" is called, all
#    package/modules exported by all catkin packages are added to the PYTHONPATH.
# more information can be found here: http://docs.ros.org/jade/api/catkin/html/user_guide/setup_dot_py.html
find_package(catkin REQUIRED COMPONENTS
    mpi_cmake_modules)

catkin_python_setup()

# In order to define the CATKIN variables such as CATKIN_PACKAGE_SHARE_DESTINATION
# used in the build_doxygen_documentation() macros from the mpi_cmake_modules
# one need to declare this as a catkin package.
catkin_package()

####################################################################################
#                           activating unit tests                                  #
####################################################################################
# to run the tests:
# 1. call "catkin_make" from root of catkin workspace
# 2. call "source ./devel/setup.bash"
# 3. call "catkin_make run_tests"

if(CATKIN_ENABLE_TESTING)

  # regular unit tests (test is the name of the folder in which all the tests are)
  catkin_add_nosetests(tests)

  # test that requires roscore to run:
  # tests are started via *.test files which are ros launch files
  # consequently, these tests may run under the assumption roscore is running
  find_package(rostest REQUIRED)
  add_rostest(tests/test_pid_ros.test)


endif()

####################################################################################
#                           example of python demo                                 #
####################################################################################

# what is the difference between a demo and unit tests ?
# unit tests are there to check availability of low grain functionalities
# demos are higher level and can be used for testing as well, but their main
# purpose are to provide users with clear example of usage. 

# the continuous integration script will run all the executables found in 
# the demos folder. So just put your demo there. See demos/ci_example_python_demo as 
# an example of python file transformed into an executable.
# if your demo requires some setup, you may create a bash script that do the setup and
# call your demo python script. See demos/ci_example_ros_demo as example
# note: the python script will call first "catkin_make" and "source ./devel/setup.bash"
# to ensure PYTHONPATH is set correctly. See explanations at beginning of this file.

##########################
# building documentation #
##########################
build_doxygen_documentation()
