The header files are not directly in /include/ but in /include/<catkin_package_name>/ because it clarified include in code in other catkin packages.
e.g.:

#include <ci_example/gains_configuration.h> 

is much clearer than:

#include <gains_configuration.h>  // where does gains_configuration.h comes from ?
