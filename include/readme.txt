The header files are not directly in /include/ but in /include/<catkin_package_name>/ because it clarified include in code in other catkin packages.
e.g.:

#include <tests_demo_example/youpi.hpp> 

is much clearer than:

#include <youpi.hpp>  // where does youpi.hpp comes from ?
