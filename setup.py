from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ci_example'],
    package_dir={'': 'python'},
)

setup(**d)
