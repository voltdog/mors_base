## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

a = generate_distutils_setup(
    packages=['mors_sim', 'lcm_msgs2'],
    package_dir={'': 'scripts'}
    )

setup(**a)