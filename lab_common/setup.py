#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = ['lab_ros_common']
d['package_dir'] = {'': 'python_src'}

setup(**d)
