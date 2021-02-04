#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages = ['SAFMC-21-D2-Team2'],
    package_dir = {'': 'src'},
    install_requires = ['opencv-python']
)

setup(**setup_args)
