#!/usr/bin/env python

# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['glenn_simulation',
              'glenn_simulation.glenn_description',
              'glenn_simulation.randomize_arena'],
    package_dir={'': 'src'},
)

setup(**setup_args)
