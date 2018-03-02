#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['wild_thumper'],
    package_dir={'': 'src'}
)

setup(**d)
