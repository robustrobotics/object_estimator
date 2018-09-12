#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['name']        = 'object_estimator',
d['version']     = '0.0.1'
d['description'] = 'Object internal state estimator using sensory signals'
d['author']      = 'Daehyung Park'
d['packages']    = ['object_estimator']
d['package_dir'] = {'': 'src'}

setup(**d)
