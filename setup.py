#!/usr/bin/env python
## Authors: Daehyung Park
## MIT, 2018
##
## Distributed under the MIT Software License.

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['name']        = 'object_estimator'
d['version']     = '0.0.1'
d['description'] = 'Object internal state estimator using sensory signals'
d['author']      = 'Daehyung Park'
d['packages']    = ['object_estimator']
d['package_dir'] = {'': 'src'}

setup(**d)
