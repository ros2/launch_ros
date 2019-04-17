#!/usr/bin/env python

import glob

from setuptools import setup

package_name = 'apex_launchtest_ros'

setup(
    name=package_name,
    version='0.1',
    description='Apex integration test runner and utilities',

    author='Pete Baughman',
    author_email='pete.baughman@apex.ai',

    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/apex_launchtest_ros']),
        ('lib/' + package_name, glob.glob('example_nodes/**')),
        ('share/' + package_name + '/examples', glob.glob('examples/[!_]**')),
    ],
    packages=[
        'apex_launchtest_ros',
    ],
    tests_require=['pytest'],
    zip_safe=True,
)
