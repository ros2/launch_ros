import glob

from setuptools import find_packages
from setuptools import setup

setup(
    name='launch_testing_ros',
    version='0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/launch_testing_ros']),
        ('lib/launch_testing_ros', glob.glob('example_nodes/**')),
        ('share/launch_testing_ros/examples', glob.glob('examples/[!_]**')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Pete Baughman',
    author_email='pete.baughman@apex.ai',
    maintainer='Pete Baughman',
    maintainer_email='pete.baughman@apex.ai',
    url='https://github.com/ros2/launch_ros',
    download_url='https://github.com/ros2/launch_ros/releases',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Test the output of a ROS node.',
    long_description='A package providing utilities for writing ROS2 enabled launch tests.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
)
