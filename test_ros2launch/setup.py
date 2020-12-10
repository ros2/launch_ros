from setuptools import find_packages
from setuptools import setup

package_name = 'test_ros2launch'

setup(
    name=package_name,
    version='0.10.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'launch_ros',
        'ros2launch',
    ],
    zip_safe=True,
    maintainer='Ted Kern',
    maintainer_email='ted.kern@canonical.com',
    download_url='https://github.com/ros2/launch/releases',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Tests for ROS specific launch cli.',
    long_description=(
        'This package provides tests for the ROS specific '
        'launch cli.'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
)
