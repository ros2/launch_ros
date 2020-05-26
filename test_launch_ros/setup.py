from setuptools import find_packages
from setuptools import setup

package_name = 'test_launch_ros'

setup(
    name=package_name,
    version='0.10.2',
    packages=find_packages(exclude=['test']),
    install_requires=[
        'setuptools',
        'demo_nodes_py',
        'launch_ros',
        'pyyaml',
    ],
    zip_safe=True,
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    author='William Woodall',
    author_email='william@osrfoundation.org',
    maintainer='William Woodall',
    maintainer_email='william@osrfoundation.org',
    url='https://github.com/ros2/launch',
    download_url='https://github.com/ros2/launch/releases',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Tests for ROS specific extensions to `launch`.',
    long_description=(
        'This package provides tests for the ROS specific '
        'extensions to the launch package.'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
)
