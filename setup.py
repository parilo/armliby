#!/usr/bin/env python

from setuptools import setup

setup(
    name='armliby',
    version='1.0',
    description='Robotic arm control library',
    author='Anton Pechenko',
    author_email='anton@my_family_name_dot_org',
    url='https://github.com/parilo/pyarmlib',
    packages=['armliby'],
    install_requires=[
        'open3d',
        'websockets',
        'yourdfpy',
        'pytorch-kinematics',
    ],
) 
