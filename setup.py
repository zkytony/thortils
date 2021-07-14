#!/usr/bin/env python

from setuptools import setup, find_packages
from thortils import AI2THOR_VERSION

setup(name='thortils',
      packages=find_packages(),
      version='0.1',
      description='Code related to Ai2-Thor. Try to do one thing once.',
      python_requires='>3.6',
      install_requires=[
          'ai2thor=={}'.format(AI2THOR_VERSION)
      ],
      author='Kaiyu Zheng',
      author_email='kaiyutony@gmail.com')
