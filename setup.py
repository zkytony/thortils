#!/usr/bin/env python

# Please make sure this is correct.
AI2THOR_VERSION = '3.3.4'

from setuptools import setup, find_packages

setup(name='thortils',
      packages=find_packages(),
      version='0.1',
      description='Code related to Ai2-Thor. Try to do one thing once.',
      python_requires='>3.6',
      install_requires=[
          'numpy',
          'matplotlib',
          'ai2thor=={}'.format(AI2THOR_VERSION)
      ],
      author='Kaiyu Zheng',
      author_email='kaiyutony@gmail.com')
