#!/usr/bin/env python

from setuptools import setup, find_packages
import os
ABS_PATH = os.path.abspath(os.path.dirname(__file__))
with open(os.path.join(ABS_PATH, "AI2THOR_VERSION")) as f:
    AI2THOR_VERSION = f.readlines()[0].strip()

setup(name='thortils',
      packages=find_packages(),
      version='0.1',
      description='Code related to Ai2-Thor. Try to do one thing once.',
      python_requires='>3.6',
      install_requires=[
          'numpy',
          'matplotlib',
          'ai2thor=={}'.format(AI2THOR_VERSION),
          'open3d==0.13.0',
          'tqdm'
      ],
      license='MIT',
      author='Kaiyu Zheng',
      author_email='kaiyutony@gmail.com')
