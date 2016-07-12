from setuptools import setup, find_packages
from catkin_pkg.python_setup import generate_distutils_setup

requirements = [line.strip() for line in open("requirements.txt")]

d = generate_distutils_setup()

d['name'] = "graspit_commander"
d['description'] = "Python interface for GraspIt! simulator"
d['packages'] = find_packages()


setup(**d)

