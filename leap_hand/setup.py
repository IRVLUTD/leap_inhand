from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# This setup treats the 'scripts' directory as the primary package.
d = generate_distutils_setup(
    packages=['leap_hand_utils'],
    package_dir={'': 'scripts'}
)

setup(**d)