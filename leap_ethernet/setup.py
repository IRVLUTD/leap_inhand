from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    py_modules=['client', 'keyboard_control'],
    package_dir={'': 'src'}
)

setup(**d)
