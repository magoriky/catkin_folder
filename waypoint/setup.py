from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup_args = generate_distutils_setup(
    packages=['waypoint'],
    package_dir={'': 'src'},
    install_requires=['scipy']
)

setup(**setup_args)