from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup_args = generate_distutils_setup(
    packages=['gps_collector'],
    package_dir={'': 'src'},
    install_requires=['paho-mqtt']
)

setup(**setup_args)