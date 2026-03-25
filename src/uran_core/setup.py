from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['uran_core'],
    package_dir={'': '.'},
)

setup(**setup_args)
