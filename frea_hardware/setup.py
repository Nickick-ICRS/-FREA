from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['frea_hardware'],
    scripts=['nodes/frea_head'],
    package_dir={'': 'python/src'}
)

setup(**d)
