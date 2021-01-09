from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['frea_motion_control'],
    scripts=['scripts/train_motion_controller'],
    package_dir={'': 'python/src'}
)

setup(**d)
