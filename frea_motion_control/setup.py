from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['frea_motion_control'],
    scripts=['scripts/base_link_chassis_publisher']
    package_dir={'': 'python/src'}
)

setup(**d)
