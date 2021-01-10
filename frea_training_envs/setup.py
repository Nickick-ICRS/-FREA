from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['frea_training_envs'],
    package_dir={'': 'python/src'}
)

setup(**d)
