#from setuptools import setup
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
d = generate_distutils_setup(packages=["niryo_pick_and_place"],
                             package_dir={"": "src"})

setup(**d)
