from setuptools import find_packages
from setuptools import setup

setup(
    name='ur_sim',
    version='0.0.0',
    packages=find_packages(
        include=('ur_sim', 'ur_sim.*')),
)
