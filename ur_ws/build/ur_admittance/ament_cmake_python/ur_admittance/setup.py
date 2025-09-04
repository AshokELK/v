from setuptools import find_packages
from setuptools import setup

setup(
    name='ur_admittance',
    version='0.0.1',
    packages=find_packages(
        include=('ur_admittance', 'ur_admittance.*')),
)
