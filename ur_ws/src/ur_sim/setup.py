from setuptools import setup
import os
from glob import glob

package_name = 'ur_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include all URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        # Include all config files
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chintu',
    maintainer_email='chintu@todo.todo',
    description='Universal Robot simulation package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apply_force = ur_sim.apply_force:main',
        ],
    },
)
