import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'wardrone_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'missions'), glob('missions/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arnau',
    maintainer_email='arnau@avson.eu',
    description='Launch files, configuration, and mission files for the WarDrone',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
