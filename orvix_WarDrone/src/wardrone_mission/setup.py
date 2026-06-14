from setuptools import find_packages, setup

package_name = 'wardrone_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arnau',
    maintainer_email='arnau@avson.eu',
    description='Mission state machine controller for the WarDrone',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_controller_node = wardrone_mission.mission_controller_node:main',
        ],
    },
)
