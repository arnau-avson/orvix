from setuptools import find_packages, setup

package_name = 'wardrone_navigation'

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
    description='Waypoint navigation and safety monitoring for the WarDrone',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_navigator_node = wardrone_navigation.waypoint_navigator_node:main',
            'safety_monitor_node = wardrone_navigation.safety_monitor_node:main',
            'obstacle_detector_node = wardrone_navigation.obstacle_detector_node:main',
            'obstacle_avoidance_node = wardrone_navigation.obstacle_avoidance_node:main',
        ],
    },
)
