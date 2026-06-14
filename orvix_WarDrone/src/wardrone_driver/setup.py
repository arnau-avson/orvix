from setuptools import find_packages, setup

package_name = 'wardrone_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arnau',
    maintainer_email='arnau@todo.todo',
    description='MAVSDK-to-ROS 2 bridge for PX4 flight controller communication',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mavsdk_bridge_node = wardrone_driver.mavsdk_bridge_node:main',
        ],
    },
)
