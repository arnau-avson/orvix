from setuptools import find_packages, setup

package_name = 'wardrone_vio'

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
    description='Visual-Inertial Odometry integration for GPS-denied navigation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vio_bridge_node = wardrone_vio.vio_bridge_node:main',
            'vio_evaluator_node = wardrone_vio.vio_evaluator_node:main',
        ],
    },
)
