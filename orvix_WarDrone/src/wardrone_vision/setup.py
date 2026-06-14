from setuptools import find_packages, setup

package_name = 'wardrone_vision'

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
    description='Camera pipeline, YOLO detection, and object tracking for the WarDrone',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = wardrone_vision.camera_node:main',
            'detector_node = wardrone_vision.detector_node:main',
            'tracker_node = wardrone_vision.tracker_node:main',
        ],
    },
)
