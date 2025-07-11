from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_sensor_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #(os.path.join('share',package_name,'launch'),glob('launch/*.launch.py')),
        ('share/robot_sensor_publisher/launch', ['launch/robot_system.launch.py']),
    ],
    install_requires=['setuptools'
        'rosidl_default_generators',
        'rosidl_runtime_py',
    ],
    zip_safe=True,
    maintainer='palmzaawow',
    maintainer_email='palmzaawow@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_publisher = robot_sensor_publisher.sensor_publisher:main',
        ],
    },
 \
)
