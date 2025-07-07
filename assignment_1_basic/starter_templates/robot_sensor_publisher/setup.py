from setuptools import find_packages, setup

package_name = 'robot_sensor_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # TODO: Add launch files installation
        # ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@example.com',
    description='ROS2 package for publishing simulated sensor data',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # TODO: Add your executable here
            # 'sensor_publisher = robot_sensor_publisher.sensor_publisher:main',
        ],
    },
)
