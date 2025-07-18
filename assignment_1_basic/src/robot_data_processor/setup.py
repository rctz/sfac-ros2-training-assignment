from setuptools import find_packages, setup

package_name = 'robot_data_processor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],

    install_requires=['setuptools'
        'rosidl_default_generators',
        'rosidl_runtime_py',
    ],
    zip_safe=True,
    maintainer='palmzaawow',
    maintainer_email='palmrock4@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_processor = robot_data_processor.data_processor:main',
        ],
    },
)
