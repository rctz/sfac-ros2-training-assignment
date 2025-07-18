from setuptools import find_packages, setup

package_name = 'robot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        ('share/' + package_name + '/launch', ['launch/nav2_only.launch.py', 'launch/nav2_node.launch.py']),
        ('share/' + package_name + '/config', ['config/nav2_params.yaml', 'config/model03.yaml', 'config/model03.pgm']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='palmzaawow',
    maintainer_email='palmrock4@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
