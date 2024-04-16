from setuptools import find_packages, setup

package_name = 'turin_tcp'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rui',
    maintainer_email='rui@todo.todo',
    description='TODO: This package is for setting up connection between ROS2 and Turin Robot through TCP net command',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tcp_connection_node = turin_tcp.tcp_connection_node:main',
            'joint_state_publisher = turin_tcp.joint_state_publisher:main',
            'joint_state_listener = turin_tcp.joint_state_listener:main',
        ],
    },
)
