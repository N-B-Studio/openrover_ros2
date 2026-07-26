from setuptools import find_packages, setup
import os

from glob import glob
from setuptools import find_packages, setup


package_name = 'robot_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join(
                'share',
                package_name,
                'launch'
            ),
            glob('launch/*.launch.py')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pleycothh',
    maintainer_email='pleycothh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'velocity_publisher = robot_tutorial.velocity_publisher:main',
            'pose_subscriber = robot_tutorial.pose_subscriber:main',
            'turtle_controller = robot_tutorial.turtle_controller:main',
            'callback_demo = robot_tutorial.callback_demo:main',
            'service_server = robot_tutorial.service_server:main',
            'service_client = robot_tutorial.service_client:main',
            'action_server = robot_tutorial.action_server:main',
            'serial_base_node = robot_tutorial.serial_base_node:main',
        ],
    },
)
