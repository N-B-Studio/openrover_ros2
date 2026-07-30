from setuptools import find_packages, setup


package_name = 'ros_car_base'


setup(
    name=package_name,
    version='0.1.0',

    packages=find_packages(
        exclude=['test']
    ),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            [
                'resource/' + package_name,
            ],
        ),
        (
            'share/' + package_name,
            [
                'package.xml',
            ],
        ),
    ],

    install_requires=[
        'setuptools',
    ],

    zip_safe=True,

    maintainer='radxa',

    maintainer_email=
        'radxa@localhost',

    description=
        'ROS 2 UART base driver for the ROS car.',

    license='Apache-2.0',

    tests_require=[
        'pytest',
    ],

    entry_points={
        'console_scripts': [
            (
                'base_driver = '
                'ros_car_base.base_driver:main'
            ),
        ],
    },
)