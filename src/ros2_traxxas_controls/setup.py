from setuptools import setup
import os
from glob import glob
package_name = 'ros2_traxxas_controls'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='george',
    maintainer_email='georgetnhmartin@gmail.com',
    description='This Package contains all nodes necessary for low level controls to a PWM driven Traxxas Vehicle',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_driver = ros2_traxxas_controls.ros2_traxxas_motor_commands:main',
            'keyboard_teleop_hold = ros2_traxxas_controls.traxxas_teleop_hold:main'
        ],
    },
)
