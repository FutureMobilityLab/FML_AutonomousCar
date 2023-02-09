from setuptools import setup

package_name = 'csv_bag_parser'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='george',
    maintainer_email='georgetnhmartin@gmail.com',
    description='Exports a CSV to root folder of FML_AutonomousCar. Used primarily when replaying ros2 bags',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['parser = csv_bag_parser.csv_parser:main',
        ],
    },
)
