import os
from glob import glob
from setuptools import setup

package_name = 'gps_data_manipulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('data/', glob(os.path.join('data','*csv'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aneesh Chodisetty',
    maintainer_email='ch.aneesh1996@gmail.com',
    description='A package to publish, subscribe and manipulate gps data.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_read = gps_data_manipulation.data_read:main',
            'data_process = gps_data_manipulation.data_process:main',
        ],
    },
)
