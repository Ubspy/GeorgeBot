import os

from glob import glob
from setuptools import setup

package_name = 'arduino_serial'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jack Moren',
    maintainer_email='jack-m@live.com',
    description='Sends movement commands to the arduino via serial connection',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bidirectional = arduino_serial.arduino_serial:main'
        ],
    },
)
