import os

from glob import glob
from setuptools import setup

package_name = 'aws_client'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GeorgeBlue',
    maintainer_email='g807b153@gmail.com',
    description='Gets information from various services and sends them to the EC2 Instance',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'client = aws_client.aws_client:main'
        ],
    },
)
