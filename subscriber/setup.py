from setuptools import setup

package_name = 'py_subscriber_aws'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='blue23',
    maintainer_email='g807b153@gmail.com',
    description='Subscriber for AWS',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'listener = py_subscriber_aws.subscriber_member_function:main'
        ],
    },
)
