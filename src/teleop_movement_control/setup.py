from setuptools import setup

package_name = 'teleop_movement_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jack Moren',
    maintainer_email='jack-m@live.com',
    description='Gets the controller data published and converts it to a movement instruction',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop = teleop_movement_control.teleop_movement_control:main'
        ],
    },
)
