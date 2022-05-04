import os
from setuptools import setup

package_name = 'nodebot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stepkun',
    maintainer_email='stephan.kunz@kabelbw.de',
    keywords=['remote', 'control'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: NGMC-License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='A ROS2 remote control',
    license='NGMC-License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_joysticks = two_joystick_test:main',
            'app = main:main',
        ],
    },
)
