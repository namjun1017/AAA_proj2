from setuptools import find_packages, setup
import glob
import os

package_name = 'burger'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', glob.glob('resource/*')),
        ('share/' + package_name + '/launch', glob.glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nj',
    maintainer_email='rlaskawns101@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_move = burger.robot_move:main',
            'detection = burger.detection:main',
            'order_test_subscriber = burger.order_test_subscriber:main',
            'robot_move_integrated = burger.robot_move_integrated:main',
            'test=burger.test:main',
            'box_detection = burger.box_detection:main',
            'realsense = burger.realsense:main',
            'marker_check = burger.marker_check:main',
        ],
    },
)
