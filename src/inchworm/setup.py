from setuptools import find_packages, setup
import os

package_name = 'inchworm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['inchworm/robot_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryung',
    maintainer_email='ryung9514@naver.com',
    description='Launch-only package for inchworm control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
        'launch.frontend.launch_py': [
            'robot_launch = inchworm.robot_launch:generate_launch_description',
        ],
    }
)
