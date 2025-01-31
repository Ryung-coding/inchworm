from setuptools import setup, find_packages

package_name = 'crawling_robot_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(where='src'),  # Python 패키지를 자동 탐색
    package_dir={'': 'src'},  # src 내부를 패키지 디렉토리로 설정
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='ryung',
    maintainer_email='ryung9514@naver.com',
    description='Python-based gripper control using ROS2 and serial communication',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'gripper_control = crawling_robot_controller.gripper_control:main',
        ],
    },
)

