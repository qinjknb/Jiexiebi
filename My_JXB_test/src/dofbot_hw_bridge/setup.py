from glob import glob
import os

from setuptools import setup

package_name = 'dofbot_hw_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyserial', 'PyYAML'],
    zip_safe=True,
    maintainer='dofbot',
    maintainer_email='dofbot@example.com',
    description='DOFBOT Pro 机械臂 MoveIt 实机桥（STM32 + 总线舵机）',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_bridge = dofbot_hw_bridge.trajectory_bridge:main',
            'servo_tool = dofbot_hw_bridge.servo_tool:main',
            'reach_test = dofbot_hw_bridge.reach_test:main',
            'dofbot_selftest = dofbot_hw_bridge.selftest:main',
        ],
    },
)
