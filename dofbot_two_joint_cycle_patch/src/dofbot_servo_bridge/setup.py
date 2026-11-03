import glob
import os

from setuptools import find_packages, setup

package_name = "dofbot_servo_bridge"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob.glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob.glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="ROS2 /joint_states 到德晟智能舵机的仿真-实物同步桥",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "servo_bridge = dofbot_servo_bridge.servo_bridge_node:main",
        ],
    },
)
