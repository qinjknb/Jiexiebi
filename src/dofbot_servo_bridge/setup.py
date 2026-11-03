import os

from setuptools import setup


package_name = "dofbot_servo_bridge"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), ["config/bridge.yaml"]),
        (os.path.join("share", package_name, "launch"), [
            "launch/servo_bridge.launch.py", "launch/mirror_demo.launch.py",
            "launch/mirror_cycle.launch.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Six-joint MoveIt simulation to GCWJ serial servo mirror",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "servo_bridge = dofbot_servo_bridge.servo_bridge_node:main",
    ]},
)
