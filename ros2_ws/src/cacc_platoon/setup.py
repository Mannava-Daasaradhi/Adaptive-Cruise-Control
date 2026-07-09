from glob import glob

from setuptools import find_packages, setup

package_name = "cacc_platoon"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config",
         glob("config/*.yaml") + glob("config/*.rviz")),
        ("share/" + package_name + "/worlds", glob("worlds/*.sdf")),
        ("share/" + package_name + "/models/cacc_car",
         glob("models/cacc_car/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Daasaradhi Mannava",
    maintainer_email="daasaradhimannava@gmail.com",
    description="Distributed CACC platoon simulation (one node per vehicle).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "leader_node = cacc_platoon.leader_node:main",
            "vehicle_node = cacc_platoon.vehicle_node:main",
            "channel_node = cacc_platoon.channel_node:main",
            "recorder_node = cacc_platoon.recorder_node:main",
            "viz_node = cacc_platoon.viz_node:main",
            "gz_bridge_node = cacc_platoon.gz_bridge_node:main",
        ],
    },
)
