"""Bring up the full platoon: leader + N vehicle nodes + channel + recorder.

    ros2 launch cacc_platoon platoon.launch.py n:=12 controller:=cthp \
        params:=/path/to/case.yaml outfile:=/tmp/platoon_run.csv

``params`` must be a ROS 2 params YAML applied to every node (physics,
gains, channel impairments); per-node ``index`` / ``n_followers`` /
``outfile`` are injected here and override the file.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawn(context):
    n = int(LaunchConfiguration("n").perform(context))
    controller = LaunchConfiguration("controller").perform(context)
    params = LaunchConfiguration("params").perform(context)
    outfile = LaunchConfiguration("outfile").perform(context)
    viz = LaunchConfiguration("viz").perform(context).lower() in ("true", "1")
    common = [params] if params else []

    nodes = [
        Node(package="cacc_platoon", executable="leader_node", name="leader",
             output="screen", parameters=common),
        Node(package="cacc_platoon", executable="channel_node",
             name="v2v_channel", output="screen",
             parameters=common + [{"n_followers": n}]),
        Node(package="cacc_platoon", executable="recorder_node",
             name="recorder", output="screen",
             parameters=common + [{"n_followers": n, "outfile": outfile}]),
    ]
    if viz:
        nodes.append(
            Node(package="cacc_platoon", executable="viz_node", name="viz",
                 output="screen", parameters=common + [{"n_followers": n}]))
    for i in range(1, n + 1):
        nodes.append(
            Node(package="cacc_platoon", executable="vehicle_node",
                 name=f"vehicle_{i}", output="screen",
                 parameters=common + [{"index": i, "controller": controller}]))
    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("n", default_value="6",
                              description="number of follower vehicles"),
        DeclareLaunchArgument("controller", default_value="cthp",
                              description="acc | cacc | cthp"),
        DeclareLaunchArgument("params", default_value="",
                              description="ROS params YAML for all nodes"),
        DeclareLaunchArgument("outfile", default_value="/tmp/platoon_run.csv",
                              description="recorder CSV path"),
        DeclareLaunchArgument("viz", default_value="false",
                              description="publish rviz2 markers (viz_node)"),
        OpaqueFunction(function=_spawn),
    ])
