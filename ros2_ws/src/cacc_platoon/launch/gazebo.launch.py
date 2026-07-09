"""Bring up the CACC platoon inside Gazebo (gz-sim Harmonic).

    ros2 launch cacc_platoon gazebo.launch.py n:=6 controller:=cthp \
        params:=<case.yaml> rviz:=true

Pipeline:
  1. gz-sim serves ``worlds/highway.sdf`` (via ros_gz_sim).
  2. N+1 kinematic ``cacc_car`` models are spawned (leader white, followers a
     viridis gradient) at their initial platoon-frame spacing.
  3. the usual CACC nodes run (leader + channel + recorder + N vehicle nodes)
     -- reused from ``platoon.launch.py`` so the physics is byte-identical.
  4. ``gz_bridge_node`` teleports each car to its live CACC position every tick.

Gazebo only *renders*; the vehicle nodes remain the single physics authority.
Needs ``ros-jazzy-ros-gz`` + ``gz-harmonic`` (see 08_ROS2_Architecture.md §7).
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from cacc_platoon.gz_assets import WORLD, VEH_LEN, car_rgb, car_sdf_doc

# initial spacing used only to place the freshly-spawned cars; the bridge
# corrects every pose on its first tick from the true CACC state
_GAP0 = 1.0 + 0.95 * 20.0 + VEH_LEN   # r + h*v0 + L  (case-A defaults)


def _spawn(context):
    n = int(LaunchConfiguration("n").perform(context))
    controller = LaunchConfiguration("controller").perform(context)
    params = LaunchConfiguration("params").perform(context)
    outfile = LaunchConfiguration("outfile").perform(context)
    do_rviz = LaunchConfiguration("rviz").perform(context).lower() in ("true", "1")
    headless = LaunchConfiguration("headless").perform(context).lower() in ("true", "1")

    pkg = get_package_share_directory("cacc_platoon")
    world_path = os.path.join(pkg, "worlds", "highway.sdf")
    ros_gz = get_package_share_directory("ros_gz_sim")

    # 1) gz-sim: full GUI, or server-only (-s) when headless (CI / no display)
    gz_flags = "-s -r" if headless else "-r"
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"{gz_flags} -v 3 {world_path}"}.items())

    # 2) spawn the cars (delayed so the /world/.../create service is up)
    spawns = []
    for i in range(n + 1):
        x = -(i * _GAP0) - 0.5 * VEH_LEN
        spawns.append(Node(
            package="ros_gz_sim", executable="create", output="screen",
            arguments=["-world", WORLD, "-name", f"vehicle_{i}",
                       "-string", car_sdf_doc(f"vehicle_{i}", car_rgb(i)),
                       "-x", f"{x:.3f}", "-y", "0", "-z", "0"]))

    # 3) the real CACC nodes (single source of truth)
    platoon = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "platoon.launch.py")),
        launch_arguments={"n": str(n), "controller": controller,
                          "params": params, "outfile": outfile}.items())

    # 4) ROS -> gz pose bridge
    common = [params] if params else []
    bridge = Node(package="cacc_platoon", executable="gz_bridge_node",
                  name="gz_bridge", output="screen",
                  parameters=common + [{"n_followers": n, "world": WORLD}])

    nodes = [gz, platoon, TimerAction(period=3.0, actions=spawns),
             TimerAction(period=4.0, actions=[bridge])]

    if do_rviz:
        rviz_cfg = os.path.join(pkg, "config", "platoon.rviz")
        nodes += [
            Node(package="cacc_platoon", executable="viz_node", name="viz",
                 output="screen", parameters=common + [{"n_followers": n}]),
            Node(package="rviz2", executable="rviz2", name="rviz2",
                 arguments=["-d", rviz_cfg], output="screen")]
    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("n", default_value="6"),
        DeclareLaunchArgument("controller", default_value="cthp"),
        DeclareLaunchArgument("params", default_value=""),
        DeclareLaunchArgument("outfile", default_value="/tmp/platoon_gz.csv"),
        DeclareLaunchArgument("rviz", default_value="false",
                              description="also open rviz2 with the markers"),
        DeclareLaunchArgument("headless", default_value="false",
                              description="run gz-sim server-only, no GUI"),
        OpaqueFunction(function=_spawn),
    ])
