#!/usr/bin/env python3

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():

    remappable_topics = [
        DeclareLaunchArgument("input_topic", default_value="~/input"),
        DeclareLaunchArgument("output_topic", default_value="~/output"),
    ]

    args = [
        DeclareLaunchArgument("name", default_value="ros2_cpp_node", description="node name"),
        DeclareLaunchArgument("namespace", default_value="", description="node namespace"),
        DeclareLaunchArgument("params", default_value=os.path.join(get_package_share_directory("ros2_cpp_multi_threaded_pkg"), "config", "params.yml"), description="path to parameter file"),
        DeclareLaunchArgument("log_level", default_value="info", description="ROS logging level (debug, info, warn, error, fatal)"),
        DeclareLaunchArgument("num_threads", default_value="1", description="number of threads for MultiThreadedExecutor"),
        DeclareLaunchArgument("use_sim_time", default_value="false", description="use simulation clock"),
        *remappable_topics,
    ]

    nodes = [
        Node(
            package="ros2_cpp_multi_threaded_pkg",
            executable="ros2_cpp_node",
            namespace=LaunchConfiguration("namespace"),
            name=LaunchConfiguration("name"),
            parameters=[{"num_threads": LaunchConfiguration("num_threads")}, LaunchConfiguration("params")],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            remappings=[(la.default_value[0].text, LaunchConfiguration(la.name)) for la in remappable_topics],
            output="screen",
            emulate_tty=True,
        )
    ]

    return LaunchDescription([
        *args,
        SetParameter("use_sim_time", LaunchConfiguration("use_sim_time")),
        *nodes,
    ])
