import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    launch_desc = LaunchDescription()

    estimator_node = Node(
            package='state_estimator',
            executable='estimator'
    )
    launch_desc.add_action(estimator_node)

    path_planner_node = Node(
            package="path_planner",
            executable="planner"
    )
    launch_desc.add_action(path_planner_node)

    controller_node = Node(
            package="controller",
            executable='controller'
    )
    launch_desc.add_action(controller_node)

    data_recorder_node = Node(
                package="get_data",
                executable="data_collector"
    )
    launch_desc.add_action(data_recorder_node)
    return launch_desc

