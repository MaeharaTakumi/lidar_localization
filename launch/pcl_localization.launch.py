import os

import launch
import launch.actions
import launch.events

import launch_ros
import launch_ros.actions
import launch_ros.events

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

import lifecycle_msgs.msg

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_share_dir = get_package_share_directory('pcl_localization_ros2')

    ld = launch.LaunchDescription()

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_share_dir, 'rviz', 'localization2.rviz'),
        description='Full path to the RViz config file to use.'
    )

    map_path_arg = DeclareLaunchArgument(
        'map_path_arg',
        default_value='',
        description='Full path to the PCD map file to override the default in YAML.'
    )

    # NDT だけを起動する（実機用）。EKF と合わせて環境ごとに起動するときは
    #   ros2 launch localization_bringup localization.launch.py env:=real

    # base_link -> velodyne（実機の取付）。ノードは base_link への変換にこの TF を使う
    lidar_tf = launch_ros.actions.Node(
        name='lidar_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '-0.17', '--y', '0.0', '--z', '1.2',
                   '--roll', '0.0', '--pitch', '0.0', '--yaw', '-0.017',
                   '--frame-id', 'base_link', '--child-frame-id', 'velodyne']
        )

    imu_tf = launch_ros.actions.Node(
        name='imu_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','1','base_link','imu_link']
        )

    localization_param_dir = launch.substitutions.LaunchConfiguration(
        'localization_param_dir',
        default=os.path.join(
            get_package_share_directory('pcl_localization_ros2'),
            'param',
            'localization.yaml'))

    pcl_localization = launch_ros.actions.LifecycleNode(
        name='pcl_localization',
        namespace='',
        package='pcl_localization_ros2',
        executable='pcl_localization_node',
        parameters=[
            localization_param_dir,
            {'map_path': LaunchConfiguration('map_path_arg')}
        ],
        remappings=[('/cloud','/velodyne_points'),
                    ('/pcl_pose', '/current_pose')],
        output='screen')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        # 点群の stamp は別 PC の時計なので、RViz は点群と同じ時刻で配信される /tf_rviz を TF として使う
        # （/tf_static はそのまま。静的 TF は時刻に依存しない）
        remappings=[('/tf', '/tf_rviz')],
        output='screen'
    )

    to_inactive = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(pcl_localization),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    from_unconfigured_to_inactive = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=pcl_localization,
            goal_state='unconfigured',
            entities=[
                launch.actions.LogInfo(msg="-- Unconfigured --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(pcl_localization),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )

    from_inactive_to_active = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=pcl_localization,
            start_state = 'configuring',
            goal_state='inactive',
            entities=[
                launch.actions.LogInfo(msg="-- Inactive --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(pcl_localization),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    ld.add_action(rviz_config_arg)
    ld.add_action(rviz_node)
    ld.add_action(map_path_arg)
    ld.add_action(from_unconfigured_to_inactive)
    ld.add_action(from_inactive_to_active)

    ld.add_action(pcl_localization)
    ld.add_action(imu_tf)
    ld.add_action(lidar_tf)
    ld.add_action(to_inactive)

    return ld
