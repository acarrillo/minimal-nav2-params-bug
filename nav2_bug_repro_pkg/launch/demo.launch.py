import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    package_dir = get_package_share_directory('nav2_bug_repro_pkg')

    map_file = LaunchConfiguration('map')
    mode = LaunchConfiguration('mode')

    params_file = PythonExpression([
        "'", os.path.join(package_dir, 'config', 'good_params.yaml'), "' if '", mode, "' == 'good' else '",
        os.path.join(package_dir, 'config', 'bad_params.yaml'), "'"
    ])

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(package_dir, 'maps', 'map.yaml'),
        description='Full path to map yaml file'
    )

    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value='bad',
        description='Mode to run: "good" or "bad"'
    )

    lifecycle_nodes = ['map_server', 'planner_server']

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file,
            'use_sim_time': False
        }]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': lifecycle_nodes
        }]
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765
        }]
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
    )

    path_request_node = Node(
        package='nav2_bug_repro_pkg',
        executable='path_request_node.py',
        name='path_request_node',
        output='screen'
    )

    return LaunchDescription([
        declare_map_cmd,
        declare_mode_cmd,
        map_server,
        planner_server,
        lifecycle_manager,
        foxglove_bridge,
        static_tf,
        path_request_node
    ])