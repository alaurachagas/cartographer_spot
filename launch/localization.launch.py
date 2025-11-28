from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    cfg_dir = PathJoinSubstitution([FindPackageShare('spot_cartographer'), 'config'])
    lua_basename = 'real_spot_localization.lua'

    carto = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        arguments=[
            '-configuration_directory', cfg_dir,
            '-configuration_basename', lua_basename,
            '-load_state_filename', '/map/validation_map.pbstream',
            '-load_frozen_state', 'true'
        ],
        remappings=[
            ('points2', '/velodyne_points_cartographer'),
            ('odom',    '/odometry'),       # your Spot odom topic
        ],
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'resolution': 0.05},
            {'use_sim_time': False}
            ],
        )

    return LaunchDescription([
        carto, 
        cartographer_occupancy_grid_node
    ])
