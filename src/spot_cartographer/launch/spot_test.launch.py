from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Cartographer Node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        parameters=[{
            'use_sim_time': True,
        }],
        arguments=[
            '-configuration_directory', '/home/hiwi/devel_ana/carto_ws/src/spot_cartographer/config',
            '-configuration_basename', 'spot_test.lua',
        ],
        remappings=[
            ('points2', '/Spot/Velodyne_Puck/point_cloud'),
            ('imu', '/Spot/imu'),
            ('odom', '/Spot/odometry'),
        ]   
    )

    # Cartographer Occupancy Grid Node
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        parameters=[
            {'use_sim_time': True},
            {'resolution': 0.05},
        ]
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '/home/hiwi/devel_ana/carto_ws/src/spot_cartographer/config/spot_cartographer.rviz'],
        parameters=[{
            'use_sim_time': True,
        }]
    )

    # Return the launch description
    return LaunchDescription([
        # Launch the nodes
        cartographer_node,
        cartographer_occupancy_grid_node,
        rviz_node,

    ])
