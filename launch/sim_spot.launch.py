from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

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
            '-configuration_directory', PathJoinSubstitution([FindPackageShare('spot_cartographer'), 'config']),
            '-configuration_basename', 'sim_spot.lua',
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

    # pc2_time_normalizer = Node(
    #     package='cartographer_ros',
    #     executable='pc2_time_normalizer',
    #     name='pc2_time_normalizer',
    #     parameters=[{
    #         'input':  '/velodyne_points_decoded',      # from point_cloud_transport
    #         'output': '/velodyne_points_cartographer'  # to Cartographer
    #     }]
    # )

    # RViz Node
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', '/home/spot/devel_ana/cartographer_spot/src/spot_cartographer/config/spot_cartographer.rviz'],
    #     parameters=[{
    #         'use_sim_time': True,
    #     }]
    # )
    return LaunchDescription([
        # Launch the nodes
        cartographer_node,
        cartographer_occupancy_grid_node,
        # pc2_time_normalizer,
        # rviz_node,

    ])
