from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            parameters=[{
                'use_sim_time': True
            }],
            arguments=[
                '-configuration_directory', '/home/spot/devel_ana/cartographer_spot/src/spot_cartographer/config',
                '-configuration_basename', 'spot_3d.lua'
            ],
            remappings=[
                ('/points2', '/Spot/Velodyne_Puck/point_cloud'),
                ('/imu', '/Spot/imu'),
                ('/odom', '/Spot/odometry')
            ]
        ),
        # RViz Node
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', '/home/hiwi/devel_ana/carto_ws/src/spot_cartographer/config/spot_cartographer.rviz'],
        #     parameters=[{
        #         'use_sim_time': True,
        #     }]
        # )
    ])
