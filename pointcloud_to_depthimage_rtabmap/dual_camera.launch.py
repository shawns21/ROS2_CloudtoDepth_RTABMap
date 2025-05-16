from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def make_camera_nodes(camera_names, use_sim_time):

    nodes = []

    for name in camera_names:

        remaps = [

            ('camera_info', f'{name}/camera/camera_info'),
            ('rgb/image', f'{name}/camera/image_raw'),
            ('rgb/camera_info', f'{name}/camera/camera_info'),
            ('image_raw', f'{name}/depth/image16'),
            ('image', f'{name}/depth/image32'),
            ('depth/image', f'{name}/depth/image32'),
            ('cloud', 'velodyne_points')
 
        ]

        nodes.append(

            Node(
                package='rtabmap_util', executable='pointcloud_to_depthimage', output='screen',
                parameters=[{'approx_sync':True, 'use_sim_time':use_sim_time, 'fixed_frame_id':'odom'}],
                name=f'{name}_pcl_to_depth',
                remappings=remaps
            )

        )

        nodes.append(
                
            Node(
                package='rtabmap_sync', executable='rgbd_sync', output='screen',
                parameters=[{'approx_sync':True, 'use_sim_time':use_sim_time}],
                name=f'{name}_rgbd_sync',
                remappings=remaps
            )

        )

    return(nodes)


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')

    parameters={
          'frame_id':'base_footprint',
          'use_sim_time':use_sim_time,
          'subscribe_depth':True,
          'use_action_for_goal':True,
          'Reg/Force3DoF':'true',
          'Grid/RayTracing':'true', # Fill empty space
          'Grid/3D':'false', # Use 2D occupancy
          'Grid/RangeMax':'3',
          'Grid/NormalsSegmentation':'false', # Use passthrough filter to detect obstacles
          'Grid/MaxGroundHeight':'0.05', # All points above 5 cm are obstacles
          'Grid/MaxObstacleHeight':'0.4',  # All points over 1 meter are ignored
          'Optimizer/GravitySigma':'0' # Disable imu constraints (we are already in 2D)
    }
    
    camera_nodes = make_camera_nodes(['front', 'rear'], use_sim_time)

    base_remappings=[

          #Set your pointcloud topic
          ('cloud', 'velodyne_points'),

          #Set your rgb camera topics
          ('camera_info', 'front/camera/camera_info'),
          ('rgb/image', 'front/camera/image_raw'),
          ('rgb/camera_info', 'front/camera/camera_info'),

    ]

    fixed_remappings=[

          #Do not change

          #Remapping pointcloud to depth image node to publish to these topics:
          ('image_raw', 'front/depth/image16'),
          ('image', 'front/depth/image32'),

          #Making rgbd_sync subscribe to the new topic
          ('depth/image', 'front/depth/image32')

    ]




    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'),

        # Nodes to launch

        *camera_nodes,

        # SLAM Mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=base_remappings + fixed_remappings,
            arguments=['-d']),
            
        # Localization mode:

        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=base_remappings + fixed_remappings),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[parameters],
            remappings=base_remappings + fixed_remappings),

                
    ])
