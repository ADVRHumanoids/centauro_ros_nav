import os
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    centauro_nav_dir = get_package_share_directory('centauro_ros_nav')
    perception_utils_dir = get_package_share_directory('perception_utils')

    # Create the launch configuration variables
    slam = LaunchConfiguration('slam')
    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')

    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_simulator = LaunchConfiguration('use_simulator')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')
    use_octomap = LaunchConfiguration('use_octomap')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static'),
                  ('/odom', '/centauro/odom')]

    # Declare the launch arguments
    declare_slam_cmd = DeclareLaunchArgument(
        'slam', default_value='False', description='Whether run a SLAM'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(centauro_nav_dir, 'maps', 'empty.yaml'),
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(centauro_nav_dir, 'config', 'centauro_nav_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description='Whether to use composed bringup',
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        # default_value=os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz'),
        default_value=os.path.join(centauro_nav_dir, 'rviz', 'nav2.rviz'),
        description='Full path to the RVIZ config file to use',
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='True', description='Whether to start RVIZ'
    )
    
    declare_use_octomap_cmd = DeclareLaunchArgument(
       'use_octomap',
       default_value='True',
       description='Whether to start Octomap')

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(centauro_nav_dir, 'launch', 'bringup_nav_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'slam': slam,
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
        }.items(),
    )

    rviz_node_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config_file]
    )

    octomap_velodyne_cmd = Node(
       condition=IfCondition(use_octomap),
       package='octomap_server',
       executable='octomap_server_node',
       name='octomap_server',
       output='own_log',
       parameters=[{'resolution': 0.25},
                   {'frame_id': "odom"},
                   {'base_frame_id': "velodyne_calib"},
                   {'sensor_model.max_range': 20.0},
                   {'use_sim_time': use_sim_time},
                   {'occupancy_min_z': -0.6},
                   {'occupancy_max_z': 1.0},
                   {'sensor_model.hit': 0.55},
                   {'sensor_model.miss': 0.45}],
       remappings=[('/cloud_in', '/lidar/points'),
                   ('/projected_map', '/map')]
    )

    octomap_d435_cmd = Node(
       condition=IfCondition(use_octomap),
       namespace='local',
       package='octomap_server',
       executable='octomap_server_node',
       name='octomap_server_d435',
       output='screen',
       parameters=[{'resolution': 0.10},
                   {'frame_id': "odom"},
                   {'base_frame_id': "D435i_camera_link"},
                   {'sensor_model.max_range': 5.0},
                   {'use_sim_time': use_sim_time},
                   {'occupancy_min_z': -2.5},
                   {'occupancy_max_z': 2.5}],
       remappings=[('/local/cloud_in', '/D435i_camera/points_filtered'), #/D435i_camera/points_filtered /D435i_camera/points
                   ('/local/projected_map', '/local/map')]
    )

    static_map_odom_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_odom',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
    )


    valid_target_selector_cmd = Node(
        package='centauro_ros_nav',
        executable='valid_target_selector_node',
        name='valid_target_selector_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'map_topic_name' : "/map"}]
    )

    # filter_d435_cmd = Node(
    #     package='perception_utils',
    #     executable='cloud_processing_node',
    #     name='cloud_processing_node',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}]
    # )

    filter_d435_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(perception_utils_dir,
                                                   'launch', 
                                                   'cloud_filtering.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'voxel_dim': '0.02',
            # 'sor_std_dev': '0.03',
            # 'sor_mean_k': '7',
            'pt_min_x': '-5.0',
            'pt_max_x': '5.0',
            'pt_min_y': '-5.0',
            'pt_max_y': '5.0',
            'pt_min_z': '-5.0',
            'pt_max_z': '5.0',
            'target_frame': 'pelvis',
        }.items(),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_octomap_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_respawn_cmd)

    ld.add_action(octomap_velodyne_cmd)
    ld.add_action(octomap_d435_cmd)
    ld.add_action(static_map_odom_cmd)
    ld.add_action(valid_target_selector_cmd)
    
    # Add the actions to launch all of the navigation nodes
    ld.add_action(rviz_node_cmd)
    ld.add_action(bringup_cmd)
    ld.add_action(filter_d435_cmd)

    return ld