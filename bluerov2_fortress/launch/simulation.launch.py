from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetParameter, SetRemap
import os
def generate_launch_description():

    # Caminho do mundo
    world_file = '/home/momesso/autonomous/src/bluerov2_fortress/worlds/bluerov2_heavy_underwater.world'
    pkg_stereo_image_proc = get_package_share_directory(
        'stereo_image_proc')

    # Paths
    stereo_image_proc_launch = PathJoinSubstitution(
        [pkg_stereo_image_proc, 'launch', 'stereo_image_proc.launch.py'])

    # Caminho do arquivo de parâmetros do ArduSub
    orca_bringup_dir = get_package_share_directory('bluerov2_fortress')

    sim_left_ini = os.path.join(orca_bringup_dir, 'config', 'sim_left.ini')
    sim_right_ini = os.path.join(orca_bringup_dir, 'config', 'sim_right.ini')

    pkg_project_gazebo = get_package_share_directory('bluerov2_fortress')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Load the SDF file from "description" package
    sdf_file = os.path.join(pkg_project_gazebo, 'models', 'bluerov2_heavy', 'model.sdf')

    # Convert SDF to URDF
    try:
        urdf_content = subprocess.check_output(['ign', 'sdf', '-p', sdf_file], universal_newlines=True)
    except Exception as e:
        raise RuntimeError(f"Failed to convert SDF to URDF: {e}")

    params = {'robot_description': urdf_content}

    # Publicar a descrição do robô usando o robot_state_publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r -v5 {world_file}',
            'on_exit_shutdown': 'false',
            'use_sim_time': 'true'
        }.items()
    )

    # Criação dos nós para as transformações estáticas
  
    return LaunchDescription([
        DeclareLaunchArgument(
            'gzclient',
            default_value='True',
            description='Launch Gazebo UI?'
        ),
        node_robot_state_publisher,
        #*static_transform_nodes,
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['gazebo/stereo_camera/left', 'gazebo/stereo_camera/right'],
            parameters=[{
                'use_sim_time': True,
            }],
            output='screen',
        ),
        Node(
            package='bluerov2_fortress',
            executable='camera_info_publisher',
            name='left_info_publisher',
            output='screen',
            parameters=[{
                'camera_info_url': 'file://' + sim_left_ini,
                'camera_name': 'stereo_left',
                'frame_id': 'stereo_left_frame',
                'timer_period_ms': 5,
               
            }],
            remappings=[
                ('/camera_info', 'gazebo/stereo_camera/left/camera_info'),
            ],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{
                'config_file': os.path.join(orca_bringup_dir, 'config', 'ros2_bridge.yaml'),
                'use_sim_time': True,
                
            }],
            output='screen'
        ),

        Node(
            package='bluerov2_fortress',
            executable='camera_sync_publisher',
            
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'timeout': 0.5}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            parameters=[{'timeout': 0.0}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_chassis',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'chassis'],
            parameters=[{'timeout': 0.5}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='chassis_to_camera_link',
            arguments=['0', '0', '0', '0', '0', '0', 'chassis', 'camera_link'],
            parameters=[{'timeout': 0.5}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_left_camera',
            arguments=['-0.18', '0.18', '-0.0675', '3.14', '3.14', '1.57', 'camera_link', ' bluerov2_heavy/left_camera_link/left_camera'],
            parameters=[{'timeout': 0.5}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_right_camera',
            arguments=['-0.18', '-0.18', '-0.0675', '3.14', '3.14', '1.57', 'camera_link', 'bluerov2_heavy/right_camera_link/right_camera'],
            parameters=[{'timeout': 0.5}]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu',
            arguments=['0', '0', '3.14', '0', '0', '0', 'base_link', 'bluerov2_heavy/base_link/imu_sensor'],
            parameters=[{'timeout': 0.5}]
        ),



      
        

        Node(
            package='bluerov2_fortress',
            executable='camera_info_publisher',
            name='right_info_publisher',
            output='screen',
            parameters=[{
                'camera_info_url': 'file://' + sim_right_ini,
                'camera_name': 'stereo_right',
                'frame_id': 'stereo_right_frame',
                'timer_period_ms': 5,
                
            }],
            remappings=[
                ('/camera_info', 'gazebo/stereo_camera/right/camera_info'),
            ],
        ),
        # # # Uncompress images for stereo_image_rect and remap to expected names from stereo_image_proc
        
        # Uncompress images for stereo_image_rect and remap to expected names from stereo_image_proc
        Node(
            package='image_transport', executable='republish', name='republish_left', output='screen',
            namespace='stereo_camera',
            arguments=['compressed', 'raw'],
            remappings=[('in/compressed', 'left/image_raw_throttle/compressed'),
                        ('out',           'left/image_raw')]),
        Node(
            package='image_transport', executable='republish', name='republish_right', output='screen',
            namespace='stereo_camera',
            arguments=['compressed', 'raw'],
            remappings=[('in/compressed', 'right/image_raw_throttle/compressed'),
                        ('out',           'right/image_raw')]),

        # Run the ROS package stereo_image_proc for image rectification   
        GroupAction(
            actions=[

                SetRemap(src='camera_info',dst='camera_info_throttle'),
                SetRemap(src='camera_info',dst='camera_info_throttle'),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([stereo_image_proc_launch]),
                    launch_arguments=[
                        ('left_namespace', 'stereo_camera/left'),
                        ('right_namespace', 'stereo_camera/right'),
                        ('disparity_range', '128'),
                    ]
                ),
            ]
        ),
        gz_sim
    ])