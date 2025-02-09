# Requirements:
#   A realsense D435i
#   Install realsense2 ros2 package (ros-$ROS_DISTRO-realsense2-camera)
#
#   RTAB-Map in mapping mode

import os

from launch import LaunchDescription, Substitution, LaunchContext
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from typing import Text
from ament_index_python.packages import get_package_share_directory


class ConditionalText(Substitution):
    def __init__(self, text_if, text_else, condition):
        self.text_if = text_if
        self.text_else = text_else
        self.condition = condition

    def perform(self, context: 'LaunchContext') -> Text:
        if self.condition == True or self.condition == 'true' or self.condition == 'True':
            return self.text_if
        else:
            return self.text_else

            
def launch_setup(context, *args, **kwargs):     


    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')
    database_path = LaunchConfiguration('database_path')


    parameters={
    # 'frame_id':'camera_link',
    'frame_id': 'base_link',
    'use_sim_time': use_sim_time,
    'subscribe_depth': True,
    'subscribe_odom_info': True,
    'approx_sync': False,
    'qos_image': qos,
    'qos_imu': qos,
    'wait_imu_to_init': True,
    'database_path': database_path,
    }

    # use infra camera
    # remappings=[
    #       ('imu', '/imu/data'),
    #       ('rgb/image', '/camera/infra1/image_rect_raw'),
    #       ('rgb/camera_info', '/camera/infra1/camera_info'),
    #       ('depth/image', '/camera/depth/image_rect_raw')] 
    
    # use color camera 
    # remappings=[
    #     ('imu', '/imu/data'),
    #     ('rgb/image', '/camera/color/image_raw'),
    #     ('rgb/camera_info', '/camera/color/camera_info'),
    #     ('depth/image', '/camera/depth/image_rect_raw')] 

    # use color camera 
    remappings=[
        ('imu', '/imu/data'),
        ('rgb/image', '/camera/image_raw'),
        ('rgb/camera_info', '/camera/camera_info'),
        ('depth/image', '/camera/depth/image_raw')] 


    return [

        DeclareLaunchArgument('args',  default_value=LaunchConfiguration('rtabmap_args'), 
                              description='Can be used to pass RTAB-Map\'s parameters or other flags like --udebug and --delete_db_on_start/-d'),
    
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=[parameters],
            remappings=remappings),

        # SLAM mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']), # This will delete the previous database (~/.ros/rtabmap.db)
            
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True',
            #    'Grid/RayTracing':'True',
            #    'map_always_update':True,
               }],
            remappings=remappings),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[parameters],
            condition=IfCondition(LaunchConfiguration("rtabmap_viz")),
            remappings=remappings),

        Node(
            package='rviz2', executable='rviz2', output='screen',
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=[["-d"], [LaunchConfiguration("rviz_cfg")]]
            ),

        Node(
            package='rtabmap_util', executable='point_cloud_xyzrgb', output='screen',
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{
                "decimation": 4,
                "voxel_size": 0.0,
                "approx_sync": LaunchConfiguration('approx_sync'),
                "approx_sync_max_interval": LaunchConfiguration('approx_sync_max_interval')
            }],
            remappings=remappings),

                
        # Compute quaternion of the IMU
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False, 
                         'world_frame':'enu', 
                         'publish_tf':False}],
            remappings=[('imu/data_raw', '/camera/imu')]),
        
        # The IMU frame is missing in TF tree, add it:
        # Node(
        #     package='tf2_ros', executable='static_transform_publisher', output='screen',
        #     arguments=['0', '0', '0', '0', '0', '0', 'camera_gyro_optical_frame', 'camera_imu_optical_frame']),
    ]


def generate_launch_description():

    config_rviz = os.path.join(
        get_package_share_directory('gazebo'), 'configs', 'sim.rviz')   

    return LaunchDescription([

        # Declare Launch Argument
        DeclareLaunchArgument('approx_sync', default_value='false', 
                              description='If timestamps of the input topics should be synchronized using approximate or exact time policy.'),
        
        DeclareLaunchArgument('approx_sync_max_interval', default_value='0.0', 
                              description='(sec) 0 means infinite interval duration (used with approx_sync=true)'),
        
        DeclareLaunchArgument('database_path', default_value='~/autonomous/src/gazebo/map_database/rtabmap.db',  
                              description='Where is the map saved/loaded.'),
        
        DeclareLaunchArgument('localization', default_value='false', 
                              description='Launch in localization mode.'),
        
        DeclareLaunchArgument('rtabmap_viz', default_value='false',  
                              description='Launch RTAB-Map UI (optional).'),
        
        DeclareLaunchArgument('rviz', default_value='true', 
                              description='Launch RVIZ (optional).'),
        
        DeclareLaunchArgument('rviz_cfg', default_value=config_rviz, 
                              description='Configuration path of rviz2.'),
        
        DeclareLaunchArgument('initial_pose', default_value='', 
                              description='Set an initial pose (only in localization mode). Format: "x y z roll pitch yaw" or "x y z qx qy qz qw". Default: see "RGBD/StartAtOrigin" doc'),
        
        DeclareLaunchArgument('rtabmap_args',   default_value='',                   
                              description='Backward compatibility, use "args" instead.'),

        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument('qos', default_value='2',
                              description='QoS used for input sensor topics'),

        OpaqueFunction(function=launch_setup)

        
    ])