# Requirements:
#   A realsense D435i
#   Install realsense2 ros2 package (ros-$ROS_DISTRO-realsense2-camera)
# Example:
#   $ ros2 launch rtabmap_examples realsense_d435i_stereo.launch.py

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    parameters=[{
          'frame_id':'base_link',
          'subscribe_stereo':True,
          'subscribe_odom_info': True,
          'approx_sync': True  ,
          'sync_queue_size': 40,
          'topic_queue_size': 200,
          'Grid/RangeMax': '0.0',
          'wait_imu_to_init':False}]
    
 


    return LaunchDescription([

     

        Node(
            package='autonomous_map', executable='a_estrela', output='screen',
            parameters=[{
                "distanceToObstacle": 0.05,
            }],
        ),

        Node(
            package='autonomous_map', executable='send_poses', output='screen',
        ),


        Node(
            package='autonomous_map', executable='create_graph', output='screen',
            parameters=[{
                "distanceToObstacle": 0.05,
                "resolution": 1,
                "maxSecurityDistance": 0.25,
                "maxHeightSecurityDistance": 0.25,
                "fixedNavigableVertices": False,
                "fixedNavigableVerticesDistance": 0.25,
            }],
        ),

        Node(
            package='autonomous_map', executable='navigable_graph', output='screen',
            parameters=[{
                "distanceToObstacle": 0.05,
                "resolution": 1,
                "x_min": -10.0,
                "x_max": 10.0,
                "y_min": -10.0,
                "y_max": 10.0,
                "z_min": 0.25,
                "z_max": 0.25,
            }],
        ),  

    
     


      
    ])







