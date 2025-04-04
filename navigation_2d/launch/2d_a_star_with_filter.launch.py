import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    
    parameters = [{
        "distanceToObstacle": 0.05,
        "diagonalEdges": 3,
        "maxSecurityDistance": 0.15,
        "maximumHeight": 0.2,
        "minimumHeight": 0.06,
        'visualOdometry': False
    }]

  
    rviz_config_file = os.path.join(get_package_share_directory('navigation_2d'), 'rviz', 'default.rviz')

      
    return LaunchDescription([

        

        Node(
            package='navigation_2d',
            executable='2d_a_star_with_filter',
            output='screen',
            parameters=parameters,
        ),

        
        Node(
            package='algorithms',
            executable='send_poses',
            parameters=parameters,
            output='screen',
        ),

        Node(
            package='navigation_2d',
            executable='create_graph',
            output='screen',
            parameters=parameters,
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
        ),


    ])