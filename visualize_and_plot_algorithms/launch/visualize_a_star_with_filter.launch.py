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
        "distanceToObstacle": 0.2,
        "diagonalEdges": 3,
        "maxSecurityDistance": 0.20,
        "maxSecurityHeightDistance": 0.20,
        "activate_only_with_obstacles": True
    }]

  
    rviz_config_file = os.path.join(get_package_share_directory('visualize_and_plot_algorithms'), 'rviz', 'visualize_a_star_with_filter.rviz')

      
    return LaunchDescription([

        

        Node(
            package='visualize_and_plot_algorithms',
            executable='visualize_a_star_with_filter',
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
            package='algorithms',
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