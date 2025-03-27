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
        "distanceToObstacle": 0.3,
        "diagonalEdges": 3,
        "maxSecurityDistance": 0.30,
        "maxSecurityHeightDistance": 0.30,
        "time_between_points": 1, # In ms.
        "activate_only_with_obstacles": False
    }]

    rviz_config_file = os.path.join(get_package_share_directory('visualize_and_plot_algorithms'), 'rviz', 'visualize_obstacles_graph.rviz')

    
      
    return LaunchDescription([

        

        Node(
            package='visualize_and_plot_algorithms',
            executable='visualize_obstacles_graph',
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