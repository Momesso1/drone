import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    
    # Definindo os parâmetros a serem usados nos nós
    parameters = [{
        "distanceToObstacle": 0.2,
        "diagonalEdges": 3,
        "maxSecurityDistance": 0.20,
        "maxSecurityHeightDistance": 0.20,
        "time_between_points": 1, #Em ms.
    }]

  
    
      
    return LaunchDescription([

        

        Node(
            package='autonomous_map',
            executable='visualize_obstacles_graph',
            output='screen',
            parameters=parameters,
        ),

       


    ])