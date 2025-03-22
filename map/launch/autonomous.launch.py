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
        "distanceToObstacle": 0.25,
        "diagonalEdges": 3,
        "maxSecurityDistance": 0.25,
        "maxSecurityHeightDistance": 0.25,
        "x_min": -2.0,
        "x_max": 2.0,
        "y_min": -2.0,
        "y_max": 2.0,
        "z_min": 0.0,
        "z_max": 2.0,
    }]

  
    
      
    return LaunchDescription([

        Node(
            package='map',
            executable='a_estrela',
            output='screen',
            parameters=parameters,
        ),

        
       

       

       


    ])