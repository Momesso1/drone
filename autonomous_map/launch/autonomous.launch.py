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
        "diagonalEdges": 4,
        "maxSecurityDistance": 0.2,
        "maxSecurityHeightDistance": 0.2,
    }]

  
    
      
    return LaunchDescription([

        Node(
            package='autonomous_map',
            executable='a_estrela',
            output='screen',
            parameters=parameters,
        ),

        
        Node(
            package='autonomous_map',
            executable='send_poses',
            parameters=parameters,
            output='screen',
        ),

        Node(
            package='autonomous_map',
            executable='create_graph',
            output='screen',
            parameters=parameters,
        ),

       

    


    ])