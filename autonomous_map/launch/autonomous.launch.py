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
        "distanceToObstacle": 0.3,
        "resolution": 1,
        "x_min": -42.0,
        "x_max": 42.0,
        "y_min": -42.0,
        "y_max": 42.0,
        "z_min": 0.0,
        "z_max": 3.0,
        "maxSecurityDistance": 0.3,
        "maxSecurityHeightDistance": 0.3,
        "fixedNavigableVertices": False,
        "fixedNavigableVerticesDistance": 0.25,
        #"use_3d": True,
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

       
        Node(
            package='autonomous_map',
            executable='navigable_graph',
            output='screen',
            parameters=parameters,
        ),

    


    ])