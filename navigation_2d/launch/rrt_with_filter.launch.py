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
        "distanceToObstacle": 0.15,
        "diagonalEdges": 3,
        "maxSecurityDistance": 0.15,
        "maximumHeight": 0.2,
        "minimumHeight": 0.06,
        "max_iterations": 50000,
        "goal_sample_rate": 0.1, #Probabilidade do algoritmo expandir a árvore diretamente para o objeto em vez de abrir aleatoriamente pelo mapa.
        'visualOdometry': False
    }]

  
    rviz_config_file = os.path.join(get_package_share_directory('algorithms'), 'rviz', 'default.rviz')

      
    return LaunchDescription([

        

        Node(
            package='navigation_2d',
            executable='rrt_with_filter',
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



    ])