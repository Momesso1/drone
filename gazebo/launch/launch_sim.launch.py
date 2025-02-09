
'''
please check your folder(work_space) name!!!

run: 
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws/src/gazebo/models
    ros2 launch gazebo launch_sim.launch.py

if your gazebo is died
try:
    
    
'''

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

    robot_name = LaunchConfiguration('robot_name')
    height = LaunchConfiguration('height')
    use_sim_time = LaunchConfiguration('use_sim_time')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    package_name='gazebo'
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='1.0')


    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    

    robot_description_config = Command([
        'xacro ', xacro_file
        ])

    params = {'robot_description': robot_description_config}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    
    world = os.path.join(
        get_package_share_directory('gazebo'),
        'worlds',
        'turtlebot3_house.world'
    )


    # gazebo_params_file = os.path.join(get_package_share_directory(package_name),'configs','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', robot_name,
                                   '-x', x_pose,
                                   '-y', y_pose,
                                   '-z', height],
                        output='screen')

    # rviz2
    rviz =  Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=['-d' + os.path.join(get_package_share_directory(package_name), 'configs', 'sim.rviz')]
        )


    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),

        DeclareLaunchArgument(
            'robot_name', default_value='airlab_drone',
            description='The name of robot'),

        DeclareLaunchArgument(
            'height', default_value='0.1',
            description='The height of robot generate'),

        DeclareLaunchArgument(
            'rviz', default_value='false', 
            description='Launch RVIZ (optional).'),
        
        node_robot_state_publisher,
        gzserver_cmd,
        spawn_entity,
        rviz,
        gzclient_cmd,
     


    ])