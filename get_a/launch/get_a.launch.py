import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler

from launch_ros.actions import Node
import xacro
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'get_a'
    file_subpath = 'description/example_robot.urdf.xacro'

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )

    path_to_package = get_package_share_directory('get_a')
    sub_folder = 'config'
    file_name = 'get_a.rviz'
    file_path = os.path.join(path_to_package,sub_folder,file_name)  # join path  

    rviz = Node(
       package='rviz2',
       executable='rviz2',
       name='rviz',
       arguments=['-d', file_path],
       output='screen')

    robot_desc_path = os.path.join(get_package_share_directory(
                                    'get_a'), 
                                    'description',
                                    'example_robot.urdf.xacro')

    robot_description = xacro.process_file(robot_desc_path).toxml() #read URDF file from xacro

    robot_state_publisher = Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='screen',
                                  parameters=[
                                    # {'use_sim_time': 'false'},
                                    {'robot_description': robot_description}
    ])

    joint_state_publisher_gui = Node(package='joint_state_publisher_gui',
                                        executable='joint_state_publisher_gui',
                                        name='joint_state_publisher_gui'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )


    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'robot'],
                    output='screen')

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["velocity_controllers", "--controller-manager", "/controller_manager"],
    )


    # Run the node
    return LaunchDescription([

        gazebo,
        # rviz,
        # robot_state_publisher,
        # joint_state_publisher_gui,
        node_robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster_spawner,
        robot_controller_spawner
    ])

