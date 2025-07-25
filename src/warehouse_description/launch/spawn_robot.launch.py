import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Get the path to your package
    pkg_path = get_package_share_directory('warehouse_description')
    
    # Get the path to your URDF file
    urdf_file_path = os.path.join(pkg_path, 'urdf', 'warehouse_robot.urdf')

    # Get the path to your world file
    world_file_path = os.path.join(pkg_path, 'worlds', 'empty_world.world')

    # 1. Start Gazebo Sim
    # The '-r' argument tells Gazebo to run the simulation right away
    start_gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file_path],
        output='screen'
    )

    # 2. Start the Robot State Publisher
    # This node still reads the URDF and publishes the /tf topic
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file_path).read()}]
    )

    # 3. Spawn the robot in Gazebo
    # The 'ros_gz_sim create' executable is the new way to spawn entities.
    # It reads the 'robot_description' topic to get the URDF.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'warehouse_robot', # The name for the entity in Gazebo
                   '-allow_renaming', 'true'],
        output='screen'
    )

    # Create the LaunchDescription and add the actions
    return LaunchDescription([
        start_gazebo,
        robot_state_publisher,
        spawn_robot
    ])
