import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # Get the path to your package
    pkg_path = get_package_share_directory('warehouse_description')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    # Get the path to your URDF file
    urdf_file_path = os.path.join(pkg_path, 'urdf', 'warehouse_robot.urdf')

    # Get the path to your world file
    world_file_path = os.path.join(pkg_path, 'worlds', 'empty_world.world')
    ekf_config_path = os.path.join(pkg_path, 'config', 'ekf.yaml')
    # 1. Start Gazebo Sim
    # The '-r' argument tells Gazebo to run the simulation right away
    start_gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file_path],
        output='screen'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', world_file_path], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()
    )


    # This node manually creates a bridge between Gazebo and ROS 2 topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Bridge ROS 2 /cmd_vel to Gazebo /model/warehouse_robot/cmd_vel
            '/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',
            
            # Bridge Gazebo /odom to ROS 2 /odom
            '/odom@nav_msgs/msg/Odometry]gz.msgs.Odometry'
        ],
        output='screen'
    )


    # 2. Start the Robot State Publisher
    # This node still reads the URDF and publishes the /tf topic
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(urdf_file_path).read()}]
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

    start_robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path]
    )

    # Create the LaunchDescription and add the actions
    return LaunchDescription([
        start_gazebo,
        # gzserver_cmd,
        # gzclient_cmd,
        robot_state_publisher,
        spawn_robot,
        bridge,
        #start_robot_localization_node
    ])
