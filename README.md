git clone ...
cd ~/warehouse_robot_ws/src

colcon build --symlink-install
source install/setup.bash
(good practice to run rosdep install to ensure all dependencies are included)
ros2 launch warehouse_description spawn_robot.launch.py
