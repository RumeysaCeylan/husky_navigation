**For testing :**<br>
ros2 topic pub /joy sensor_msgs/msg/Joy "{header: {stamp: {sec: 0, nanosec: 0}}, axes: [1.0, 0.0], buttons: [0, 0]}"<br><br>
**For running the code:**<br>
ros2 run kinematics_husky control_husky<br><br>
**For launching the hysky in gazebo:**<br>
ros2 launch clearpath_gz simulation.launch.py setup_path:=$HOME/clearpath_ws/<br><br>
**if joystick is connected as a hardware:**<br>
ros2 run joy joy_node
