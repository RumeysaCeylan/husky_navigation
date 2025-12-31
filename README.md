**For testing :**
ros2 topic pub /joy sensor_msgs/msg/Joy "{header: {stamp: {sec: 0, nanosec: 0}}, axes: [1.0, 0.0], buttons: [0, 0]}"
**For running the code:**
ros2 run kinematics_husky control_husky
**For launching the hysky in gazebo:**
ros2 launch clearpath_gz simulation.launch.py setup_path:=$HOME/clearpath_ws/
