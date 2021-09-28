# Ackermann Steering Controller ROS2
This is ROS2 Controller for a ackermann steering drive mobile base.  
These codes were migrated from [ackermann_steering_controller](https://github.com/ros-controls/ros_controllers/tree/noetic-devel/ackermann_steering_controller) of ROS1.  
You can refer to the related documentation at this [link](http://wiki.ros.org/ackermann_steering_controller).  

## Installation  
```bash
$ apt install python3-vcstool # for vcs
$ cd /path/to/workspace/
$ wget https://raw.githubusercontent.com/ros-controls/ros2_control/foxy/ros2_control/ros2_control.repos
$ vcs import src < ros2_control.repos
$ git clone -b ros2-master https://github.com/ros-controls/control_toolbox.git ./src/
```

### TODO 
- [x] Trouble shooting to build
- [ ] Add the Odometry/Velocity Calculation on ackermann_steering_controller.cpp
- [ ] Add test codes for ackermann_steering_controller.cpp
- [ ] Update Odometry.hpp and .cpp
- [ ] Update documents (remarks, descriptions, etc.)
- [ ] Add Test Code
- [ ] PR