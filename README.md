# Bobik_bridge

This ros2 node is supposed to be started in main computer. It mediates traffic between ros2 nodes on main computer (e.g. nav2, moveit) and computer installed on Bobik robot.

Communication between main computer and [Bobik robot](https://github.com/slesinger/bobik_driver) is done via 0MQ/UDP.

# Install
```bash
cd ~/ros2_foxy
clear && colcon build --packages-select bobik_bridge
```

# Testing
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard   #on main computer
ros2 run bobik_bridge bobik_bridge    #on main computer
bobik_driver    #on robot
```



# TODO
pomaly wifi ping
bluetooth na intel 7260, https://elinux.org/Jetson/Bluetooth
install opencv
install cuda tools, https://elinux.org/Jetson/Installing_CUDA
kinect, https://elinux.org/Jetson/Cameras
