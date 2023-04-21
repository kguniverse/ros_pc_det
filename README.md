# ROS real-time 3D point cloud detection

## prequisites

- ROS2-humble
- ros-humble-vision-msgs
- ros-humble-vision-msgs-rviz-plugins

## build

### 1. install dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```

### 2. build

```bash
colcon build
. install/setup.bash
ros2 launch pc_det view_bin.launch.py
```

## screenshots

![screenshot](asset/Screenshot1.png)