# ROS real-time 3D point cloud detection

## prequisites

- ROS2(humble, foxy)
- vision-msgs
- vision-msgs-rviz-plugins
- tf-transformation
- sensor-msgs
- mmdet3d(temporary) 1.0.0-rc6

## build

### 1. install dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Note:

If you are using Foxy distro instead of Humble, you need to manually build these dependencies:
- vision-msgs
- vision-msgs-rviz-plugins


### 2. rewrite config

In the patch 0.0.2, we introduced the ros2 params feature in order to avoid verbose build. You can rewrite the config file `config.yaml` in `conf/` directory.

#### params

1. pub_nuscenes_bin
    - time_period

        Time period for releasing the point cloud. In nuscenes source, it is the frequency of publishing point cloud; in real-time lidar scan, it is the scan frequency.

    - nus_lidar_path

        The path to the pointcloud folder. It should end with 'LIDAR_TOP'.
        
2. infer_node
    - mmdet3d_path

        The absolute path to the mmdet3d package.

    - config_file

        The relative path of config file in mmdet3d path.

    - checkpoint_file
    
        The absolute path to the checkpoint file.(.pth)


### 3. build

```bash
colcon build --packages-select pc_det
. install/setup.bash
```

## run

1. nuscenes demo source
``` bash
ros2 launch pc_det nuscenes_demo_launch.py
```

## screenshots

![screenshot](asset/Screenshot1.png)
