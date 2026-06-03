# GraspNet ROS 2 Node

## Overview
This package implements a ROS 2 node for GraspNet inference using TensorRT.
It subscribes to a point cloud topic, runs inference, and publishes sampled points and grasp markers.

## Build
```bash
cd ~/graspnet-baseline/ros_infer
source /opt/ros/jazzy/setup.bash
colcon build --base-paths graspnet_ros
```

## Usage
1. Source the workspace:
   ```bash
   source install/setup.bash
   ```

2. Run the node:
   ```bash
   ros2 run graspnet_ros graspnet_node --ros-args \
       -p engine_path:=/home/lyz/graspnet-baseline/graspnet.trt \
       -p plugin_path:=/home/lyz/graspnet-baseline/tensorrt_plugins/build/libfps_plugin.so \
       -p num_point:=20000 \
       -p frame_id:=camera_color_optical_frame
   ```

## Topics
- **Input**: `/yoloe_multi_text_prompt/pointcloud_colored` (`sensor_msgs/msg/PointCloud2`)
- **Output**: 
  - `sampled_cloud` (`sensor_msgs/msg/PointCloud2`): The downsampled point cloud used for inference.
  - `grasp_markers` (`visualization_msgs/msg/MarkerArray`): Visual markers for grasps.

## Visualization
Open RViz2 and add:
- PointCloud2 topic: `sampled_cloud`
- MarkerArray topic: `grasp_markers`
Ensure the Fixed Frame matches `frame_id` (default: `camera_color_optical_frame`).
