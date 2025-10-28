# Edge Detection and 3D Robot Visualization using ROS

This package covers three tasks:

1. run edge detection on images (no ROS, just OpenCV),
2. wrap that into ROS (service + live topics),
3. project the detected edges into 3D using depth and visualize them together with the robot model in RViz.

Everything is written/tested on:
- Ubuntu 20.04 (WSL)
- ROS Noetic
- Python3 + OpenCV + cv_bridge


## 1. Task 1: edge detection on normal images (no ROS)

Files:
- `src/edge_detector.py`
- `scripts/run_edge_detection.py`
- `data/` (I put sample images here during testing)

What it does:
- reads an image
- converts to gray, blur, Canny edges
- uses HoughLinesP to get line segments
- draws those line segments back on top of the original image in green

How I tested it:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash

rosrun edge_detector_pkg2 run_edge_detection.py

## Task 2: Vision_ROS
1. Wrap the edge detector as a ROS service.
   - Service type: `srv/DetectEdges.srv`
   - Server: `scripts/edge_service_node.py`
   - Client: `scripts/edge_service_client.py`
   - Launch: `launch/edge_service.launch`

   The client loops through the images in `data/`, calls the service for each file, and writes out `<name>_edges.png`.

2. Run the edge detector live on RGB frames from the bag and visualize in RViz.
   - `scripts/rgb_edge_node.py` republishes:
     - `/edge_detector/rgb_in` (raw camera image)
     - `/edge_detector/rgb_edges` (camera image with detected edges drawn)
   - `scripts/edge_points_node.py` also projects edge pixels into 3-D using depth and camera intrinsics, and publishes a point cloud on `/edge_points` (type `sensor_msgs/PointCloud2`).
   - Launch: `launch/edge_bag_rviz.launch`

   I run this with:
   ```bash
   rosparam set /use_sim_time true
   roslaunch edge_detector_pkg2 edge_bag_rviz.launch
   rosbag play --clock -l data/edge_detection_data/withpointcloud.bag
   rviz  # then add /edge_detector/rgb_edges and /edge_points

## 3. Task 3 – Robot Model + 3-D Edge Projection

### Goal
Combine the robot model (Mira Picker) and the 3-D edge point cloud generated from RGB-D data in one RViz session.  
This demonstrates how the detected edges from Task 2 can be projected into real 3-D space and aligned with the robot coordinate frames.

---

### What Happens
- The robot model and TF frames are loaded using the provided `mira_picker` launch file.  
- My node `edge_points_node.py` keeps listening to the RGB and depth data coming from the bag file.  
- It extracts edges (using Canny + Hough), converts their pixel positions to 3-D coordinates with the camera intrinsics (fx, fy, cx, cy), and publishes them on `/edge_points`.  
- The cloud is continuously updated while the bag plays in a loop, giving a live visualization of how edges exist in 3-D space relative to the robot.

---

### Launch File
`launch/edge_task3.launch`

This launch file:
1. Includes the robot description and starts RViz with the robot model already loaded.
2. Starts my node `edge_points_node.py`, which publishes `/edge_points` (3-D point cloud) and `/edge_detector/rgb_edges` (2-D overlay image).

---

### How to Run

```bash
# terminal 1
roscore

# terminal 2
source ~/catkin_ws/devel/setup.bash
roslaunch edge_detector_pkg2 edge_task3.launch

# terminal 3
cd ~/catkin_ws/src/edge_detector_pkg2/data/edge_detection_data
rosbag play --clock -l withpointcloud.bag
---
**File Structure**

edge_detector_pkg2/
├── src/
│ └── edge_detector.py
├── scripts/
│ ├── run_edge_detection.py
│ ├── edge_service_node.py
│ ├── edge_service_client.py
│ ├── rgb_edge_node.py
│ └── edge_points_node.py
├── srv/
│ └── DetectEdges.srv
├── launch/
│ ├── edge_service.launch
│ ├── edge_bag_rviz.launch
│ └── edge_task3.launch
├── data/
│ └── image_1.png
  └── image_1_result.png
├── package.xml
├── CMakeLists.txt
├── README.md
---
