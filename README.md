# ROS2+KITTI:
## Objet detection & Stereo depth estimation
![ROS](https://img.shields.io/badge/ros2-galactic-blue)
![Ubuntu](https://img.shields.io/badge/ubuntu-20.04-blue)
![Python](https://img.shields.io/badge/python-3.8-blue)
- Visualize raw images and point cloud in ROS2
- Implement **object detection** algorithm YOLOv5
- Implement **stereo depth estimation** algorithm SGBM

***
# Build and Run
### Clone the repository to local
```bash
git clone https://github.com/junmeng6025/ros2_kitti.git
```
### Download the KITTI dataset and extract. 
The raw data should be organized as:
```bash
ros2_kitti
├── colcon_ws
│   └──...
│
├──rawdata #----------------------------- create this folder yourself
│   ├── 2011_09_26
│   │   ├── 2011_09_26_drive_0005_sync
│   │   │   ├── image_00
│   │   │   ├── image_01
│   │   │   ├── image_02
│   │   │   ├── image_03
│   │   │   ├── oxts
│   │   │   └── velodyne_points
│   │   ├── calib_cam_to_cam.txt
│   │   ├── calib_imu_to_velo.txt
│   │   └── calib_velo_to_cam.txt
│   │
│   ├── 2011_09_26_calib.zip #----------- download from KITTI website
│   └── 2011_09_26_drive_0005_sync.zip #- download from KITTI website
│
└── README.md
```
### Build the workspace
- use colcon build command to generate executable files automatically
  ```bash
  cd ros2_kitti/colcon_ws
  colcon build
  ```
- after build, copy the folder `/data` in
    ```bash
    /src/kitti_ros2/yolov5
    ```
    to
    ```bash
    /install/kitti_ros2/lib/python3.8/site-packages/yolov5
    ```
### Launch the nodes
source the `setup.sh` to current terminal
```bash
# stay in the directory /ros2_kitti/colcon_ws
source install/setup.sh
```
There are two options, select one:
- **A:**  Visualize KITTI data in Rviz **without** detection or depth estimation
  ```bash
  ros2 launch kitti_ros2 kitti_visualization.launch.py
  ```
- **B:**  Implement the detection and depth estimation
  ```bash
  ros2 launch kitti_ros2 kitti_stereo_detect.launch.py 
  ```
# Modify and git push
*To keep the repository light-weighted, delete YOLO model files such as `yolov5x.pt` in `colcon_ws/src/kitti_ros2/yolov5/` before **git add**. These would be automatically downloaded when launch for the first time.*
