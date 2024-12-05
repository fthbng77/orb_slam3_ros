# ORB-SLAM3-ROS

A ROS implementation of [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) V1.0 that focuses on the ROS part.

This package uses ```catkin build```. Tested on Ubuntu 20.04.
## 1. Prerequisites
### Eigen3
```
sudo apt install libeigen3-dev
```
### Pangolin
```
cd ~
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build && cd build
cmake ..
make
sudo make install
```
### OpenCV
Check the OpenCV version on your computer (required [at least 3.0](https://github.com/UZ-SLAMLab/ORB_SLAM3)):
```
python3 -c "import cv2; print(cv2.__version__)" 
```
On a freshly installed Ubuntu 20.04.4 LTS with desktop image, OpenCV 4.2.0 is already included. If a newer version is required (>= 3.0), follow [installation instruction](https://docs.opencv.org/4.x/d0/d3d/tutorial_general_install.html) and change the corresponding OpenCV version in `CMakeLists.txt`

### (Optional) `hector-trajectory-server`
Install `hector-trajectory-server` to visualize the real-time trajectory of the camera/imu. Note that this real-time trajectory might not be the same as the keyframes' trajectory.
```
sudo apt install ros-[DISTRO]-hector-trajectory-server
```
## 2. Installation
```
cd ~/catkin_ws/src
git clone https://github.com/thien94/orb_slam3_ros.git
cd ../
catkin build
```

## 3. Run Examples

### Mono mode with [NTU VIRAL](https://ntu-aris.github.io/ntu_viral_dataset/)'s [`eee_01.bag`](https://researchdata.ntu.edu.sg/api/access/datafile/68133):

```
# In one terminal:
roslaunch orb_slam3_ros ntuviral_mono.launch
# In another terminal:
rosbag play eee_01.bag -s 50 # The UAV starts moving at t~50s
```

### Mono-inertial mode with [EuRoC](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)'s [`MH_01_easy.bag`]( http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.bag):
```
# In one terminal:
roslaunch orb_slam3_ros euroc_mono_inertial.launch
# In another terminal:
rosbag play MH_01_easy.bag
```
### Stereo-inertial mode with [TUM-VI](https://vision.in.tum.de/data/datasets/visual-inertial-dataset)'s [`dataset-corridor1_512_16.bag`](https://vision.in.tum.de/tumvi/calibrated/512_16/dataset-corridor1_512_16.bag)
```
# In one terminal:
roslaunch orb_slam3_ros tum_vi_stereo_inertial.launch
# In another terminal:
rosbag play dataset-corridor1_512_16.bag
```
### RGB-D mode with [TUM](http://vision.in.tum.de/data/datasets/rgbd-dataset/download)'s [`rgbd_dataset_freiburg1_xyz.bag`](https://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.bag)
```
# In one terminal:
roslaunch orb_slam3_ros tum_rgbd.launch
# In another terminal:
rosbag play rgbd_dataset_freiburg1_xyz.bag
```
- **Note**: change `TUMX.yaml` to `TUM1.yaml`,`TUM2.yaml` or `TUM3.yaml` for freiburg1, freiburg2 and freiburg3 sequences respectively.


### Save and load map 

The map file will have `.osa` extension, and is located in the `ROS_HOME` folder (`~/.ros/` by default).
#### Load map:
- Set the name of the map file to be loaded with `System.LoadAtlasFromFile` param in the settings file (`.yaml`).
- If the map file is not available, `System.LoadAtlasFromFile` param should be commented out otherwise there will be error.
#### Save map:
- **Option 1**: If `System.SaveAtlasToFile` is set in the settings file, the map file will be automatically saved when you kill the ros node.
- **Option 2**: You can also call the following ros service at the end of the session
```
rosservice call /orb_slam3/save_map [file_name]
```

## 4. ROS topics, params and services
### Subscribed topics
- `/camera/image_raw` for Mono(-Inertial) node
- `/camera/left/image_raw` for Stereo(-Inertial) node
- `/camera/right/image_raw` for Stereo(-Inertial) node
- `/imu` for Mono/Stereo/RGBD-Inertial node
- `/camera/rgb/image_raw` and `/camera/depth_registered/image_raw` for RGBD node
### Published topics
- `/orb_slam3/camera_pose`, left camera pose in world frame, published at camera rate
- `/orb_slam3/body_odom`, imu-body odometry in world frame, published at camera rate
- `/orb_slam3/tracking_image`, processed image from the left camera with key points and status text
- `/orb_slam3/tracked_points`, all key points contained in the sliding window
- `/orb_slam3/all_points`, all key points in the map
- `/orb_slam3/kf_markers`, markers for all keyframes' positions
- `/tf`, with camera and imu-body poses in world frame
### Params
- `voc_file`: path to vocabulary file required by ORB-SLAM3
- `settings_file`: path to settings file required by ORB-SLAM3
- `enable_pangolin`: enable/disable ORB-SLAM3's Pangolin viewer and interface. (`true` by default)
