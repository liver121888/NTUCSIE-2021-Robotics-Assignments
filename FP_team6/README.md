# NTUCSIE-2021-Robotics-Final_Project

 Mobile Lost and Found: MLF-6110
 
 
 All the .git submodule is removed for easier usage.
 
 On robot: src, ROS Master.
 
 On remote computer: src_remote, visualization.
 
 Robot: clbrobot with Jetson Nano + Orbbec Astra Pro + RPLIDAR-A1
 
## Usage

### Run Project Steps

#### Command Line

Run the following commands in separated terminals.

### `@clbrobot`

```bash
# Terminal 1
roslaunch realsense2_camera rs_camera_custom.launch 
# Terminal 2
roslaunch clbrobot navigate.launch
# or for map building
roslaunch clbrobot lidar_slam.launch
```

### `@remote`

```bash

# Terminal 1
rviz
# Terminal 2
source /home/<username>/<ws>/devel/setup.bash
rosrun lost_n_found lost_n_found.py
# Terminal 3
source /home/<username>/<ws>/devel/setup.bash
rosrun lost_n_found object_selection.py
# Terminal 4
source /home/<username>/<ws>/devel/setup.bash
roslaunch find_object_2d find_object_3d_custom.launch
```
  
src source: https://github.com/ykevin/rikirobot_project.git (Does not exist anymore)
