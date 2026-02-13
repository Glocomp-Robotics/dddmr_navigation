# 🤖 dddmr_navigation

## Glocomp Specific Changes
-> Added params for imu and odom topics for odom3s node so you can change your odom and imu topics
-> Added lego_loam_b2.launch specifically for Glocomp B2

## Installation instructions
```
sudo apt-get install -y apt-utils && apt-get install -y curl gnupg2 && apt-get install -y lsb-release && apt-get clean all
sudo apt-get install -y iputils-ping net-tools htop build-essential
sudo apt install -y zip
sudo apt install -y libmetis-dev
sudo apt install -y python3-pip
sudo apt install -y libopencv-dev python3-opencv
sudo apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio
sudo apt install -y libfreeimage-dev
sudo apt install -y gdb
sudo apt install -y ros-humble-apriltag-msgs
```


```
# Gtsam install
cd /tmp && sudo git clone https://github.com/borglab/gtsam.git && cd gtsam && sudo git checkout 4.2a9
cd /tmp/gtsam && sudo mkdir build && cd build && sudo cmake -DCMAKE_BUILD_TYPE=Release -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_USE_SYSTEM_METIS=ON -DPCL_ENABLE_MARCHNATIVE=OFF .. && sudo make install -j$(nproc)
```

```
# Pcl install
sudo apt install -y libpcap-dev
sudo apt install -y libusb-1.0-0-dev
cd /tmp && sudo git clone https://github.com/PointCloudLibrary/pcl.git && cd pcl && sudo git checkout pcl-1.15.0
cd /tmp/pcl && sudo mkdir build && cd build && sudo cmake -DPCL_ENABLE_AVX=OFF -DPCL_ENABLE_SSE=OFF -DPCL_ENABLE_MARCHNATIVE=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr .. && sudo make install -j$(nproc)
```

```
#install apriltag
cd /tmp && sudo git clone https://github.com/AprilRobotics/apriltag.git && cd apriltag && sudo git checkout 3806edf38ac4400153677e510c9f9dcb81f472c8 && sudo cmake -B build -DCMAKE_BUILD_TYPE=Release && sudo cmake --build build --target install
```

## 🚀 Now we have a dog!
<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_navigation/lite3_down_obstacle_avoidance.gif" width="700" height="420"/>
</p>

---

## 🚀 Big update!
We’ve just integrated a Gazebo models using Unitree-go2 with the DDDMR Navigation Stack, unlocking true 3D navigation for simulation and testing. Using the latest quadruped robots go2 combined with our advanced stack, you can explore navigation capabilities that go far beyond traditional 2D navigation frameworks.

👉 Jump in, simulate, and experience features that Nav2 alone can’t achieve — multi-level mapping, ramp navigation, and obstacle handling in complex environments. 

[👾 Let's play go2 using dddmr navigation](https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_beginner_guide)

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/3d_nav_gz.gif" width="700" height="420"/>
</p>

---

> [!NOTE]
> DDDMR Navigation Stack is designed to solve the issues that [Nav2](https://github.com/ros-navigation/navigation2) not able to handle: such as multi-layer floor mapping and localization, path planning in stereo structures and percption markings and clearings in a 3D point cloud map.

<table align='center'>
  <tr width="100%">
    <td width="40%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_navigation/multilevel_map.gif" width="400" height="260"/><p align='center'>Multilevel map</p></td>
    <td width="40%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_navigation/obstacle_avoidance.gif" width="400" height="260"/><p align='center'>Obstacle avoidance on ramps</p></td>
  </tr>
  <tr width="100%">
    <td width="40%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_navigation/mapping_navigating.gif" width="400" height="260"/><p align='center'>Navigating while mapping</p></td>
    <td width="40%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_semantic_segmentation/dddmr_semantic_segmentation_to_pointcloud.gif" width="400" height="260"/><p align='center'>Semantic segmentation and navigation (stay tuned🔥)</p></td>
  </tr>
</table>

DDDMR navigation (3D Mobile Robot Navigation) is a navigation stack allows users to map, localize and autonomously navigate in 3D environments. 

Below figure shows the comparison between 2D navigation stack and DDD(3D) navigation.
Our stack is a total solution for a mobile platform to navigate in 3D environments. There are plenty advantages for choosing DDD navigation:
  
✨ The standard procedures of DDD mobile robots and 2D mobile robots are the same, make it easier for 2D navigation stack users to transit to DDD navigation without difficulties:
  1. Mapping and refining the map using our packages and tools.
  2. Turn off mapping, use MCL to localize the robot by providing an initial pose.
  3. Send a goal to the robot, the robot will calculate the global plan and avoid obstacles using local planner.

✨ DDD navigation is no longer suffered from terrain situations. For example, ramps in factories or wheelchair accessible.

✨ DDD navigation has been well tested is many fields and is based on the cost-effective hardware, for example, 16 lines lidar, intel NUC/Jetson Orin Nano and consumer-grade imu. We are trying to make the solution as affordable as possible.

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_navigation/navigation_diagram.png" width="780" height="560"/>
</p>

🤝 We would like to thank the contributors and users of [Navigation](https://github.com/ros-planning/navigation) and [Navigation2](https://github.com/ros-navigation/navigation2), ddd navigation stands on your shoulder!

😵‍💫😵‍💫😵‍💫😵‍💫😵‍💫😵‍💫😵‍💫😵‍💫😵‍💫😵‍💫 I have a robot, but where to start?[Click me to see the beginner's guide](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/README.md)😵‍💫😵‍💫😵‍💫😵‍💫😵‍💫😵‍💫😵‍💫😵‍💫😵‍💫😵‍💫
## 🏁 Detail documentations for each package
<details><summary> <b>💡 Click me to see Mapping</b> </summary>
https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_lego_loam
</details>
<details><summary> <b>💡 Click me to see Localization</b> </summary>
https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_mcl_3dl
</details>
<details><summary> <b>💡 Click me to see Perception</b> </summary>
https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_perception_3d
</details>
<details><summary> <b>💡 Click me to see Global planner</b> </summary>
https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_global_planner
</details>
<details><summary> <b>💡 Click me to see Local planner</b> </summary>
https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_local_planner
</details>
<details><summary> <b>💡 Click me to see Move base</b> </summary>
https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_p2p_move_base
</details>

## Demonstrations of DDD navigation functions
<table align='center'>
  <tr width="100%">
    <td width="50%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_navigation/mapping.gif" width="400" height="260"/><p align='center'>3D mapping</p></td>
    <td width="50%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_navigation/global_planner.gif" width="400" height="260"/><p align='center'>3D global planning</p></td>
  </tr>
  <tr width="100%">
    <td><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_navigation/local_planner.gif" width="400" height="260"/><p align='center'>3D local planning</p></td>
    <td><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_navigation/navigation.gif" width="400" height="260"/><p align='center'>3D navigation</p></td>
  </tr>
  <tr width="100%">
    <td><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/perception_3d/scanning_lidar_demo.gif" width="400" height="260"/><p align='center'>Support vairant sensors (Unitree G4)</p></td>
    <td><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/perception_3d/multi_depth_camera_demo.gif" width="400" height="260"/><p align='center'>Support vairant sensors (Depth Camera)</p></td>
  </tr>
</table>

