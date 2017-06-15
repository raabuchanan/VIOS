# VIOS
**A visual-inertial version of [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2)**

**Original Authors:** [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2))

**VIOS Author:** Russell Buchanan 

# Prerequisites
This version requires all the same prerequisites as the original ORB_SLAM2:

* C++11 or C++0x Compiler
* [Pangolin](https://github.com/stevenlovegrove/Pangolin)
* [OpenCV](http://opencv.org) **Required at leat 2.4.3. Tested with OpenCV 2.4.11 and OpenCV 3.2**
* [Eigen3](http://eigen.tuxfamily.org) **Required at least 3.1.0**
* DBoW2 and g2o (Included in Thirdparty folder)
* ROS (Required for VIOS)
    * Currently VIOS only works with Stereo and ROS

# How to use

There are two possible ways to use VIOS depending on how you want to interface with ROS. The first option is to use the built in interface for debuging purposes. It will only work with VIOS but allows changes to be made and VIOS to be re-compiled. Also since there is no launch file you need to change things like topic names inside ros_stereo.cc and re-compile. 

The second option is to use the orb_slam_2_ros interface which I have forked [here](https://github.com/russellaabuchanan/orb_slam_2_ros) and modified to work with either VIOS of ORB_SLAM2. It does the same thing as the built-in interface plus publishes the robot pose as a tf message so it can work with [msf](https://github.com/ethz-asl/ethzasl_msf).


## [Option 1] Built-in interface
### Compiling

1. Clone the repository to ```<work_space>``` (doesn't have to be in a cattkin workspace):
```
git clone https://github.com/russellaabuchanan/VIOS
```

2. Run the script `build.sh` to build the *Thirdparty* libraries and *VIOS*.
```
cd <work_space>/VIOS
chmod +x build.sh
./build.sh
```
3. Add the path including Examples/ROS/VIOS to the ROS_PACKAGE_PATH environment variable. To do this copy the following line into your ~/.bashrc:
```
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:<work_space>/VIOS/Examples/ROS
```
4. Don't forget to re-source you .bashrc
```
source ~/.bashrc
```

5. Run `build_ros.sh` to build the ROS interface that comes with VIOS.
```
cd <work_space>/VIOS
chmod +x build_ros.sh
./build_ros.sh
```
### Running
First, open a terminal with Ctrl+Alt+T and launch ROS master:
```
roscore
```
Currently I have only tested on the [EuRoC Dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). Download one of the .bag files to any directoy. Since the VI-Sensor buffers IMU messages it is necesary to first process the EuRoC bag with the ROS [Cookbook](http://wiki.ros.org/rosbag/Cookbook). Once that is done run the cooked bag in any terminal with:
```
rosbag play <bag file name>.bag
```

Run VIOS in a third terminal using the correct settings file:
```
rosrun VIOS Stereo Vocabulary/ORBvoc.txt Settings/EuRoC.yaml true
```

## [Option 2] orb_slam_2_ros interface
Instead of cloning this repository go [here](https://github.com/russellaabuchanan/orb_slam_2_ros) and build this project. Running is similar to above but using roslaunch.

# Settings files
ORB_SLAM2 and VIOS rely on .yaml files which contain camera intrinsic and extrinsic data. In VIOS all the settings files can be found in the Settings folder. Currently I already have settings files for most MAVs in the ASL lab as well as for the EuRoC dataset however it might be necessary to create a new settings file. There should porbably be a tool made to automate this process for for now here is a step by step guide to build one from scratch.

**Note: The OpenCV documentation suggests it should work with either radial-tangential or equidistant distortion models but I've have difficulty getting it to work with equidistant so I recomend using rad-tan distortion model.**

1. First make sure you have following cam-chains from [Kalibr](https://github.com/ethz-asl/kalibr): 
    * Cam-chain between the two stereo cameras which we'll call cam0cam1-chain.
    * Cam-imu-chain from Cam0 (should be left camera) to imu0 (built into VI-Sensor) which we'll call cam0imu0-chain
    * Cam-imu-chain from Cam0 to FCU IMU which we'll call cam0fcu-chain
2. Make a copy of an existing Settings file and rename it to the new settings file you want to create.
3. The following data can be copied directly from the cam-chains into the settings file:
    * cam0cam1-chain, cam0, `distortion_coeffs` --> settings.yaml, `LEFT.D`, data (Might need to change cols depending on distortion model)
    * cam0cam1-chain, cam1, `distortion_coeffs` --> settings.yaml, `RIGHT.D`, data(Might need to change cols depending on distortion model)
    * cam0cam1-chain, cam0, `intrinsics` --> settings.yaml, `LEFT.K`, data (The order changes though. 0-->(0,0), 1-->(1,1), 2-->(0,2),3-->(1,2))
    * cam0cam1-chain, cam1, `intrinsics` --> settings.yaml, `RIGHT.K`, data (The order changes though. 0-->(0,0), 1-->(1,1), 2-->(0,2),3-->(1,2))
    * cam0cam1-chain, `T_cn_cnm1` --> settings.yaml, `T_CAM1_CAM0`, data
    * cam0imu0-chain, cam1, `T_cam_imu` --> settings.yaml, `T_CAM0_IMU`, data
    * cam0fcu-chain, cam1, `T_cam_imu` --> settings.yaml, `T_CAM0_BODY`, data


4. Next comment out RGIHT/LEFT R and P matrices and run the built-in ROS interface. This will detect that P and R are missing and genereate them using cv::stereoRectify and print them to screen.
5. Unfortunately we're not done, `R_l`, `P_l` and `R_r`,`P_r` now need to be copied from terminal into the settings file at `RGIHT.R`, `RIGHT.P` and `LEFT.R`,`LEFT.P`.
6. Now `Camera.fx`,`Camera.fy`,`Camera.cx`,`Camera.cy` need to be changed to the values in either LEFT/RIGHT P matrix (They should be the same values) at `P(0,0)`,`P(0,0)`,`P(0,2)`,`P(2,0)` respectively.
7. Finally `Camera.bf` (the rectified baseline times fx) can be copied from `RIGHT.P(1,0)` which should be some negative number. Change it to positive and copy it to `Camera.bf`.
8. That's it!



