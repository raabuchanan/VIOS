# VIOS
## A visual-inertial version of [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2)
** Original Authors:** [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2))

** VIOS Author:** Russell Buchanan 

# Prerequisites
This version requires all the same prerequisites as the original ORB_SLAM2:

* C++11 or C++0x Compiler
* [Pangolin](https://github.com/stevenlovegrove/Pangolin)
* [OpenCV](http://opencv.org) **Required at leat 2.4.3. Tested with OpenCV 2.4.11 and OpenCV 3.2**
* [Eigen3](http://eigen.tuxfamily.org) **Required at least 3.1.0**
* DBoW2 and g2o (Included in Thirdparty folder)
* ROS (Required for VIOS)
    * Currently VIOS only works with Stero and ROS

# Building

Clone the repository:
```
git clone https://github.com/russellaabuchanan/VIOS
```

Run the script `build.sh` to build the *Thirdparty* libraries and *VIOS*. Run `build_ros.sh` to build the ROS interface that comes with VIOS.
```
cd VIOS
chmod +x build.sh
./build.sh
```

# Running
First, open a terminal with Ctrl+Alt+T and execute:
```
roscore
```

Currently I have only tested on the [EuRoC Dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). Download one of the .bag files to any directoy and play it with:
```
rosbag play <bag file name>.bag
```

Run VIOS:
```
rosrun VIOS Stereo Vocabulary/ORBvoc.txt Settings/EuRoC.yaml true
```



