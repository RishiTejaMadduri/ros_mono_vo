# C++ Based Monocular Visual Odometry 
Yet another MonoVO implementation. Written in C++. There are 2 branches to this repo. The main branch relies on ROS topics for its input image stream and calibration information and can be very easily changed to accept input from usb_cam and the vo-kitti branch takes input from kitti dataset.


## Sample Results:
![sample_results](images/result.gif)

## Instructions
```
cd catkin_ws/src
```
```
git clone https://github.com/RishiTejaMadduri/ros_mono_vo.git
```
```
cd ..
```
```
catkin_make
```
```
roslaunch roslaunch ros_mono_vo ros_mono_vo.launch 
```

## References:
While I built the pipeline from scract, the idea for publishing the odom poses was borrowed from https://github.com/edward0im/simple_mono_vo_ros



