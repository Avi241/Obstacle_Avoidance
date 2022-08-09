# About  #
This repo is a platform for testing the Obstacle avoidance alogirthm on the drone. This work uses sjtu_drone as a simulation platform.
sjtu_drone is a quadrotor simulation program forked from ['tum_simulator'] (http://wiki.ros.org/tum_simulator) , which is developed with ROS + Gazebo.

# Requirements #
This package is compatible with ROS Melodic version (Ubuntu 18.04).
Please refer the following for the ['installation'](http://wiki.ros.org/Installation/Ubuntu) of ROS on your computer. Prefreable is ROS melodic with ubuntu 18.04 or ROS Noetic with Ubuntu 20.04.

# Download and Compiling this package#
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/Avi241/Obstacle_Avoidance.git
$ cd ~/catkin_ws
$ catkin build
```

Here <catkin_ws> is the path of the catkin workspace. Please refer to the [tutorial](http://wiki.ros.org/ROS/Tutorials) about how to create a catkin workspace in ROS.

# Run
The simplest way is calling after you have built the workspace successfully.

```
$ cd <where you check out the code>
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch sjtu_drone simple.launch
```
# Running with keyboard
In second terminal:

```
$ rosrun sjtu_drone drone_keyboard
```
# Steps 

Press Z to takeoff
Now you can control your drone with keyboars as shown in the Gui


# For obstacle avoidance you need to modify the following code 

Go to location ~/catkin_ws/src/Obstacle_Avoidance/sjtu-drone/scripts . Open the file ```obstacle_avoidance.py ``` now you can write your obstacle avoidane algorithms here in the main function of this code.