# GoChaseIt
[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)


https://github.com/Mennatallah98/GoChaseIt/assets/45118345/f172894b-6187-44c6-8fe7-33e57f743afe


## Overview

This project is the second project in Udacity Robotics Software Engineer nano degree where a robot was built and it has to follow a white ball detected by a camera using a differential drive and it is consisted of 2 packgaes : <br>
	&emsp; &emsp;-**my_robot**:which contains the URDF of the robot with the attached sensors  <br>
	&emsp; &emsp;-**ball_chaser**:which contains 2 nodes one to drive the robot and the other to process the image to follow the white ball <br>

**Keywords:** Gazebo, Ros, URDF, C++, Plugins, image processing


**Author: Mennatallah Aly<br />**


The GoChaseIt package has been tested under [ROS] Melodic on Ubuntu 18.04. an Gazebo 9.0.0



## Building from Source

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_ws/src
	sudo apt update
	git clone https://github.com/Mennatallah98/GoChaseIt.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make
	
Source the workspace by adding this line .bashrc

	source ~/catkin_ws/devel/setup.bash


## Usage

In a new terminal

Open the world the world in gazebo and rviz with the rbot included

	roslaunch my_robot world.launch

In another window 

Run the launch file starting the driving and the image processing nodes

	roslaunch ball_chaser ball_chaser.launch


## Launch files

* **robot_description.launch:** Runs the robot file and starts the joint publisher robot state publisher.

* **world.launch:** Starts rviz and gazebo with the customized world , spawns the robot and launches robot_description.

* **ball_chaser.launch:** runs the nodes responsible for driving the and detecting the white ball.
 
## Nodes

### drive_bot

Sends speed commands to the robot according to the place of the dected white ball.


#### Published Topics

* **`/cmd_vel`** ([geometry_msgs/Twist])

	The speed to move the robot.
	
		
### process_image

Detects the white ball in the camera frame and determines its place with respect to the robot.


#### Subscribed Topics

* **`/camera/rgb/image_raw`** ([sensor_msgs/Image])

	The camera image from which the ball is detected.

#### Services

* **`DriveToTarget`** ([ball_chaser/DriveToTarget])

	Sends the robot velocities to drive_bot according to the position of the ball

## Structure

	└── GoChaseIt                                   # Go Chase It Project
	    ├── ball_chaser                             # ball_chaser package
	    │   ├── CMakeLists.txt                      # compiler instructions
	    │   ├── include
	    │   │   └── ball_chaser
	    │   ├── launch                              # launch folder for launch files 
	    │   │   └── ball_chaser.launch
	    │   ├── package.xml                         # package info 
	    │   ├── src                                 # source folder for C++ scripts
	    │   │   ├── drive_bot.cpp
	    │   │   └── process_image.cpp
	    │   └── srv                                 # service folder for ROS services
	    │       └── DriveToTarget.srv
	    ├── my_robot                                # my_robot package
	    │   ├── CMakeLists.txt                      # compiler instructions
	    │   ├── launch                              # launch folder for launch files 
	    │   │   ├── robot_description.launch
	    │   │   └── world.launch
	    │   ├── meshes                              # meshes folder for sensors
	    │   │   └── hokuyo.dae
	    │   ├── package.xml                         # package info
	    │   ├── urdf                                # urdf folder for xarco files
	    │   │   ├── my_robot2.gazebo
	    │   │   ├── my_robot2.xacro
	    │   │   ├── my_robot.gazebo
	    │   │   └── my_robot.xacro
	    │   └── worlds                              # world folder for world files
	    │       └── myworld.world
	    └── README.md

[ROS]: http://www.ros.org
[geometry_msgs/Twist]: http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html
[sensor_msgs/Image]: http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html
[ball_chaser/DriveToTarget]: https://github.com/Mennatallah98/GoChaseIt/blob/main/ball_chaser/srv/DriveToTarget.srv
