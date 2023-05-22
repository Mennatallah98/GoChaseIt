# GoChaseIt
[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)


https://github.com/Mennatallah98/GoChaseIt/assets/45118345/f172894b-6187-44c6-8fe7-33e57f743afe


## Overview

This project is the second project in Udacity Robotics Software Engineer nano degree where a robot was built and it has to follow a white ball detected by a camera using a differential drive and it is consisted of 2packgaes : <br>
	&emsp; &emsp;-**my_robot**:which contains the URDF of the robot with the attached sensors  <br>
	&emsp; &emsp;-**ball_chaser**:which contains 2 nodes one to drive the robot and the other to process the image to follow the white ball <br>

**Keywords:** Gazebo, Ros, URDF, C++, Plugins, image processing


**Author: Mennatallah Aly<br />**


The GoChaseIt package has been tested under [ROS] Melodic on Ubuntu 18.04. an Gazebo 9.0.0



#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/ethz-asl/ros_best_practices.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make

### Running in Docker

Docker is a great way to run an application with all dependencies and libraries bundles together. 
Make sure to [install Docker](https://docs.docker.com/get-docker/) first. 

First, spin up a simple container:

	docker run -ti --rm --name ros-container ros:noetic bash
	
This downloads the `ros:noetic` image from the Docker Hub, indicates that it requires an interactive terminal (`-t, -i`), gives it a name (`--name`), removes it after you exit the container (`--rm`) and runs a command (`bash`).

Now, create a catkin workspace, clone the package, build it, done!

	apt-get update && apt-get install -y git
	mkdir -p /ws/src && cd /ws/src
	git clone https://github.com/leggedrobotics/ros_best_practices.git
	cd ..
	rosdep install --from-path src
	catkin_make
	source devel/setup.bash
	roslaunch ros_package_template ros_package_template.launch

### Unit Tests

Run the unit tests with

	catkin_make run_tests_ros_package_template

### Static code analysis

Run the static code analysis with

	catkin_make roslint_ros_package_template

## Usage

Describe the quickest way to run this software, for example:

Run the main node with

	roslaunch ros_package_template ros_package_template.launch

## Config files

Config file folder/set 1

* **config_file_1.yaml** Shortly explain the content of this config file

Config file folder/set 2

* **...**

## Launch files

* **launch_file_1.launch:** shortly explain what is launched (e.g standard simulation, simulation with gdb,...)

     Argument set 1

     - **`argument_1`** Short description (e.g. as commented in launch file). Default: `default_value`.

    Argument set 2

    - **`...`**

* **...**

## Nodes

### ros_package_template

Reads temperature measurements and computed the average.


#### Subscribed Topics

* **`/temperature`** ([sensor_msgs/Temperature])

	The temperature measurements from which the average is computed.


#### Published Topics

...


#### Services

* **`get_average`** ([std_srvs/Trigger])

	Returns information about the current average. For example, you can trigger the computation from the console with

		rosservice call /ros_package_template/get_average


#### Parameters

* **`subscriber_topic`** (string, default: "/temperature")

	The name of the input topic.

* **`cache_size`** (int, default: 200, min: 0, max: 1000)

	The size of the cache.


### NODE_B_NAME

...


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/ethz-asl/ros_best_practices/issues).


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

