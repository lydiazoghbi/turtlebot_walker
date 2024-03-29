# TurtleBot Walker Tutorial
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Licensing
3-Clause BSD License

Copyright (c) Lydia Zoghbi 2019

Permission is hereby granted, free of charge, to any person or organization
obtaining a copy of the software and accompanying documentation covered by
this license (the "Software") to use, reproduce, display, distribute,
execute, and transmit the Software, and to prepare derivative works of the
Software, and to permit third-parties to whom the Software is furnished to
do so, all subject to the following:

The copyright notices in the Software and this entire statement
must be included in all copies of the Software, in whole or in part, and
all derivative works of the Software, unless such copies or derivative
works are solely in the form of machine-executable object code generated by
a source language processor.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

## Overview
This is a ROS package which runs a Gazebo simulation for a Turtlebot robot, confined within a predefined space, with obstacles and walls that need to be avoided. BEWARE: the following code was developed on a virtual machine, so the performance and speed of the Gazebo simulation on a different machine might differ. It was a massive pain to run and debug the algorithm as the simulation was ridiculously slow. The package contains a node "walker_node.cpp", which subscribes to to the /scans topic in order to retrieve laser data, and subsequently sends commands to the topic /cmd_vel_mux/input/navi to move the robot accordingly.

## Assumptions & Dependencies
This code was developed and tested on Ubuntu 16.04.6 using ROS Kinetic. We are assuming that your catkin workspace ~/catkin_ws is already initialized, Gazebo is installed, and Turtlebot_Gazebo package downloaded.

## Installing and Building Instructions
Run the following commands in the terminal:
```
cd ~/catkin_ws/src
git clone --recursive https://github.com/lydiazoghbi/turtlebot_walker.git
```
To build the workspace, run:
```
cd ~/catkin_ws
catkin_make
```

## Running the Demo through a Launch file
In order to run the simulation, simply type the following in a new terminal:
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch turtlebot_walker walker.launch broadcast:=1 rate:=20 record:=0
```
This should open a Gazebo window, load in an environment that I created, and start moving the Turtlebot within the environment. 

## Record a ROSBag File
In the previous command, the recording option for the rosbag file is disabled (it is false by default, too). In order to enable the recording of a rosbag file, run the following in a terminal:
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch turtlebot_walker walker.launch broadcast:=1 rate:=20 record:=1
```

## Play a ROSbag file
A rosbag file that I generated is present in the results directory. Considering how slow my simulations were, I recorded the rosbag file for a little over 30 seconds, to ensure that the algorithm's functionality is demonstrated properly. The rosbag file (even while excluding camera readings) was pretty large, so I had to compress it. The file upload to this Github repository is a compressed version. In order to decompress it, simply type the following:
```
cd ~/catkin_ws/src/turtlebot_walker/results/
rosbag decompress *.bag
```
This will create an additional rosbag file (named walker.bag), and rename the original compressed file to walker.ori.bag. To play the rosbag file type the following after killing all other nodes (note, a Gazebo simulation will not open):
```
roscore
```
In a new terminal, type:
```
cd ~/catkin_ws/src/turtlebot_walker/results
rosbag play ./walker.bag
```
If you wish to examine the recorded data, in a new terminal type in the following:
```
rostopic echo /scan
```
This will show the readings obtained from the laser sensor. Note that when the sensor display a nan value, this means that no obstacle is detected within the maximum working range of the sensor. If you wish to examine the velocities published to the robot, type in a new terminal:
```
rostopic echo /cmd_vel_mux/input/navi
```
You should see in the terminal a list of the velocities that were published to the Turtlebot. Make sure that you decompress the rosbag file before running the previous commands.
