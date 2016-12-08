# WASP Autonomous systems challenge

# LTH, Team 2

Tutorials:
http://www.cas.kth.se/wasp/summerschool2016/

## How to install:

* Setup ROS: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
* Go in the src folder of your catkin workspace
* Clone this repository
* Install the following distribution packages:
	* python-opencv
* Install following ros packages:
	* ros-indigo-opencv3 (used by perception)
	* ros-indigo-cv-bridge (used by perception)
	* ros-indigo-image-transport (used by perception)
	* ros-indigo-tf (used by perception)
	* ros-indigo-apriltags (used by perception)
	* ros-indigo-parrot-arsdk (used by drone)
	* ros-indigo-camera-info-manager (used by drone)
	* ros-indigo-tf2-geometry-msgs (used by drone)
	* ros-indigo-roslint (used by drone)
* Go to your catkin workspace and run catkin_make
* Test and enjoy :)
