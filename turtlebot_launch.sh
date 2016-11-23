#!/bin/sh

#Force killing children when exiting
trap "trap - TERM && kill -- -$$" INT TERM EXIT

#Minimal launch
echo "ROS minimal launch"
roslaunch turtlebot_bringup minimal.launch --screen > roslaunch.log 2>&1 &
sleep 30

#Lidar
echo "ROS lidar launch"
roslaunch rplidar_ros rplidar.launch > lidar_launch.log 2>&1 &
sleep 10
rosrun rplidar_ros rplidarNodeClient > lidar_ros.log 2>&1 &
sleep 10

#Static transform
echo "ROS static transform launch"
rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 base_link laser 100 > tf.log 2>&1 &
sleep 10

#Navigation
echo "ROS navigation launch"
roslaunch turtlebot_navigation amcl_demo.launch > navigation.log 2>&1 &
sleep 10

echo "ROS: All launched !"
echo "Press CTRL+C to stop everything"
while true; do
    : #Nothing
done
