# AuE893: Autonomy Science and System

This repository contains the code and assets for Clemson Univeristy course AuE893: Autonomy Science and System for Spring 2020 semester. 

## Team Number: 04
		Atef Emran 
		Anirudha Sundar 
		Abhijeet Mordekar  
		Kartik Loya 
		Ziyue Feng
 
## Final Project


	The final project is an obstacle course which has to be implemented by Turtlebot3 using ROS. It is first implemented in the Gazebo and then the real world. It mainly contains of 4 maneouvers.

		1. Wall follower and Obstacle avoidance
			Turtlebot maintains safe front distance from obstacle and maneuvers through course until it finds yellow lanes. In this it manipulates Lidar values for the design of controller.
		
		3. Line following
			Turtlebot follows the yellow line using image processor (open-cv package). The controller uses lane centroid to control the angular speed of the bot.

		4. Traffic sign detection
			Darknet package is used for traffic sign detection. Traffic sign callback functions stop the turtlebot for 3 seconds.

		5. April Tag follower
			April_ros package is used to detect apriltags and then its co-ordinates are used to design the controller for tracking the april tag
	
This project is dependent on the following packages:
1. [Turtlebot3 package](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)	
2. [Turlebot3 simulation](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)
3. [open cv](https://github.com/ros-perception/vision_opencv)
4. [Apriltag ROS](https://github.com/AprilRobotics/apriltag_ros)
5. [Darknet Ros(YOLO)](https://github.com/leggedrobotics/darknet_ros)

## Commands to Execute
	1. For Gazebo simulation
		1. '$ roslaunch project_turtlebot_maneouvres turtlebot3_maneouvres.launch'
		2. '$ rosrun project_turtlebot_maneouvres keyboard_teleop_apriltag.py'    -- for operating the 2nd turtlebot in the gazebo world.
	
	2. For real world
		1. Bring up the turtleot
		2. '$ roslaunch project_turtlebot_maneouvres turtlebot3_maneouvres_real.launch'
		3. '$ rosrun image_transport republish compressed in:=/raspicam_node/image raw out:=camera/rgb/image_raw'     -- tp republish raspicam_node to camer/rgb
	
##  Results

### Gazebo simulation
1. Wall following and obstacle avoidance in gazebo
	
	![Wall following and obstacle avoidance in gazebo](https://media2.giphy.com/media/si6g6yiLhNC97hD4iM/giphy.gif)

2. Line following and stop sign detection in gazebo

	![Line following and stop sign detection in gazebo](https://media1.giphy.com/media/W8vedwATymTwBeuzEl/giphy.gif)

3. AprilTag tracking in gazebo

	![AprilTag tracking in gazebo](https://media.giphy.com/media/tGV4TY5m9PdG6Ycu6H/giphy.gif)


### Real World Simulation
1. wall following and obstacle avoidance in real

	![wall following and obstacle avoidance in real](https://media.giphy.com/media/K16mv68PfDrX2IFCUJ/giphy-downsized.gif)

2. Line following and stop sign detection in real

	![Line following and stop sign detection in real](https://media.giphy.com/media/5HeiJdOfLHx1OjHdyA/giphy-downsized.gif)

3. AprilTag tracking in real

	![AprilTag tracking in real](https://media.giphy.com/media/GZVnmHQRJpgRdRydyt/giphy-downsized.gif)

	

