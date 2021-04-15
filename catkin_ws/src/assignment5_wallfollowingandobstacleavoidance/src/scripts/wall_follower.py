#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import struct
import numpy as np

class avoidance:

    def __init__(self):
        # unique node (using anonymous=True).
        rospy.init_node('avoidance', anonymous=True)

        # Publisher which will publish to the topic '/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.new_measurment)
        self.rate = rospy.Rate(10)
        self.max= 3.5

    def new_measurment(self,lidar_readings):
        self.rate.sleep()

        #front_front
        front_front=(np.mean(lidar_readings.ranges[1:15])+np.mean(lidar_readings.ranges[345:360]))/2
        if (front_front<lidar_readings.range_max):
            self.front_front = front_front
        else:
            self.front_front = self.max

        #front_left
        front_left=np.mean(lidar_readings.ranges[16:60])
        if (front_left<lidar_readings.range_max):
            self.front_left = front_left
        else:
            self.front_left = self.max

        #front_right
        front_right=np.mean(lidar_readings.ranges[300:344])
        if (front_right<lidar_readings.range_max):
            self.front_right = front_right
        else:
            self.front_right = 0.999*self.max

        #front
        front=(np.mean(lidar_readings.ranges[1:5])+np.mean(lidar_readings.ranges[355:360]))/2
        if (front<lidar_readings.range_max):
            self.front = front
        else:
            self.front = self.max

        #right
        right=np.mean(lidar_readings.ranges[265:275])
        if (right<lidar_readings.range_max):
            self.right = right
        else:
            self.right = self.max

        #left
        left=np.mean(lidar_readings.ranges[85:95])
        if (left<lidar_readings.range_max):
            self.left = left
        else:
            self.left = self.max

    def move(self):
        rospy.sleep(3)      #Needed while running from launch file
        vel_msg = Twist()

        #the input paramters; you have the choice to make them a user input
        # speed_lin = input("Enter the desired Linear Speed (0.3 is prefered to match the critical distances):")
        # speed_ang = input("Enter the desired Angular Speed (0.3 is prefered to match the critical distances):")
        speed_lin = 0.3
        speed_ang = 0.5

        speed_lin = float(speed_lin)
        speed_ang = float(speed_ang)


        #Calculations based on the inputs
        threshold = 1
        threshold_ang = 1.2
        critical = 0.6
        critical_side = 0.05

        # The indefinite loop
        i=0
        while i<5:
            # Keep moving!
            vel_msg.linear.x = speed_lin
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

            # Let's descover what is surrounding us! (kp value)
            diff = self.front_left-self.front_right

            while self.front_front<threshold and self.front_front>critical :
                # Reduce linear speed and rotate
                vel_msg.linear.x = speed_lin
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = diff*speed_ang/abs(diff)
                self.velocity_publisher.publish(vel_msg)
                self.rate.sleep()
                diff = self.front_left-self.front_right
                # what if it became critical from any angle for some reason stop this loop
                if self.front_front<critical or self.front_left<critical or self.front_right<critical:
                    break

            #otherwise : keep adjusting!
            while (self.front_left<threshold_ang and self.front_left>critical) or (self.front_right<threshold_ang and self.front_right>critical):
                # Reduce linear speed and rotate
                vel_msg.linear.x = speed_lin
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = diff*speed_ang/abs(diff)
                self.velocity_publisher.publish(vel_msg)
                self.rate.sleep()
                diff = self.front_left-self.front_right
                # what if it became critical from any angle for some reason stop this loop
                if self.front_front<critical or self.front_left<critical or self.front_right<critical:
                    break

            #what if it is very close!! --> okay now it is serious!
            while (self.front_front<critical or self.front_left<critical or self.front_right<critical):
                # Stop and rotate
                vel_msg.linear.x = 0.8*speed_lin
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = 2*diff*speed_ang/abs(diff)
                self.velocity_publisher.publish(vel_msg)
                self.rate.sleep()
                print(self.front)

        # ctrl + C, the node will stop.
        rospy.spin()

try:
    x = avoidance()
    x.move()

except rospy.ROSInterruptException:
    pass

