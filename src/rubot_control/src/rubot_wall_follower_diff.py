#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class WallFollower:
    def __init__(self):
        rospy.init_node("simple_nav")

        # Parameters
        self.safe_distance = rospy.get_param("~safe_distance", 0.5)
        self.max_distance = rospy.get_param("~max_distance", 0.5)
        self.forward_speed = rospy.get_param("~forward_speed", 0.2)
        self.backward_speed = rospy.get_param("~backward_speed", -0.1)
        self.rotation_speed = rospy.get_param("~rotation_speed", 1.0)

        self.fright_distance = 999 
        self.right_distance = 999
        self.bright_distance = 999
        self.bleft_distance = 999
        self.left_distance = 999
        self.fleft_distance = 999

        # Velocity publisher
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback, queue_size=10)

        self.closest_distance = float("inf")
        self.closest_side = None  # 'left' or 'right'
        self.rate = rospy.Rate(10)  # 10 Hz

    def laser_callback(self, scan):
        ranges = np.array(scan.ranges)
        ranges[np.isinf(ranges)] = 999  # Replace inf values with a large number

        sector = len(ranges)//8

        # Divide the ranges into left and right
        fright_ranges = ranges[(3 * sector):(4 * sector)]
        right_ranges = ranges[sector:(3 * sector)]
        bright_ranges = ranges[0:sector]
        bleft_ranges = ranges[(7 * sector):]
        left_ranges = ranges[(5 * sector):(7 * sector)]
        fleft_ranges = ranges[(4 * sector):(5 * sector)]

        self.fright_distance = np.min(fright_ranges)
        self.right_distance = np.min(right_ranges)
        self.bright_distance = np.min(bright_ranges)
        self.bleft_distance = np.min(bleft_ranges)
        self.left_distance = np.min(left_ranges)
        self.fleft_distance = np.min(fleft_ranges)

        
        


    def compute_min_distance(self):
        # Determine which side has the closest object
        if self.fright_distance < self.right_distance and self.fright_distance < self.bright_distance and self.fright_distance < self.bleft_distance and self.fright_distance < self.left_distance and self.fright_distance < self.fleft_distance:
            self.closest_distance = self.fright_distance
            self.closest_side = 'fright'

        elif self.right_distance < self.bright_distance and self.right_distance < self.bleft_distance and self.right_distance < self.left_distance and self.right_distance < self.fleft_distance:
            self.closest_distance = self.right_distance
            self.closest_side = 'right'

        elif self.bright_distance < self.bleft_distance and self.bright_distance < self.left_distance and self.bright_distance < self.fleft_distance:
            self.closest_distance = self.bright_distance
            self.closest_side = 'bright'

        elif self.bleft_distance < self.left_distance and self.bleft_distance <  self.fleft_distance:
            self.closest_distance = self.bleft_distance
            self.closest_side = 'bleft'

        elif self.left_distance < self.fleft_distance:
            self.closest_distance = self.left_distance
            self.closest_side = 'left'
        else:
            self.closest_distance = self.fleft_distance
            self.closest_side = 'fleft'



    def run(self):
        msg = Twist()
        while not rospy.is_shutdown():
            self.compute_min_distance()
            if self.closest_distance < self.safe_distance:

                if self.closest_side == 'fright':
                    msg.angular.z = self.rotation_speed

                elif self.closest_side == 'right':
                    while self.closest_distance < self.max_distance:
                        msg.linear.x = self.forward_speed    
                        msg.angular.z = self.rotation_speed
                    msg.linear.x = self.forward_speed
                    msg.angular.z = -(self.rotation_speed)

                elif self.closest_side == 'bright':
                    msg.angular.z = -(self.rotation_speed)

                elif self.closest_side == 'bleft':
                    msg.angular.z = -(self.rotation_speed)

                elif self.closest_side == 'left':
                    msg.angular.z = -(self.rotation_speed)

                elif self.closest_side == 'fleft':
                    msg.angular.z = self.rotation_speed
            else:
                msg.linear.x = self.forward_speed
                msg.linear.y = 0
                msg.angular.z = 0

            self.cmd_vel_pub.publish(msg)
            self.rate.sleep()

if __name__ == "__main__":
    node = WallFollower()
    node.run()
