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
        self.forward_speed = rospy.get_param("~forward_speed", 0.2)
        self.backward_speed = rospy.get_param("~backward_speed", -0.1)
        self.lateral_speed = rospy.get_param("~lateral_speed", 0.1)
        self.rotation_speed = rospy.get_param("~rotation_speed", 1.0)

        self.fright_distance = 999 
        self.bright_distance = 999
        self.bleft_distance = 999
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

        # Divide the ranges into left and right
        fright_ranges = ranges[len(ranges)//4:len(ranges)//2]
        bright_ranges = ranges[0:len(ranges)//4]
        bleft_ranges = ranges[(len(ranges)//4)*3:]
        fleft_ranges = ranges[len(ranges)//2:(len(ranges)//4)*3]

        self.fright_distance = np.min(fright_ranges)
        self.bright_distance = np.min(bright_ranges)
        self.bleft_distance = np.min(bleft_ranges)
        self.fleft_distance = np.min(fleft_ranges)

        
        


    def compute_min_distance(self):
        # Determine which side has the closest object
        if self.fright_distance < self.bright_distance and self.fright_distance < self.bleft_distance and self.fright_distance < self.fleft_distance:
            self.closest_distance = self.fright_distance
            self.closest_side = 'fright'

        elif self.bright_distance < self.bleft_distance and self.bright_distance < self.fleft_distance:
            self.closest_distance = self.bright_distance
            self.closest_side = 'bright'

        elif self.bleft_distance <  self.fleft_distance:
            self.closest_distance = self.bleft_distance
            self.closest_side = 'bleft'
        else:
            self.closest_distance = self.fleft_distance
            self.closest_side = 'fleft'



    def run(self):
        msg = Twist()
        while not rospy.is_shutdown():
            self.compute_min_distance()
            if self.closest_distance < self.safe_distance:

                if self.closest_side == 'fright':
                    msg.linear.x = self.backward_speed
                    msg.linear.y = self.lateral_speed

                elif self.closest_side == 'bright':
                    msg.linear.x = self.forward_speed
                    msg.linear.y = self.lateral_speed

                elif self.closest_side == 'bleft':
                    msg.linear.x = self.forward_speed
                    msg.linear.y = -(self.lateral_speed)

                elif self.closest_side == 'fleft':
                    msg.linear.x = self.backward_speed
                    msg.linear.y = -(self.lateral_speed)
            else:
                msg.linear.x = self.forward_speed
                msg.linear.y = 0
                msg.angular.z = 0

            self.cmd_vel_pub.publish(msg)
            self.rate.sleep()

if __name__ == "__main__":
    node = WallFollower()
    node.run()
