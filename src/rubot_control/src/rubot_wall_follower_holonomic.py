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
        self.lateral_speed = rospy.get_param("~lateral_speed", 0.1)
        self.rotation_speed = rospy.get_param("~rotation_speed", 1.0)

        self.fright_distance = 999 
        self.right_distance = 999 
        self.bright_distance = 999
        self.bleft_distance = 999
        self.left_distance = 999
        self.fleft_distance = 999
        self.front_distance = 999
        self.ranges = []

        # Velocity publisher
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback, queue_size=10)

        self.closest_distance = float("inf")
        self.closest_side = None  # 'left' or 'right'
        self.rate = rospy.Rate(10)  # 10 Hz

    def laser_callback(self, scan):
        self.ranges = np.array(scan.ranges)
        
        


    def compute_min_distance(self):
        # Determine which side has the closest object
        if self.front_distance < self.fright_distance and self.front_distance < self.right_distance and self.front_distance < self.bright_distance and self.front_distance < self.bleft_distance and self.front_distance < self.left_distance and self.front_distance < self.fleft_distance:
            self.closest_distance = self.front_distance
            self.closest_side = 'front'

        elif self.fright_distance < self.right_distance and self.fright_distance < self.bright_distance and self.fright_distance < self.bleft_distance and self.fright_distance < self.left_distance and self.fright_distance < self.fleft_distance:
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

    def calcul_ranges(self):
        self.ranges[np.isinf(self.ranges)] = 999  # Replace inf values with a large number

        sector = len(self.ranges)//8

        # Divide the ranges into left and right
        fright_ranges = self.ranges[int(2.5 * sector):int(3.5 * sector)]
        right_ranges = self.ranges[sector:int(2.5 * sector)]
        bright_ranges = self.ranges[0:sector]
        bleft_ranges = self.ranges[(7 * sector):]
        left_ranges = self.ranges[int(5.5 * sector):int(7 * sector)]
        fleft_ranges = self.ranges[int(4.5 * sector): int(5.5 * sector)]
        front_ranges = self.ranges[int(3.5 * sector): int(4.5 * sector)]

        self.fright_distance = np.min(fright_ranges)
        self.right_distance = np.min(right_ranges)
        self.bright_distance = np.min(bright_ranges)
        self.bleft_distance = np.min(bleft_ranges)
        self.left_distance = np.min(left_ranges)
        self.fleft_distance = np.min(fleft_ranges)
        self.front_distance = np.min(front_ranges)


    def run(self):
        msg = Twist()
        while not rospy.is_shutdown():
            self.calcul_ranges()
            self.compute_min_distance()
            if self.closest_distance < self.safe_distance:
                if self.closest_side == 'right':
                    msg.linear.x = self.forward_speed
                    msg.linear.y = 0
                elif self.closest_side == 'front':
                    msg.linear.x = 0
                    msg.linear.y = self.lateral_speed
                elif self.closest_side == 'fright':
                    msg.linear.x = self.forward_speed
                    msg.linear.y = self.lateral_speed
                elif self.closest_side == 'bright':
                    msg.linear.x = self.backward_speed
                    msg.linear.y = -(self.lateral_speed)
                elif self.closest_side == 'bleft':
                    pass
                    #msg.linear.x = self.backward_speed
                elif self.closest_side == 'left':
                    pass
                    #msg.linear.x = self.backward_speed
                elif self.closest_side == 'fleft':
                    pass
                    #msg.linear.x = self.backward_speed
            else:
                msg.linear.x = self.forward_speed
                msg.linear.y = 0
                msg.angular.z = 0

            self.cmd_vel_pub.publish(msg)
            self.rate.sleep()

if __name__ == "__main__":
    node = WallFollower()
    node.run()
