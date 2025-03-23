#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class SimpleNavigator:
    def __init__(self):
        rospy.init_node("simple_nav")

        # Parameters
        self.safe_distance = rospy.get_param("~safe_distance", 0.5)
        self.forward_speed = rospy.get_param("~forward_speed", 0.2)
        self.backward_speed = rospy.get_param("~backward_speed", -0.1)
        self.rotation_speed = rospy.get_param("~rotation_speed", 1.0)

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
        right_ranges = ranges[0:len(ranges)//2]
        left_ranges = ranges[len(ranges)//2:]

        # Find the closest distance on each side
        left_distance = np.min(left_ranges)
        right_distance = np.min(right_ranges)

        # Determine which side has the closest object
        if left_distance < right_distance:
            self.closest_distance = left_distance
            self.closest_side = 'left'
        else:
            self.closest_distance = right_distance
            self.closest_side = 'right'

    def run(self):
        msg = Twist()
        while not rospy.is_shutdown():
            if self.closest_distance < self.safe_distance:
                msg.linear.x = self.backward_speed
                if self.closest_side == 'left':
                    msg.angular.z = self.rotation_speed  # Rotate right
                else:
                    msg.angular.z = -self.rotation_speed  # Rotate left
            else:
                msg.linear.x = self.forward_speed
                msg.angular.z = 0

            self.cmd_vel_pub.publish(msg)
            self.rate.sleep()

if __name__ == "__main__":
    node = SimpleNavigator()
    node.run()
