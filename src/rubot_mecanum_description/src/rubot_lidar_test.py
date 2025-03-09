#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan

def getIndexAtAngle(n_beams, atNangle):
    return (n_beams // 360 * atNangle) 

def callback(msg):
    print ("Number of scan points: "+ str(len(msg.ranges)))
    # values at 0 degrees
    print ("Distance at 0deg: " + str(msg.ranges[getIndexAtAngle(1147, 0)]))
    # values at 30 degrees
    print ("Distance at 0deg: " + str(msg.ranges[getIndexAtAngle(1147, 30)]))
    # values at 90 degrees
    print ("Distance at 90deg: " + str(msg.ranges[getIndexAtAngle(1147, 90)]))
    # values at 180 degrees
    print ("Distance at 180deg: " + str(msg.ranges[getIndexAtAngle(1147, 180)]))
    # values at 270 degrees
    print ("Distance at 270deg: " + str(msg.ranges[getIndexAtAngle(1147, 270)]))
    # values at 360 degrees
    print ("Distance at 360deg: " + str(msg.ranges[getIndexAtAngle(1147, 360)]))

rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()