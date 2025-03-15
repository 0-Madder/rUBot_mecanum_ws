#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

pub = None
d = 0
vx = 0
wz = 0
vf = 0

isScanRangesLengthCorrectionFactorCalculated = False
scanRangesLengthCorrectionFactor = 2


def clbk_laser(msg):
    global isScanRangesLengthCorrectionFactorCalculated
    global scanRangesLengthCorrectionFactor
    
    if not isScanRangesLengthCorrectionFactorCalculated:
        scanRangesLengthCorrectionFactor = len(msg.ranges) / 360
        isScanRangesLengthCorrectionFactorCalculated = True

    front_min= int(0 * scanRangesLengthCorrectionFactor)
    front_max = int(30 * scanRangesLengthCorrectionFactor)
    fright_min = int(30 * scanRangesLengthCorrectionFactor)
    fright_max = int(60 * scanRangesLengthCorrectionFactor)
    right_min = int(60 * scanRangesLengthCorrectionFactor)
    right_max = int(120 * scanRangesLengthCorrectionFactor)
    bright_min = int(120 * scanRangesLengthCorrectionFactor)
    bright_max = int(170 * scanRangesLengthCorrectionFactor)
    back_min= int(170 * scanRangesLengthCorrectionFactor)
    back_max = int(190 * scanRangesLengthCorrectionFactor)

    regions = {
        'bright':  min(min(msg.ranges[bright_min:bright_max]), 3),
        'right':  min(min(msg.ranges[right_min:right_max]), 3),
        'fright': min(min(msg.ranges[fright_min:fright_max]), 3),
        'front':  min(min(msg.ranges[front_min:front_max]), 3),
        'back' : min(min(msg.ranges[back_min:back_max]), 3)
    }

    take_action(regions)


def take_action(regions):
    msg = Twist()
    linear_x = vx
    linear_y = 0
    angular_z = 0  # Eliminamos la rotación

    state_description = ''

    # Caso 1: La pared más cercana está a la derecha -> Seguir recto
    if regions['right'] < regions['front'] and regions['right'] < regions['fright'] and regions['right'] < regions['bright']:
        state_description = 'case 1 - Wall on the right -> Moving forward'
        linear_x = vx
        linear_y = 0

    # Caso 2: La pared más cercana está enfrente -> Moverse hacia la izquierda
    elif regions['front'] < regions['right'] and regions['front'] < regions['fright'] and regions['front'] < regions['bright']:
        state_description = 'case 2 - Wall in front -> Moving left'
        linear_x = 0
        linear_y = vx

    # Caso 3: La pared más cercana está atrás a la derecha -> Moverse en diagonal hacia afuera
    elif regions['bright'] < regions['front'] and regions['bright'] < regions['fright'] and regions['bright'] < regions['right']:
        state_description = 'case 3 - Wall behind right -> Moving diagonally outwards'
        linear_x = vx/3
        linear_y = vx/6

    # Caso 4: La pared más cercana está adelante a la derecha -> Moverse en diagonal hacia afuera
    elif regions['fright'] < regions['front'] and regions['fright'] < regions['right'] and regions['fright'] < regions['bright']:
        state_description = 'case 4 - Wall in front-right -> Moving diagonally outwards'
        linear_x = vx/6
        linear_y = -vx/3

    rospy.loginfo(state_description)
    msg.linear.x = linear_x
    msg.linear.y = linear_y
    msg.angular.z = angular_z
    pub.publish(msg)
    rate.sleep()


def shutdown():
    msg = Twist()
    msg.linear.x = 0
    msg.linear.y = 0
    msg.angular.z = 0
    pub.publish(msg)
    rospy.loginfo("Stop rUBot")


def main():
    global pub
    global sub
    global rate
    global d
    global vx
    global wz
    global vf

    rospy.init_node('wall_follower_holonomic')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    rospy.on_shutdown(shutdown)
    rate = rospy.Rate(25)

    d = rospy.get_param("~distance_laser")
    vx = rospy.get_param("~forward_speed")
    wz = rospy.get_param("~rotation_speed")  # No se usa en holonómico
    vf = rospy.get_param("~speed_factor")


if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        shutdown()
