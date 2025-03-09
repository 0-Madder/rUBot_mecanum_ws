#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class rUBot:
    def __init__(self):
        rospy.init_node("rubot_nav", anonymous=False)

        # Configurable parameters
        self._distanceLaser = rospy.get_param("~distance_laser", 0.5)  # Default value
        self._speedFactor = rospy.get_param("~speed_factor", 1.0)
        self._forwardSpeed = rospy.get_param("~forward_speed", 0.2)
        self._backwardSpeed = rospy.get_param("~backward_speed", -0.1)
        self._lateralSpeed = rospy.get_param("~lateral_speed", 0.2)
        self._rotationSpeed = rospy.get_param("~rotation_speed", 1.0)

        # Initialize publishers and subscribers
        self._msg = Twist()
        self._cmdVel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.callbackLaser)
        rospy.on_shutdown(self.shutdown)

        # Control loop rate
        self._r = rospy.Rate(5)

        # Variables for scan correction
        self.__isScanRangesLengthCorrectionFactorCalculated = False
        self.__scanRangesLengthCorrectionFactor = 1

    def start(self):
        while not rospy.is_shutdown():
            self._cmdVel.publish(self._msg)
            self._r.sleep()

    def callbackLaser(self, scan):
        if not self.__isScanRangesLengthCorrectionFactorCalculated:
            self.__scanRangesLengthCorrectionFactor = int(len(scan.ranges) / 360)
            self.__isScanRangesLengthCorrectionFactorCalculated = True
            rospy.loginfo("Scan Ranges correction factor: %5.2f, number of points: %d",
                          self.__scanRangesLengthCorrectionFactor, len(scan.ranges))

        closestDistance, elementIndex = min(
            (val, idx) for (idx, val) in enumerate(scan.ranges)
            if scan.range_min < val < scan.range_max
        )
        angleClosestDistance = elementIndex / self.__scanRangesLengthCorrectionFactor
        angleClosestDistance = self.__wrapAngle(angleClosestDistance)

        rospy.loginfo("Closest distance of %5.2f m at %5.1f degrees.", closestDistance, angleClosestDistance)

        if closestDistance < self._distanceLaser:
            if -30 < angleClosestDistance < 30:
                self._msg.linear.x = self._backwardSpeed * self._speedFactor  # Move backward
            elif 30 <= angleClosestDistance < 150:
                self._msg.linear.y = -self._lateralSpeed * self._speedFactor  # Move left
            elif -150 < angleClosestDistance <= -30:
                self._msg.linear.y = self._lateralSpeed * self._speedFactor  # Move right
            else:
                self._msg.linear.x = self._forwardSpeed * self._speedFactor  # Move forward slightly

            self._msg.angular.z = -self.__sign(angleClosestDistance) * self._rotationSpeed * self._speedFactor
            rospy.logwarn("Evading obstacle...")
        else:
            self._msg.linear.x = self._forwardSpeed * self._speedFactor
            self._msg.linear.y = 0
            self._msg.angular.z = 0

    @staticmethod
    def __sign(val):
        return 1 if val >= 0 else -1

    @staticmethod
    def __wrapAngle(angle):
        if 0 <= angle <= 180:
            return angle
        else:
            return angle - 360

    def shutdown(self):
        self._msg.linear.x = 0
        self._msg.linear.y = 0
        self._msg.angular.z = 0
        self._cmdVel.publish(self._msg)
        rospy.loginfo("Stop rUBot")

if __name__ == '__main__':
    try:
        rUBot1 = rUBot()
        rUBot1.start()
    except rospy.ROSInterruptException:
        pass
    finally:
        rUBot1.shutdown()