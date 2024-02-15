#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32  # Import Float32 type
import math

class DistancePublisher:
    def __init__(self):
        rospy.init_node('distance_publisher', anonymous=True)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.distance_pub = rospy.Publisher('/distance_moved', Float32, queue_size=10)
        self.distance_moved = 0.0

    def odom_callback(self, msg):
        x_displacement = msg.pose.pose.position.x
        y_displacement = msg.pose.pose.position.y

        distance_moved = math.sqrt(x_displacement**2 + y_displacement**2)
        self.distance_moved += distance_moved

    def publish_distance(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            self.distance_pub.publish(self.distance_moved)
            rate.sleep()

if __name__ == '__main__':
    try:
        distance_publisher = DistancePublisher()
        distance_publisher.publish_distance()
    except rospy.ROSInterruptException:
        pass

