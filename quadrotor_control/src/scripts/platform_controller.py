#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Wrench


class PlatformController:
    def __init__(self):
        rospy.init_node('platform_controller', anonymous=True)

        self.odom_sub = rospy.Subscriber('/platform/odom', Odometry, self.odom_callback)
        self.cmd_pub = rospy.Publisher('/platform/cmd_force', Wrench, queue_size=10)

        self.current_position = Point()
        self.command = Wrench()

    def odom_callback(self, data):
        self.current_position = data.pose.pose.position

    def move(self):
        self.command.force.x = 2.0
        self.command.force.y = 2.0

        self.cmd_pub.publish(self.command)


if __name__ == "__main__":
    try:
        platform_controller = PlatformController()
        platform_controller.move()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass