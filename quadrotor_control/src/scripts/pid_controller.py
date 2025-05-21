#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Wrench
from tf.transformations import euler_from_quaternion


class DronePID:
    def __init__(self):
        rospy.init_node('quadrotor_pid', anonymous=True)

        self.odom_sub = rospy.Subscriber('/ground_truth/state', Odometry, self.odom_callback)
        self.cmd_pub = rospy.Publisher('/quadrotor/cmd_force', Wrench, queue_size=10)

        self.position = Point()
        self.attitude = np.zeros(3)

        self.command = Wrench()

        self.target_height = 5.0

        # PID Control params
        self.thrust_Kp, self.thrust_Kd, self.thrust_Ki = 5.0, 0.0, 0.0
        self.prev_height_error = 0.0
        self.height_error_integral = 0.0

        self.rate = rospy.Rate(50)

    def odom_callback(self, data):
        self.position = data.pose.pose.position

        quat = [data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z,
                data.pose.pose.orientation.w]
        
        self.attitude = euler_from_quaternion(quat)

        rospy.loginfo(f"Current position(x,y,z): {self.position.x:.2f}, {self.position.y:.2f}, {self.position.z:.2f}")
        rospy.loginfo(f"Current attitude (roll,pitch,yaw): {self.attitude[0]:.2f}, {self.attitude[1]:.2f}, {self.attitude[2]:.2f}")

        self.hover()

    def hover(self):
        height_error = self.target_height - self.position.z

        height_error_derivative = height_error - self.prev_height_error
        self.height_error_integral += height_error

        thrust = (self.thrust_Kp * height_error +
                  self.thrust_Kd * height_error_derivative +
                  self.thrust_Ki * self.height_error_integral)
        
        self.prev_height_error = height_error

        self.command.force.z = thrust
        self.command.force.x, self.command.force.y = 0.0, 0.0
        self.command.torque.x, self.command.torque.y, self.command.torque.z = 0.0, 0.0, 0.0

        self.cmd_pub.publish(self.command)


if __name__ == "__main__":
    try:
        drone = DronePID()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass