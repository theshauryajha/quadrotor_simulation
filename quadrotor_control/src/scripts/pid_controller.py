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
        self.command.torque.x = 0 # No pitch
        self.command.torque.y = 0 # No roll

        self.target_point = Point()
        self.target_attitude = np.zeros(3)

        self.target_point.x = 2.0
        self.target_point.y = 3.0
        self.target_point.z = 5.0
        self.target_attitude[2] = 1.57

        # PID gain constants
        self.surge_Kp, self.surge_Kd, self.surge_Ki = 0.0, 0.0, 0.0
        self.sway_Kp, self.sway_Kd, self.sway_Ki = 0.0, 0.0, 0.0
        self.thrust_Kp, self.thrust_Kd, self.thrust_Ki = 5.0, 0.0, 0.0
        self.yaw_Kp, self.yaw_Kd, self.yaw_Ki = 0.0, 0.0, 0.0

        # PID parameters
        self.prev_x_error = 0.0
        self.x_error_integral = 0.0

        self.prev_y_error = 0.0
        self.y_error_integral = 0.0

        self.prev_height_error = 0.0
        self.height_error_integral = 0.0

        self.prev_heading_error = 0.0
        self.heading_error_integral = 0.0

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

    def calculate_pid_control(self, error, derivative, integral, Kp, Kd, Ki):
        control = (Kp * error + Kd * derivative + Ki * integral)
        return control

    def hover(self):
        x_error = self.target_point.x - self.position.x
        y_error = self.target_point.y - self.position.y
        height_error = self.target_point.z - self.position.z
        heading_error = self.target_attitude[2] - self.attitude[2]

        x_error_derivative = x_error - self.prev_x_error
        self.x_error_integral += x_error

        y_error_derivative = y_error - self.prev_y_error
        self.y_error_integral += y_error

        height_error_derivative = height_error - self.prev_height_error
        self.height_error_integral += height_error

        heading_error_derivative = heading_error - self.prev_heading_error
        self.heading_error_integral += heading_error

        thrust = self.calculate_pid_control(height_error, height_error_derivative, self.height_error_integral,
                                            self.thrust_Kp, self.thrust_Kd, self.thrust_Ki)
        
        surge, sway, yaw = 0.0, 0.0, 0.0
        
        self.prev_x_error = x_error
        self.prev_y_error = y_error
        self.prev_height_error = height_error
        self.prev_heading_error = heading_error

        self.command.force.x = surge
        self.command.force.y = sway
        self.command.force.z = thrust
        self.command.torque.z = yaw

        self.cmd_pub.publish(self.command)


if __name__ == "__main__":
    try:
        drone = DronePID()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass