#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Wrench
from tf.transformations import euler_from_quaternion


class PIDController:
    def __init__(self, target_position, target_attitude):
        self.target_position = target_position
        self.target_attitude = target_attitude

        # PID parameters
        self.prev_height_error = 0.0
        self.height_error_integral = 0.0

        self.prev_x_error = 0.0
        self.x_error_integral = 0.0

        self.prev_y_error = 0.0
        self.y_error_integral = 0.0

        self.prev_heading_error = 0.0
        self.heading_error_integral = 0.0

        self.dt = 0.02

    def thrust(self, current_height):
        Kp, Ki, Kd = 5.0, 0.0, 0.0

        height_error = self.target_position.z - current_height
        height_error_derivative = (height_error - self.prev_height_error) / self.dt
        self.height_error_integral += height_error * self.dt

        thrust_command = (Kp * height_error +
                          Kd * height_error_derivative +
                          Ki * self.height_error_integral)
        
        self.prev_height_error = height_error

        return thrust_command
    
    def surge(self, current_x):
        Kp, Ki, Kd = 5.0, 0.0, 0.0

        x_error = self.target_position.x - current_x
        x_error_derivative = (x_error - self.prev_x_error) / self.dt
        self.x_error_integral += x_error * self.dt

        surge_command = (Kp * x_error +
                         Kd * x_error_derivative +
                         Ki * self.x_error_integral)
        
        self.prev_x_error = x_error

        return surge_command
    
    def sway(self, current_y):
        Kp, Ki, Kd = 5.0, 0.0, 0.0

        y_error = self.target_position.y - current_y
        y_error_derivative = (y_error - self.prev_y_error) / self.dt
        self.y_error_integral += y_error * self.dt

        sway_command = (Kp * y_error +
                        Kd * y_error_derivative +
                        Ki * self.y_error_integral)
        
        self.prev_y_error = y_error
        
        return sway_command
    
    def yaw(self, current_heading):
        Kp, Ki, Kd = 1.0, 0.0, 0.0

        heading_error = self.normalize_angle(self.target_attitude[2] - current_heading)
        heading_error_derivative = (heading_error - self.prev_heading_error) / self.dt
        self.heading_error_integral += heading_error * self.dt

        yaw_command = (Kp * heading_error +
                       Kd * heading_error_derivative +
                       Ki * self.heading_error_integral)
        
        self.prev_heading_error = heading_error

        return yaw_command
    
    def normalize_angle(self, angle):
        while angle < -np.pi:
            angle += 2 * np.pi
        while angle > np.pi:
            angle -= 2 * np.pi
        return angle


class Drone:
    def __init__(self):
        rospy.init_node('quadrotor_pid', anonymous=True)

        self.odom_sub = rospy.Subscriber('/ground_truth/state', Odometry, self.odom_callback)
        self.cmd_pub = rospy.Publisher('/quadrotor/cmd_force', Wrench, queue_size=10)

        self.current_position = Point()
        self.current_attitude = np.zeros(3)

        self.target_point = Point()
        self.target_attitude = np.zeros(3)

        self.target_point.x = 2.0
        self.target_point.y = 3.0
        self.target_point.z = 5.0
        self.target_attitude[2] = np.pi / 2

        self.controller = PIDController(self.target_point, self.target_attitude)
        self.command = Wrench()

        self.rate = rospy.Rate(50)

    def odom_callback(self, data):
        self.current_position = data.pose.pose.position

        quat = [data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z,
                data.pose.pose.orientation.w]
        
        self.current_attitude = euler_from_quaternion(quat)

        rospy.loginfo(f"Current position(x,y,z): {self.current_position.x:.2f}, {self.current_position.y:.2f}, {self.current_position.z:.2f}")
        rospy.loginfo(f"Current attitude (roll,pitch,yaw): {self.current_attitude[0]:.2f}, {self.current_attitude[1]:.2f}, {self.current_attitude[2]:.2f}")

        self.fly()

    def fly(self):
        self.command.force.z = self.controller.thrust(self.current_position.z)
        self.command.force.x = self.controller.surge(self.current_position.x)
        self.command.force.y = self.controller.sway(self.current_position.y)
        self.command.torque.z = self.controller.yaw(self.current_attitude[2])

        self.cmd_pub.publish(self.command)


if __name__ == "__main__":
    try:
        drone = Drone()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass