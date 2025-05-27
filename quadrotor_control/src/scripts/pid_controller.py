#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Wrench, Vector3
from tf.transformations import euler_from_quaternion


class PIDController:
    def __init__(self, target_position, target_heading):
        self.target_position = target_position
        self.target_heading = target_heading

        # PID parameters
        self.prev_height_error = 0.0
        self.height_error_integral = 0.0

        self.prev_x_error = 0.0
        self.x_error_integral = 0.0

        self.prev_y_error = 0.0
        self.y_error_integral = 0.0

        self.prev_heading_error = 0.0
        self.heading_error_integral = 0.0

        self.prev_roll_error = 0.0
        self.roll_error_integral = 0.0

        self.prev_pitch_error = 0.0
        self.pitch_error_integral = 0.0

        self.dt = 0.01 # 10ms / iteration

    def roll(self, current_roll_velocity):
        Kp, Ki, Kd = 1.0, 0.0, 0.0

        roll_error = -current_roll_velocity
        roll_error_derivative = (roll_error - self.prev_roll_error) / self.dt
        self.roll_error_integral += roll_error * self.dt

        roll_command = (Kp * roll_error +
                        Kd * roll_error_derivative +
                        Ki * self.roll_error_integral)
        
        self.prev_roll_error = roll_error

        return roll_command
    
    def pitch(self, current_pitch_velocity):
        Kp, Ki, Kd = 1.0, 0.0, 0.0

        pitch_error = -current_pitch_velocity
        pitch_error_derivative = (pitch_error - self.prev_pitch_error) / self.dt
        self.pitch_error_integral += pitch_error * self.dt

        pitch_command = (Kp * pitch_error +
                         Kd * pitch_error_derivative +
                         Ki * self.pitch_error_integral)
        
        self.prev_pitch_error = pitch_error

        return pitch_command

    def thrust(self, current_height):
        Kp, Ki, Kd = 9.0, 8.5, 11.5

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

        heading_error = self.target_heading - current_heading
        heading_error = self.normalize_angle(heading_error)

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
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)

        self.cmd_pub = rospy.Publisher('/quadrotor/cmd_force', Wrench, queue_size=10)
        self.goal_pub = rospy.Publisher('/goal_pose', Point, queue_size=10)

        self.current_position = Point()
        self.current_attitude = np.zeros(3)
        self.current_angular_velocity = Vector3()

        self.target_point = Point(1.0, 1.0, 5.0)
        self.target_heading = np.pi / 2

        self.controller = PIDController(self.target_point, self.target_heading)
        self.command = Wrench()

        self.control_timer = rospy.Timer(rospy.Duration(1/100), self.fly) # 100Hz update rate

    def imu_callback(self, data):
        self.current_angular_velocity = data.angular_velocity
        
        #rospy.loginfo(f"Current pitch velocity: {self.current_angular_velocity.x:2f}")
        #rospy.loginfo(f"Current roll velocity: {self.current_angular_velocity.y:2f}")

    def odom_callback(self, data):
        self.current_position = data.pose.pose.position

        quat = [data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z,
                data.pose.pose.orientation.w]
        
        self.current_attitude = euler_from_quaternion(quat)

        if abs(self.current_position.z - self.target_point.z) <= 0.1:
            rospy.loginfo(f"Current height: {self.current_position.z:.2f}")
        else:
            rospy.logwarn(f"Current height: {self.current_position.z:.2f}")
        #rospy.loginfo(f"Current heading: {self.current_attitude[2]:.2f}")

    def fly(self, event):
        self.command.force.z = self.controller.thrust(self.current_position.z)

        self.command.force.x = self.controller.surge(self.current_position.x) * 0
        self.command.force.y = self.controller.sway(self.current_position.y) * 0

        self.command.torque.z = self.controller.yaw(self.current_attitude[2]) * 0

        # Stabilize pitch and roll using rate controllers
        self.command.torque.x = self.controller.pitch(self.current_angular_velocity.x)
        self.command.torque.y = self.controller.roll(self.current_angular_velocity.y)

        self.cmd_pub.publish(self.command)
        self.goal_pub.publish(self.target_point)


if __name__ == "__main__":
    try:
        drone = Drone()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass