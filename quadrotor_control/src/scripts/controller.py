#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3, Wrench
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from gazebo_msgs.msg import ModelState
from tf.transformations import euler_from_quaternion

class QuadrotorController:
    def __init__(self):
        rospy.init_node('quadrotor_controller', anonymous=True)
        
        # PID Gains
        self.kp_pos = rospy.get_param('~kp_pos', 1.0)
        self.ki_pos = rospy.get_param('~ki_pos', 0.1)
        self.kd_pos = rospy.get_param('~kd_pos', 0.5)
        
        self.kp_att = rospy.get_param('~kp_att', 3.0)
        self.ki_att = rospy.get_param('~ki_att', 0.1)
        self.kd_att = rospy.get_param('~kd_att', 1.0)
        
        # Control errors and integrals
        self.pos_error_prev = np.zeros(3)
        self.pos_integral = np.zeros(3)
        
        self.att_error_prev = np.zeros(3)
        self.att_integral = np.zeros(3)
        
        # Current state
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation = np.zeros(4)  # quaternion
        self.orientation_euler = np.zeros(3)  # roll, pitch, yaw
        self.angular_velocity = np.zeros(3)
        
        # Target position and orientation
        self.target_position = np.zeros(3)
        self.target_yaw = 0.0
        
        # Subscribers
        rospy.Subscriber('/ground_truth/state', Odometry, self.odom_callback)
        rospy.Subscriber('/imu', Imu, self.imu_callback)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        # Publishers
        self.force_pub = rospy.Publisher('/quadrotor/cmd_force', Wrench, queue_size=10)
        
        # Control rate
        self.rate = rospy.Rate(50)  # 50 Hz
        
    def odom_callback(self, msg):
        # Extract position and velocity from odometry
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position[2] = msg.pose.pose.position.z
        
        self.velocity[0] = msg.twist.twist.linear.x
        self.velocity[1] = msg.twist.twist.linear.y
        self.velocity[2] = msg.twist.twist.linear.z
        
        # Extract orientation
        self.orientation[0] = msg.pose.pose.orientation.x
        self.orientation[1] = msg.pose.pose.orientation.y
        self.orientation[2] = msg.pose.pose.orientation.z
        self.orientation[3] = msg.pose.pose.orientation.w
        
        # Convert to Euler angles
        self.orientation_euler = euler_from_quaternion(self.orientation)
        
    def imu_callback(self, msg):
        # Extract angular velocity
        self.angular_velocity[0] = msg.angular_velocity.x
        self.angular_velocity[1] = msg.angular_velocity.y
        self.angular_velocity[2] = msg.angular_velocity.z
        
    def cmd_vel_callback(self, msg):
        # Set target position based on current position and velocity command
        # This is a simple way to convert velocity commands to position targets
        dt = 0.5  # Look-ahead time
        self.target_position[0] = self.position[0] + msg.linear.x * dt
        self.target_position[1] = self.position[1] + msg.linear.y * dt
        self.target_position[2] = self.position[2] + msg.linear.z * dt
        
        # Set target yaw
        self.target_yaw = msg.angular.z * dt + self.orientation_euler[2]
        
    def position_controller(self):
        # Position error
        pos_error = self.target_position - self.position
        
        # PID controller for position
        p_term = self.kp_pos * pos_error
        self.pos_integral += pos_error * 0.02  # dt = 1/50 = 0.02
        i_term = self.ki_pos * self.pos_integral
        d_term = self.kd_pos * (pos_error - self.pos_error_prev) / 0.02
        
        # Save error for next iteration
        self.pos_error_prev = pos_error.copy()
        
        # Desired acceleration
        accel_desired = p_term + i_term + d_term
        
        # Simple attitude controller that generates roll and pitch based on desired acceleration
        roll_desired = np.arcsin(np.clip((accel_desired[1] / 9.81), -0.5, 0.5))
        pitch_desired = -np.arcsin(np.clip((accel_desired[0] / 9.81), -0.5, 0.5))
        
        # Set desired attitude
        att_desired = np.array([roll_desired, pitch_desired, self.target_yaw])
        
        return att_desired, accel_desired[2] + 9.81  # Add gravity compensation
        
    def attitude_controller(self, att_desired, thrust):
        # Current attitude
        att_current = self.orientation_euler
        
        # Attitude error (taking care of angle wrapping for yaw)
        att_error = np.zeros(3)
        att_error[0:2] = att_desired[0:2] - att_current[0:2]
        att_error[2] = np.arctan2(np.sin(att_desired[2] - att_current[2]), np.cos(att_desired[2] - att_current[2]))
        
        # PID controller for attitude
        p_term = self.kp_att * att_error
        self.att_integral += att_error * 0.02
        i_term = self.ki_att * self.att_integral
        d_term = self.kd_att * (att_error - self.att_error_prev) / 0.02
        
        # Save error for next iteration
        self.att_error_prev = att_error.copy()
        
        # Desired torque
        torque_desired = p_term + i_term + d_term
        
        return torque_desired, thrust
        
    def run(self):
        while not rospy.is_shutdown():
            # Run position controller to get desired attitude
            att_desired, thrust = self.position_controller()
            
            # Run attitude controller to get desired torque
            torque, thrust = self.attitude_controller(att_desired, thrust)
            
            # Create and publish force message
            force_msg = Wrench()
            force_msg.force.z = thrust
            force_msg.torque.x = torque[0]
            force_msg.torque.y = torque[1]
            force_msg.torque.z = torque[2]
            
            self.force_pub.publish(force_msg)
            
            self.rate.sleep()
        
if __name__ == '__main__':
    try:
        controller = QuadrotorController()
        controller.run()
    except rospy.ROSInterruptException:
        pass