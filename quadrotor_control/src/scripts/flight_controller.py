#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This module implements sets up a ROS Node to implement a low-level PID flight
controller, referring to high-level target setpoints determined by the Planning
node and the Drone's current state data.

This module subscribes to the Drone's Odometry data and the target parameters
published by the Planner, uses this data to compute control commands in each of
the 6 Degrees of Freedom and publishes these as forces and torques acting on
the body of the Drone.

It implements independent PID loops for altitude control (thrust), cruise
motion (surge and sway) and rotation (yaw), as well as rate controllers to
stabilize the Drone in the air (roll and pitch).
"""

import rospy
import numpy as np
from std_msgs.msg import String, Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist, Wrench
from tf.transformations import euler_from_quaternion


class Controller:
    def __init__(self):
        """
        Initilaizes the Control node to navigate to setpoints for the current
        mission state.

        This constructor sets up the necessary ROS infrastructure including
        subscribers to the Drone's state data and the target parameters for the
        current mission phase as well as a publisher to command the Drone's
        motion in the 3D environment.

        It implements a PID Controller with methods to compute control outputs
        for each Degree of Freedo with independent gain coefficients. It also
        sets realistic acceleration constraints on the Drone.

        Subscribers:
            * /quadrotor/ground_truth (Odometry): Drone state
            * /target/position (Point): Position setpoint for navigation
            * /target/heading (Float64): Heading setpoint for rotation
            * /mission_state (String): Current mission phase

        Publishers:
            * /quadrotor/cmd_force (Wrench): Force and Torque commands

        This method also initializes the main control loop at 100Hz frequency
        using a ROS Timer.
        """

        rospy.init_node('control_node', anonymous=True)

        # Subscribes to Drone and Planning data
        self.drone_sub = rospy.Subscriber('/quadrotor/ground_truth', Odometry, self.drone_callback)
        self.position_sub = rospy.Subscriber('/target/position', Point, self.position_callback)
        self.heading_sub = rospy.Subscriber('/target/heading', Float64, self.heading_callback)
        self.state_sub = rospy.Subscriber('/mission_state', String, self.state_callback)

        # Publishes Control Signals
        self.command_pub = rospy.Publisher('/quadrotor/cmd_force', Wrench, queue_size=10)

        # Setpoint data
        self.target_position = Point()
        self.target_heading = 0.0
  
        # Current mission state
        self.mission_state = String()

        # Derivative and Integral Control parameters
        self.prev_height_error = 0.0
        self.height_error_integral = 0.0

        self.prev_x_error = 0.0
        self.x_error_integral = 0.0

        self.prev_y_error = 0.0
        self.y_error_integral = 0.0

        self.prev_roll_error = 0.0
        self.roll_error_integral = 0.0

        self.prev_pitch_error = 0.0
        self.pitch_error_integral = 0.0

        self.prev_heading_error = 0.0
        self.heading_error_integral = 0.0

        # Previous control outputs for limiting acceleration
        self.prev_thrust_command = 0.0
        self.prev_surge_command = 0.0
        self.prev_sway_command = 0.0
        self.prev_yaw_command = 0.0

        # Acceleration limits
        self.max_cruise_accel = 10.0
        self.max_descent_accel = 2.0
        self.max_yaw_accel = np.pi / 18

        # Control update rate
        self.dt = 0.01 # 10ms / iteration

        # Drone data
        self.drone_position = Point()
        self.drone_attitude = np.zeros(3)
        self.drone_heading = self.drone_attitude[2]
        self.drone_velocity = Twist()

        # Time to publish Control Signals
        self.timer = rospy.Timer(rospy.Duration(1/100), self.fly) # !00Hz update rate

    def drone_callback(self, data: Odometry):
        """
        Updates the Drone's linear and angular velocities as well as the
        position and orientation obtained from ground truth and logs Pose data.

        Args:
            data(Odometry): Incoming odometry data
        """

        self.drone_position = data.pose.pose.position
        self.drone_velocity = data.twist.twist

        quat = [
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w,
        ]

        self.drone_attitude = euler_from_quaternion(quat)
        self.drone_heading = self.drone_attitude[2]

        # rospy.loginfo(f"Current position (x,y): {self.drone_position.x:.2f}, {self.drone_position.y:.2f}")
        # rospy.loginfo(f"Current height: {self.drone_position.z:.2f}")
        # rospy.loginfo(f"Current orientation about x axis (roll): {self.drone_attitude[0]:.2f}")
        # rospy.loginfo(f"Current orientation about y axis (pitch): {self.drone_attitude[1]:.2f}")
        # rospy.loginfo(f"Current heading: {self.drone_heading:.2f}")

    def position_callback(self, data: Point):
        """
        Update the desired position setpoint from the planner.

        Args:
            data (Point): Target x, y, z coordinates.
        """

        self.target_position = data
    
    def heading_callback(self, data: Float64):
        """
        Update the desired heading setpoint from the planner.

        Args:
            data (Float64): Target yaw angle (radians).
        """

        self.target_heading = data.data

    def state_callback(self, data: String):
        """
        Receive current mission state to adjust control behavior.

        Args:
            data (String): Name of current mission phase.
        """

        self.mission_state = data.data

    def limit_acceleration(self, current_command: float, prev_command: float, max_accel: float) -> float:
        """
        Clamp commanded changes to respect acceleration limits.

        Args:
            current_command(float): Computed control output for the current
                                    iteration
            prev_command(float): Previously computed control output
            max_accel(float): Limit on acceleration

        Returns:
            float: Adjusted command limited by acceleration constraints
        """

        max_change = max_accel * self.dt
        delta = current_command - prev_command
        if abs(delta) > max_change:
            delta = max_change if delta > 0 else -max_change
        return prev_command + delta

    def thrust(self) -> float:
        """
        Compute PID Control command for vertical thrust, and limit acceleration
        during descent.
        
        Returns:
            float: Computed PID Control command for vertical thrust
        """

        Kp, Ki, Kd = 5.0, 1.5, 3.5

        height_error = self.target_position.z - self.drone_position.z
        height_error_derivative = (height_error - self.prev_height_error) / self.dt
        self.height_error_integral += height_error * self.dt

        thrust_command = (Kp * height_error +
                          Kd * height_error_derivative +
                          Ki * self.height_error_integral)
        
        if self.mission_state == "DESCEND":
            thrust_command = self.limit_acceleration(thrust_command, self.prev_thrust_command, self.max_descent_accel)
        
        self.prev_height_error = height_error
        self.prev_thrust_command = thrust_command

        return thrust_command
    
    def surge(self) -> float:
        """
        Compute PID Control command for forward (body x-direction) motion.
        
        Returns:
            float: Computed PID Control command for forward motion
        """

        Kp, Ki, Kd = 1.3, 0.001, 4.3

        x_error = self.target_position.x - self.drone_position.x
        x_error_derivative = (x_error - self.prev_x_error) / self.dt
        self.x_error_integral += x_error * self.dt

        surge_command = (Kp * x_error +
                         Kd * x_error_derivative +
                         Ki * self.x_error_integral)
        
        surge_command = self.limit_acceleration(surge_command, self.prev_surge_command, self.max_cruise_accel)
        
        self.prev_x_error = x_error
        self.prev_surge_command = surge_command

        return surge_command
    
    def sway(self) -> float:
        """
        Compute PID Control command for lateral (body y-direction) motion.
        
        Returns:
            float: Computed PID Control command for lateral motion
        """

        Kp, Ki, Kd = 1.3, 0.001, 4.3

        y_error = self.target_position.y - self.drone_position.y
        y_error_derivative = (y_error - self.prev_y_error) / self.dt
        self.y_error_integral += y_error * self.dt

        sway_command = (Kp * y_error +
                        Kd * y_error_derivative +
                        Ki * self.y_error_integral)
        
        sway_command = self.limit_acceleration(sway_command, self.prev_sway_command, self.max_cruise_accel)
        
        self.prev_y_error = y_error
        self.prev_sway_command = sway_command
        
        return sway_command
    
    def roll_rate(self) -> float:
        """
        Rate controller to stabilize roll by minimizing angular velocity about
        body x-axis.
        
        Returns:
            float: Computed PID Control command for roll motion
        """

        Kp, Ki, Kd = 1.0, 0.0, 0.0

        roll_error = -self.drone_velocity.angular.x
        roll_error_derivative = (roll_error - self.prev_roll_error) / self.dt
        self.roll_error_integral += roll_error * self.dt

        roll_command = (Kp * roll_error +
                        Kd * roll_error_derivative +
                        Ki * self.roll_error_integral)
        
        self.prev_roll_error = roll_error

        return roll_command
    
    def pitch_rate(self) -> float:
        """
        Rate controller to stabilize pitch by minimizing angular velocity about
        body y-axis.
        
        Returns:
            float: Computed PID Control command for pitch motion
        """
        
        Kp, Ki, Kd = 1.0, 0.0, 0.0

        pitch_error = -self.drone_velocity.angular.y
        pitch_error_derivative = (pitch_error - self.prev_pitch_error) / self.dt
        self.pitch_error_integral += pitch_error * self.dt

        pitch_command = (Kp * pitch_error +
                         Kd * pitch_error_derivative +
                         Ki * self.pitch_error_integral)
        
        self.prev_pitch_error = pitch_error

        return pitch_command
    
    def yaw(self) -> float:
        """
        Compute PID Control command for rotational (about body z-axis) motion.
        
        Returns:
            float: Computed PID Control command for rotational (yaw) motion
        """

        Kp, Ki, Kd = 0.3, 0.001, 0.7

        heading_error = self.target_heading - self.drone_heading
        heading_error = self.normalize_angle(heading_error)

        heading_error_derivative = (heading_error - self.prev_heading_error) / self.dt
        self.heading_error_integral += heading_error * self.dt

        yaw_command = (Kp * heading_error +
                       Kd * heading_error_derivative +
                       Ki * self.heading_error_integral)
        
        yaw_command = self.limit_acceleration(yaw_command, self.prev_yaw_command, self.max_yaw_accel)
        
        self.prev_heading_error = heading_error
        self.prev_yaw_command = yaw_command

        return yaw_command
    
    def normalize_angle(self, angle: float) -> float:
        """
        Normalize an angle to the range [-pi, pi].

        Args:
            angle(float): Angle to be normalized
        
        Returns:
            float: Normalized angle
        """

        while angle < -np.pi:
            angle += 2 * np.pi
        while angle > np.pi:
            angle -= 2 * np.pi
        return angle
    
    def fly(self, event):
        """
        Main planning loop, executes at 100Hz frequency using a ROS Timer.

        Uses the PID Control methods to compute the control signals in each
        Degree of Freedom and publishes them as forces and torques acting on
        the Drone's body.
        """
        
        command = Wrench()

        if self.mission_state != "LANDED":
            command.force.z = self.thrust()
            command.force.x = self.surge()
            command.force.y = self.sway()
            command.torque.z = self.yaw()
            command.torque.x = self.roll_rate()
            command.torque.y = self.pitch_rate()

        self.command_pub.publish(command)


if __name__ == "__main__":
    try:
        controller = Controller()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass