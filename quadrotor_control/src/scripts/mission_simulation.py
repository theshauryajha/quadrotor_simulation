#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from enum import Enum
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Pose, Point, Twist, Wrench
from tf.transformations import euler_from_quaternion

# Colour for Logging
CYAN = "\033[96m"
RESET = "\033[0m"


class MissionState(Enum):
    """
    Enumeration of mission phases for drone operation.

    This state machine define the various stages of the drone's mission,
    from takeoff to landing. Each state corresponds to a distinct behaviour
    mode and setpoint for the control logic.

    States:
        TAKEOFF:    Drone ascends to the operating altitude at its current
                    position.
        CRUISE:     Drone moves laterally towards the position of the landing
                    platform, until the camera detects a fiducial marker.
        TRACK:      Drone tracks the fiducial marker detected by its camera.
        HOVER:      Drone hovers over the landing platform for 2 seconds.
        ROTATE:     Drone rotates to align itself with the fiducial marker.
        DESCEND:    Drone descendes towards the fiducial marker.
        LANDED:     Drone has successfully landed on the platform.
    """

    TAKEOFF = "TAKEOFF"
    CRUISE = "CRUISE"
    TRACK = "TRACK"
    HOVER = "HOVER"
    ROTATE = "ROTATE"
    DESCEND = "DESCEND"
    LANDED = "LANDED"


class Controller:
    """
    A PID Controller class with methods to compute control outputs for each of
    the 6 Degrees of Freedom with independent gain coefficients.

    Attributes:
        target_point (Point): The target position in 3D space
        target_heading (float): The desired yaw angle in radians

        prev_*_error (float): Previous cycle error values for derivative control
        *_error_integral (float): Accumulated error values for integral control

        prev_*_command (float): Previous command outputs to limit acceleration

        max_*_accel (float): Acceleration limits for smoother control

        dt (float): Time step between control uodates (seconds)

    Methods:
        update_target_*(new_target_*):
            Update Drone's target parameters
        limit_acceleration(current_command, prev_command, max_accel):
            Clamp commanded changes to respect acceleration limits.
        thrust(current_altitude, is_descending):
            Compute vertical thrust command
        surge(current_x):
            Compute forward (body x-direction) motion command
        sway(current_y):
            Compute lateral (body y-direction) motion command
        roll_rate(current_roll_velocity):
            Stabilize roll by minimizing angular velocity about body x-axis
        pitch_rate(current_pitch_velocity):
            Stabilize pitch by minimizing angular velocity about body y-axis
        yaw(current_heading):
            Compute rotational (about body z-axis) motion command
        normalize_angle(angle):
            Normalize any angle to the range [-pi, pi]        
    """

    def __init__(self, target_point: Point, target_heading: float = 0.0):
        """
        Initialize the 6-DOF PID Controller with a target point and heading.
        
        Args:
            target_point(Point): The target position in 3D space
            target_heading(float): The desired yaw angle in radians
        """

        self.target_point = target_point
        self.target_heading = target_heading

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

        self.prev_thrust_command = 0.0
        self.prev_surge_command = 0.0
        self.prev_sway_command = 0.0
        self.prev_yaw_command = 0.0

        self.max_cruise_accel = 10.0
        self.max_descent_accel = 2.0
        self.max_yaw_accel = np.pi / 18

        self.dt = 0.01 # 10ms / iteration

    def update_target_position(self, new_target_point: Point):
        """
        Update the controller's 3D position setpoint.

        Args:
            new_target_point(Point): New target position in 3D space
        """

        self.target_point = new_target_point

    def update_target_heading(self, new_target_heading: float):
        """
        Update the controller's heading setpoint.

        Args:
            new_target_heading(float): New target yaw angle in radians
        """

        self.target_heading = new_target_heading

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
        command_change = current_command - prev_command

        if abs(command_change) > max_change:
            limited_change = max_change if command_change > 0 else -max_change
            return prev_command + limited_change
        else:
            return command_change

    def thrust(self, current_altitude: float, is_descending: bool) -> float:
        """
        Compute PID Control command for vertical thrust, and limit acceleration
        during descent.

        Args:
            current_altitude(float): Drone's current altitude
            is_descending(bool): Flag to apply acceleration limit
        
        Returns:
            float: Computed PID Control command for vertical thrust
        """

        Kp, Ki, Kd = 5.0, 1.5, 3.5

        height_error = self.target_point.z - current_altitude
        height_error_derivative = (height_error - self.prev_height_error) / self.dt
        self.height_error_integral += height_error * self.dt

        thrust_command = (Kp * height_error +
                          Kd * height_error_derivative +
                          Ki * self.height_error_integral)
        
        if is_descending:
            thrust_command = self.limit_acceleration(thrust_command, self.prev_thrust_command, self.max_descent_accel)
        
        self.prev_height_error = height_error
        self.prev_thrust_command = thrust_command

        return thrust_command
    
    def surge(self, current_x: float) -> float:
        """
        Compute PID Control command for forward (body x-direction).

        Args:
            current_x(float): Drone's current x-coordinate in the world frame
        
        Returns:
            float: Computed PID Control command for forward motion
        """

        Kp, Ki, Kd = 1.3, 0.001, 4.3

        x_error = self.target_point.x - current_x
        x_error_derivative = (x_error - self.prev_x_error) / self.dt
        self.x_error_integral += x_error * self.dt

        surge_command = (Kp * x_error +
                         Kd * x_error_derivative +
                         Ki * self.x_error_integral)
        
        surge_command = self.limit_acceleration(surge_command, self.prev_surge_command, self.max_cruise_accel)
        
        self.prev_x_error = x_error
        self.prev_surge_command = surge_command

        return surge_command
    
    def sway(self, current_y: float) -> float:
        """
        Compute PID Control command for lateral (body y-direction).

        Args:
            current_y(float): Drone's current y-coordinate in the world frame
        
        Returns:
            float: Computed PID Control command for lateral motion
        """

        Kp, Ki, Kd = 1.3, 0.001, 4.3

        y_error = self.target_point.y - current_y
        y_error_derivative = (y_error - self.prev_y_error) / self.dt
        self.y_error_integral += y_error * self.dt

        sway_command = (Kp * y_error +
                        Kd * y_error_derivative +
                        Ki * self.y_error_integral)
        
        sway_command = self.limit_acceleration(sway_command, self.prev_sway_command, self.max_cruise_accel)
        
        self.prev_y_error = y_error
        self.prev_sway_command = sway_command
        
        return sway_command

    def roll_rate(self, current_roll_velocity: float) -> float:
        """
        Rate controller to stabilize roll.

        Args:
            current_roll_velocity(float):   Current angular velocity about body
                                            x-axis
        
        Returns:
            float: Computed PID Control command for roll motion
        """

        Kp, Ki, Kd = 1.0, 0.0, 0.0

        roll_error = -current_roll_velocity
        roll_error_derivative = (roll_error - self.prev_roll_error) / self.dt
        self.roll_error_integral += roll_error * self.dt

        roll_command = (Kp * roll_error +
                        Kd * roll_error_derivative +
                        Ki * self.roll_error_integral)
        
        self.prev_roll_error = roll_error

        return roll_command
    
    def pitch_rate(self, current_pitch_velocity: float) -> float:
        """
        Rate controller to stabilize pitch.

        Args:
            current_pitch_velocity(float):  Current angular velocity about body
                                            y-axis
        
        Returns:
            float: Computed PID Control command for pitch motion
        """
        
        Kp, Ki, Kd = 1.0, 0.0, 0.0

        pitch_error = -current_pitch_velocity
        pitch_error_derivative = (pitch_error - self.prev_pitch_error) / self.dt
        self.pitch_error_integral += pitch_error * self.dt

        pitch_command = (Kp * pitch_error +
                         Kd * pitch_error_derivative +
                         Ki * self.pitch_error_integral)
        
        self.prev_pitch_error = pitch_error

        return pitch_command
    
    def yaw(self, current_heading: float) -> float:
        """
        Compute PID Control command for rotational (about body z-axis) motion.

        Args:
            current_heading(float): Drone's current heading in world frame
        
        Returns:
            float: Computed PID Control command for rotational (yaw) motion
        """
        
        Kp, Ki, Kd = 0.3, 0.001, 0.7

        heading_error = self.target_heading - current_heading
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
        Clamp an angle to the range [-pi, pi].

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


class Platform:
    def __init__(self):
        # Platform Position Subscriber & Publisher
        self.position_sub = rospy.Subscriber('/platform/ground_truth', Odometry, self.odom_callback)
        self.position_pub = rospy.Publisher('/target_point', Point, queue_size=10)

        # Fiducial Marker Pose Subscriber
        self.marker_sub = rospy.Subscriber('/apriltag/pose', PoseStamped, self.marker_callback)
        self.marker_pose = Pose()
        self.marker_heading = 0.0
        self.marker_pose_detected = False
        
        self.position = Point()
        self.height = 0.2

    def odom_callback(self, data: Odometry):
        self.position = data.pose.pose.position
        self.position_pub.publish(self.position)

    def marker_callback(self, data: PoseStamped):
        self.marker_pose = data.pose
        marker_quat = [
            self.marker_pose.orientation.x,
            self.marker_pose.orientation.y,
            self.marker_pose.orientation.z,
            self.marker_pose.orientation.w
        ]
        self.marker_heading = euler_from_quaternion(marker_quat)[2]

        self.marker_pose_detected = True
        rospy.loginfo_once(CYAN + "AprilTag detected!" + RESET)

        # rospy.loginfo(f"AprilTag detected: "
        #               f"x={self.marker_pose.position.x:.2f}, "
        #               f"y={self.marker_pose.position.y:.2f}, " 
        #               f"z={self.marker_pose.position.z:.2f}, "
        #               f"heading={self.marker_heading:.6f}")

class Drone:
    def __init__(self):
        rospy.init_node('mission_sim', anonymous=True)

        # State Subscribers
        self.odom_sub = rospy.Subscriber('/quadrotor/ground_truth', Odometry, self.odom_callback)
        #self.imu_sub = rospy.Subscriber('/quadrotor/imu', Imu, self.imu_callback)
        
        # Control Publishers
        self.cmd_pub = rospy.Publisher('/quadrotor/cmd_force', Wrench, queue_size=10)
        self.motor_pub = rospy.Publisher('/quadrotor/motor_speeds', Float64MultiArray, queue_size=10)

        # Control signals
        self.command = Wrench()
        self.motor_msg = Float64MultiArray()
        
        # Define base motor speed
        self.base_speed = 900.0

        # Leg length
        self.leg_length = 0.15

        # Mission parameters
        self.platform = Platform()
        self.operating_altitude = 5.0
        self.hover_duration = 2.0

        # Fiducial Marker Targets
        self.marker_position = Point()
        self.marker_heading = 0.0

        # State variables
        self.current_position = Point()
        self.current_attitude = np.zeros(3)
        self.current_heading = self.current_attitude[2]
        self.current_velocity = Twist()

        # Controller
        self.controller = Controller(self.platform.position, 0.0)
        self.control_timer = rospy.Timer(rospy.Duration(1/100), self.fly) # 100Hz update rate

        # State machine
        self.mission_state = MissionState.TAKEOFF
        self.set_state_target()
        rospy.loginfo(CYAN + f"Mission state: {self.mission_state.value}" + RESET)
        self.state_start_time = rospy.Time.now()

        self.state_transitions = {
            MissionState.TAKEOFF: (self.at_operating_altitude, MissionState.CRUISE),
            MissionState.CRUISE: (self.marker_pose_detected, MissionState.TRACK),
            MissionState.TRACK: (self.at_marker_xy, MissionState.HOVER),
            MissionState.HOVER: (self.hover_complete, MissionState.ROTATE),
            MissionState.ROTATE: (self.aligned_with_marker, MissionState.DESCEND),
            MissionState.DESCEND: (self.at_platform_level, MissionState.LANDED),
            MissionState.LANDED: (None, None)
        }

    def imu_callback(self, data: Imu):
        self.current_velocity.angular = data.angular_velocity

        quat = [
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w
        ]
        
        self.current_attitude = euler_from_quaternion(quat)

        #rospy.loginfo(f"Current roll velocity: {self.current_velocity.angular.x:.2f}")
        #rospy.loginfo(f"Current pitch velocity: {self.current_velocity.angular.y:.2f}")

    def odom_callback(self, data: Odometry):
        self.current_velocity = data.twist.twist
        self.current_position = data.pose.pose.position
        
        quat = [
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w,
        ]
        
        self.current_attitude = euler_from_quaternion(quat)
        self.current_heading = self.current_attitude[2]

        #rospy.loginfo(f"Current position (x,y): {self.current_position.x:.2f}, {self.current_position.y:.2f}")
        #rospy.loginfo(f"Current height: {self.current_position.z:.2f}")
        #rospy.loginfo(f"Current orientation about x axis (roll): {self.current_attitude[0]:.2f}")
        #rospy.loginfo(f"Current orientation about y axis (pitch): {self.current_attitude[1]:.2f}")
        #rospy.loginfo(f"Current heading: {self.current_heading:.2f}")

    def at_operating_altitude(self):
        dz = abs(self.operating_altitude - self.current_position.z)
        vz = self.current_velocity.linear.z
        return dz <= 0.1 and vz <= 0.1
    
    def marker_pose_detected(self):
        return self.platform.marker_pose_detected
    
    def estimate_marker_position(self):
        self.marker_position.x = self.current_position.x - self.platform.marker_pose.position.x
        self.marker_position.y = self.current_position.y - self.platform.marker_pose.position.y
        self.marker_heading = self.platform.marker_heading
    
    def at_marker_xy(self):
        self.estimate_marker_position()

        dx = self.marker_position.x - self.current_position.x
        dy = self.marker_position.y - self.current_position.y
        vx = self.current_velocity.linear.x
        vy = self.current_velocity.linear.y
        return dx <= 0.05 and dy <= 0.05 and vx <= 0.05 and vy <= 0.05
    
    def hover_complete(self):
        elapsed_time = rospy.Time.now() - self.state_start_time
        return elapsed_time.to_sec() >= self.hover_duration
    
    def aligned_with_marker(self):
        heading_error = self.controller.normalize_angle(self.current_heading - self.marker_heading)
        yaw_velocity = self.current_velocity.angular.z
        return abs(heading_error) <= np.pi / 36 and yaw_velocity < 0.1
    
    def at_platform_level(self):
        return self.current_position.z <= self.platform.height + self.leg_length
    
    def update_mission_state(self):
        if self.mission_state == MissionState.LANDED:
            rospy.loginfo_once(CYAN + f"Landed at x={self.current_position.x:.2f}, y={self.current_position.y:.2f}" + RESET)
            return
        
        condition, next_state = self.state_transitions[self.mission_state]

        if condition and condition():
            self.transition_to_state(next_state)

    def transition_to_state(self, next_state):
        rospy.loginfo(CYAN + f"Mission state transition: {self.mission_state.value} -> {next_state.value}" + RESET)

        self.mission_state = next_state
        
        if self.mission_state == MissionState.TRACK:
            self.estimate_marker_position()

        self.state_start_time = rospy.Time.now()

        self.set_state_target()

    def set_state_target(self):
        if self.mission_state == MissionState.TAKEOFF:
            target = Point(self.current_position.x, self.current_position.y, self.operating_altitude)

        elif self.mission_state == MissionState.CRUISE:
            target = Point(self.platform.position.x, self.platform.position.y, self.operating_altitude)

        elif self.mission_state == MissionState.TRACK:
            target = Point(self.marker_position.x, self.marker_position.y, self.operating_altitude)

        elif self.mission_state == MissionState.HOVER:
            target = Point(self.current_position.x, self.current_position.y, self.operating_altitude)

        elif self.mission_state == MissionState.ROTATE:
            target = Point(self.current_position.x, self.current_position.y, self.operating_altitude)
            self.controller.update_target_heading(self.marker_heading)

        elif self.mission_state == MissionState.DESCEND:
            target = Point(self.marker_position.x, self.marker_position.y, self.platform.height)
        
        elif self.mission_state == MissionState.LANDED:
            target = self.current_position

        self.controller.update_target_position(target)
        #rospy.loginfo(f"Updated target to: x={target.x:.2f}, y={target.y:.2f}, z={target.z:.2f}")

    def fly(self, event):
        self.update_mission_state()

        if self.mission_state == MissionState.LANDED:
            self.command = Wrench()
            #self.motor_msg.data = np.zeros(4)

        else:
            is_descending = (self.mission_state == MissionState.DESCEND)

            self.command.force.z = self.controller.thrust(self.current_position.z, is_descending)
            self.command.force.x = self.controller.surge(self.current_position.x)
            self.command.force.y = self.controller.sway(self.current_position.y)
            self.command.torque.z = self.controller.yaw(self.current_heading)
            # thrust_command = self.controller.thrust(self.current_position.z)
            # self.motor_msg.data = [self.base_speed + thrust_command] * 4

        # Stabilize roll and pitch using rate controllers
        self.command.torque.x = self.controller.roll_rate(self.current_velocity.angular.x)
        self.command.torque.y = self.controller.pitch_rate(self.current_velocity.angular.y)

        #self.motor_pub.publish(self.motor_msg)
        self.cmd_pub.publish(self.command)


if __name__ == "__main__":
    try:
        drone = Drone()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass