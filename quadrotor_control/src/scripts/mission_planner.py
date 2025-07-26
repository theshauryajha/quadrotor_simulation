#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This module defines a ROS Node responsible for high-level mission planning of
a Drone with an objective of landing autonomously on a moving platform using
a fiducial marker for vision based tracking.

It managages the mission planner as a finite state machine, using live updates
from the Drone's sensors, the Platform's GPS and the fiducial marker detection
from the Vision Node, and determines the target setpoint for the current
mission state for the Control Node to navigate to.

This ROS Node subscribes to the Ground Truth of the Drone and Platform, as well
as the Pose estimated from AprilTag detections, and publishes the position and
heading setpoints, as well as the current mission state.
"""

import rospy
import numpy as np
from enum import Enum
from std_msgs.msg import String, Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PoseStamped, Twist
from tf.transformations import euler_from_quaternion

# Colour for Logging
CYAN = "\033[96m"
RESET = "\033[0m"


class MissionState(Enum):
    """
    Enumeration of mission phases for Drone operation.

    This state machine define the various stages of the Drone's mission,
    from takeoff to landing. Each state corresponds to a distinct behaviour
    mode and setpoint for the control logic.

    States:
        TAKEOFF:    Drone ascends to the operating altitude at its drone
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


class Planner:
    def __init__(self):
        """
        Initializes the Planner node for high-level drone mission planning.
        
        This constructor sets up the necessary ROS infrastructure including
        subscribers to the sensor data (Drone, Platform and Fiducial Marker)
        and publishers for mission details (current mission state, target
        position and heading).

        It initializes the mission state machine, target setpoints and drone,
        platform and marker data attributes required for planning the mission.

        Subscribers:
            * /quadrotor/ground_truth (Odometry): Drone state
            * /platform/ground_truth (Odometry): Platform state
            * /apriltag/pose (PoseStamped): Marker detection
        
        Publishers:
            * /target/position (Point): Position setpoint for navigation
            * /target/heading (Float64): Heading setpoint for rotation
            * /mission_state (String): Current mission phase

        This method also initializes the main update loop at 100Hz frequency
        using a ROS Timer.
        """
        
        rospy.init_node('planning_node', anonymous=True)

        # Subcribes to Drone, Platform and Marker data
        self.drone_sub = rospy.Subscriber('/quadrotor/ground_truth', Odometry, self.drone_callback)
        self.platform_sub = rospy.Subscriber('/platform/ground_truth', Odometry, self.platform_callback)
        self.marker_sub = rospy.Subscriber('/apriltag/pose', PoseStamped, self.marker_callback)

        # Publishes target position, heading and current mission state
        self.position_pub = rospy.Publisher('/target/position', Point, queue_size=10)
        self.heading_pub = rospy.Publisher('/target/heading', Float64, queue_size=10)
        self.state_pub = rospy.Publisher('/mission_state', String, queue_size=10)

        # Target position and heading
        self.target_position = Point()
        self.target_heading = 0.0

        # Drone data
        self.drone_position = Point()
        self.drone_attitude = np.zeros(3)
        self.drone_heading = self.drone_attitude[2]
        self.drone_velocity = Twist()

        # Mission parameters
        self.operating_altitude = 5.0
        self.hover_duration = 2.0
        
        # Platform data
        self.platform_position = Point()
        self.platform_height = 0.2

        # Marker data
        self.marker_position = Point()
        self.marker_heading = 0.0
        self.pose_detected = False

        # State machine
        self.state_transitions = {
            MissionState.TAKEOFF: (self.at_operating_altitude, MissionState.CRUISE),
            MissionState.CRUISE: (self.marker_pose_detected, MissionState.TRACK),
            MissionState.TRACK: (self.at_marker_xy, MissionState.HOVER),
            MissionState.HOVER: (self.hover_complete, MissionState.ROTATE),
            MissionState.ROTATE: (self.aligned_with_marker, MissionState.DESCEND),
            MissionState.DESCEND: (self.at_platform_level, MissionState.LANDED),
            MissionState.LANDED: (None, None)
        }

        self.mission_state = MissionState.TAKEOFF
        self.set_state_target()
        rospy.loginfo(CYAN + f"Mission state: {self.mission_state.value}" + RESET)
        self.state_start_time = rospy.Time.now()

        # Time to update Mission State
        self.timer = rospy.Timer(rospy.Duration(1/100), self.update_mission_state) # 100 Hz update rate

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

    def platform_callback(self, data: Odometry):
        """
        Updates the platform's position obtained from ground truth and logs the
        global frame coordinates.

        Args:
            data(Odometry): Incoming position data from ground truth
        """

        self.platform_position = data.pose.pose.position
        # rospy.loginfo(f"Current position (x,y): {self.platform_position.x:.2f}, {self.platform_position.y:.2f}")

    def marker_callback(self, data: PoseStamped):
        """
        Uses coordinate frame translation to determine and update the position
        and heading of the marker in the global frame using the Pose of the
        Drone in the global frame and that of the fiducial marker relative to
        the Drone, obtained from the Vision node.

        Args:
            data(PoseStamped):  Incoming Pose data from fiducial marker
                                detection
        """

        self.marker_position.x = self.drone_position.x - data.pose.position.x
        self.marker_position.y = self.drone_position.y - data.pose.position.y

        marker_quat = [
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w
        ]

        self.marker_heading = self.drone_heading - euler_from_quaternion(marker_quat)[2]

        self.pose_detected = True
        rospy.loginfo_once(CYAN + "AprilTag detected!" + RESET)

        # rospy.loginfo(f"AprilTag detected: "
        #               f"x={self.marker_pose.position.x:.2f}, "
        #               f"y={self.marker_pose.position.y:.2f}, " 
        #               f"z={self.marker_pose.position.z:.2f}, "
        #               f"heading={self.marker_heading:.6f}")

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

    def at_operating_altitude(self) -> bool:
        """
        Determines whether the Drone is stable at its operating altitude.

        Returns:
            bool:   True if altitude is within a tolerance range and vertical
                    velocity is low.
        """

        dz = abs(self.operating_altitude - self.drone_position.z)
        vz = self.drone_velocity.linear.z
        return dz <= 0.1 and vz <= 0.1
    
    def marker_pose_detected(self) -> bool:
        """
        Determines whether Pose data from the fiducial marker on the landing
        platform is available.

        Returns:
            bool: True if Pose data from the fiducial marker is available.
        """

        return self.pose_detected
    
    def at_marker_xy(self) -> bool:
        """
        Determines whether the Drone is stable vertically above the fiducial
        marker.

        Returns:
            bool:   True if the (x,y) coordinates of the Drone are close to the
                    (x,y) coordinates of the fiducial marker and the cruise
                    velocities are low.
        """

        dx = self.marker_position.x - self.drone_position.x
        dy = self.marker_position.y - self.drone_position.y
        vx = self.drone_velocity.linear.x
        vy = self.drone_velocity.linear.y
        return dx <= 0.05 and dy <= 0.05 and vx <= 0.05 and vy <= 0.05
    
    def hover_complete(self) -> bool:
        """
        Determines if the Drone has completed its hover phase.

        Returns:
            bool:   True if the time elapsed between the start of the hover
                    phase is equal to or greater than the hover duration.
        """

        elapsed_time = rospy.Time.now() - self.state_start_time
        return elapsed_time.to_sec() >= self.hover_duration
    
    def aligned_with_marker(self) -> bool:
        """
        Determines if the Drone is stable and aligned with the fiducial marker.

        Returns:
            bool:   True if the heading error between the Drone and the marker
                    is below a threshold and the Drone's yaw velocity is low.
        """

        heading_error = self.normalize_angle(self.drone_heading - self.marker_heading)
        yaw_velocity = self.drone_velocity.angular.z
        return abs(heading_error) <= np.pi / 36 and yaw_velocity < 0.1
    
    def at_platform_level(self) -> bool:
        """
        Determines if the Drone has landed on the platform.

        Returns:
            bool:   True if the current altitude of the Drone is equal to the
                    height of the platform.
        """

        return self.drone_position.z <= self.platform_height + 0.15

    def transition_to_state(self, next_state):
        """
        Transitions to the next mission state, updates the target setpoint
        accordingly and logs the transition.

        Args:
            next_state(MissionState): The next mission state to switch to
        """

        rospy.loginfo(CYAN + f"Mission state transition: {self.mission_state.value} -> {next_state.value}" + RESET)

        self.mission_state = next_state
        self.state_start_time = rospy.Time.now()

        self.set_state_target()

    def set_state_target(self):
        """
        Updates the target setpoint and heading based on the new mission phase.
        """

        if self.mission_state == MissionState.TAKEOFF:
            self.target_position = Point(self.drone_position.x, self.drone_position.y, self.operating_altitude)

        elif self.mission_state == MissionState.CRUISE:
            self.target_position = Point(self.platform_position.x, self.platform_position.y, self.operating_altitude)

        elif self.mission_state == MissionState.TRACK:
            self.target_position = Point(self.marker_position.x, self.marker_position.y, self.operating_altitude)

        elif self.mission_state == MissionState.HOVER:
            self.target_position = Point(self.drone_position.x, self.drone_position.y, self.operating_altitude)

        elif self.mission_state == MissionState.ROTATE:
            self.target_position = Point(self.drone_position.x, self.drone_position.y, self.operating_altitude)
            self.target_heading = self.marker_heading

        elif self.mission_state == MissionState.DESCEND:
            self.target_position = Point(self.marker_position.x, self.marker_position.y, self.platform_height)
        
        elif self.mission_state == MissionState.LANDED:
            self.target_position = self.drone_position
    
    def update_mission_state(self, event):
        """
        Main planning loop, executes at 100Hz frequency using a ROS Timer.

        Evaluates the current mission state and transitions to the next state
        if its condition is satisfied. Publishes the current mission state and
        the target position and heading.
        """

        if self.mission_state == MissionState.LANDED:
            rospy.loginfo_once(CYAN + f"Landed at x={self.drone_position.x:.2f}, y={self.drone_position.y:.2f}" + RESET)
        
        condition, next_state = self.state_transitions[self.mission_state]

        if condition and condition():
            self.transition_to_state(next_state)

        self.state_pub.publish(self.mission_state.value)
        self.position_pub.publish(self.target_position)
        self.heading_pub.publish(self.target_heading)


if __name__ == "__main__":
    try:
        planner = Planner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass