#!/usr/bin/env python3

import rospy
import numpy as np
from enum import Enum
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Twist, Wrench
from tf.transformations import euler_from_quaternion

# Colour for Logging
CYAN = "\033[96m"
RESET = "\033[0m"


class MissionState(Enum):
    TAKEOFF = "TAKEOFF"
    CRUISE = "CRUISE"
    HOVER = "HOVER"
    DESCEND = "DESCEND"
    LANDED = "LANDED"


class Controller:
    def __init__(self, target_point: Point, target_heading: float = 0.0):
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

        self.dt = 0.01 # 10ms / iteration

    def update_target(self, new_target: Point):
        self.target_point = new_target

    def thrust(self, current_altitude: float) -> float:
        Kp, Ki, Kd = 5.0, 1.5, 3.5

        height_error = self.target_point.z - current_altitude
        height_error_derivative = (height_error - self.prev_height_error) / self.dt
        self.height_error_integral += height_error * self.dt

        thrust_command = (Kp * height_error +
                          Kd * height_error_derivative +
                          Ki * self.height_error_integral)
        
        self.prev_height_error = height_error

        return thrust_command
    
    def surge(self, current_x: float) -> float:
        Kp, Ki, Kd = 1.3, 0.001, 3.7

        x_error = self.target_point.x - current_x
        x_error_derivative = (x_error - self.prev_x_error) / self.dt
        self.x_error_integral += x_error * self.dt

        surge_command = (Kp * x_error +
                         Kd * x_error_derivative +
                         Ki * self.x_error_integral)
        
        self.prev_x_error = x_error

        return surge_command
    
    def sway(self, current_y: float) -> float:
        Kp, Ki, Kd = 1.3, 0.001, 3.7

        y_error = self.target_point.y - current_y
        y_error_derivative = (y_error - self.prev_y_error) / self.dt
        self.y_error_integral += y_error * self.dt

        sway_command = (Kp * y_error +
                        Kd * y_error_derivative +
                        Ki * self.y_error_integral)
        
        self.prev_y_error = y_error
        
        return sway_command

    def roll_rate(self, current_roll_velocity: float) -> float:
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
        Kp, Ki, Kd = 0.3, 0.0, 0.0

        heading_error = self.target_heading - current_heading
        heading_error = self.normalize_angle(heading_error)

        heading_error_derivative = (heading_error - self.prev_heading_error) / self.dt
        self.heading_error_integral += heading_error * self.dt

        yaw_command = (Kp * heading_error +
                       Kd * heading_error_derivative +
                       Ki * self.heading_error_integral)
        
        self.prev_heading_error = heading_error

        return yaw_command

    def normalize_angle(self, angle: float) -> float:
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
        
        self.position = Point()
        self.height = 0.2

    def odom_callback(self, data: Odometry):
        self.position = data.pose.pose.position
        self.position_pub.publish(self.position)


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
        self.operating_altitude = 3.0
        self.target_heading = 0.0
        self.hover_duration = 3.0

        # State variables
        self.current_position = Point()
        self.current_attitude = np.zeros(3)
        self.current_velocity = Twist()

        # Controller
        self.controller = Controller(self.platform.position, self.target_heading)
        self.control_timer = rospy.Timer(rospy.Duration(1/100), self.fly) # 100Hz update rate

        # State machine
        self.mission_state = MissionState.TAKEOFF
        self.set_state_target()
        rospy.loginfo(CYAN + f"Mission state: {self.mission_state.value}" + RESET)
        self.state_start_time = rospy.Time.now()

        self.state_transitions = {
            MissionState.TAKEOFF: (self.at_operating_altitude, MissionState.CRUISE),
            MissionState.CRUISE: (self.at_platform_xy, MissionState.HOVER),
            MissionState.HOVER: (self.hover_complete, MissionState.DESCEND),
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

        #rospy.loginfo(f"Current position (x,y): {self.current_position.x:.2f}, {self.current_position.y:.2f}")
        #rospy.loginfo(f"Current height: {self.current_position.z:.2f}")
        #rospy.loginfo(f"Current orientation about x axis (roll): {self.current_attitude[0]:.2f}")
        #rospy.loginfo(f"Current orientation about y axis (pitch): {self.current_attitude[1]:.2f}")
        #rospy.loginfo(f"Current heading: {self.current_attitude[2]:.2f}")

    def at_operating_altitude(self):
        dz = abs(self.operating_altitude - self.current_position.z)
        vz = self.current_velocity.linear.z
        return dz <= 0.1 and vz <= 0.1
    
    def at_platform_xy(self):
        dx = self.platform.position.x - self.current_position.x
        dy = self.platform.position.y - self.current_position.y
        vx = self.current_velocity.linear.x
        vy = self.current_velocity.linear.y
        return dx <= 0.05 and dy <= 0.05 and vx <= 0.05 and vy <= 0.05
    
    def hover_complete(self):
        elapsed_time = rospy.Time.now() - self.state_start_time
        return elapsed_time.to_sec() >= self.hover_duration
    
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
        self.state_start_time = rospy.Time.now()

        self.set_state_target()

    def set_state_target(self):
        if self.mission_state == MissionState.TAKEOFF:
            target = Point(self.current_position.x, self.current_position.y, self.operating_altitude)

        elif self.mission_state == MissionState.CRUISE:
            target = Point(self.platform.position.x, self.platform.position.y, self.operating_altitude)

        elif self.mission_state == MissionState.HOVER:
            target = Point(self.current_position.x, self.current_position.y, self.operating_altitude)

        elif self.mission_state == MissionState.DESCEND:
            target = Point(self.platform.position.x, self.platform.position.y, self.platform.height)
        
        elif self.mission_state == MissionState.LANDED:
            target = self.current_position

        self.controller.update_target(target)
        #rospy.loginfo(f"Updated target to: x={target.x:.2f}, y={target.y:.2f}, z={target.z:.2f}")

    def fly(self, event):
        self.update_mission_state()

        if self.mission_state == MissionState.LANDED:
            self.command.force.z = 0.0
            self.command.force.x, self.command.force.y = 0.0, 0.0
            #self.motor_msg.data = [0.0, 0.0, 0.0, 0.0]

        else:
            self.command.force.z = self.controller.thrust(self.current_position.z)
            self.command.force.x = self.controller.surge(self.current_position.x)
            self.command.force.y = self.controller.sway(self.current_position.y)
            # thrust_command = self.controller.thrust(self.current_position.z)
            # self.motor_msg.data = [self.base_speed + thrust_command] * 4

        # Stabilize roll and pitch using rate controllers
        self.command.torque.x = self.controller.roll_rate(self.current_velocity.angular.x)
        self.command.torque.y = self.controller.pitch_rate(self.current_velocity.angular.y)

        # Heading controller
        self.command.torque.z = self.controller.yaw(self.current_attitude[2])

        #self.motor_pub.publish(self.motor_msg)
        self.cmd_pub.publish(self.command)


if __name__ == "__main__":
    try:
        drone = Drone()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass