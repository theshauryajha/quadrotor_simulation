#!/usr/bin/env python3

import rospy
from math import sqrt
import numpy as np
from enum import Enum
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Wrench, Vector3
from tf.transformations import euler_from_quaternion


class MissionState(Enum):
    TAKEOFF = "TAKEOFF"
    HOVER = "HOVER"
    CRUISE = "CRUISE"
    DESCEND = "DESCEND"
    LANDED = "LANDED"


class Controller:
    def __init__(self, target_point: Point):
        self.target_point = target_point

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
        Kp, Ki, Kd = 1.3, 0.001, 2.9

        x_error = self.target_point.x - current_x
        x_error_derivative = (x_error - self.prev_x_error) / self.dt
        self.x_error_integral += x_error * self.dt

        surge_command = (Kp * x_error +
                         Kd * x_error_derivative +
                         Ki * self.x_error_integral)
        
        self.prev_x_error = x_error

        return surge_command
    
    def sway(self, current_y: float) -> float:
        Kp, Ki, Kd = 1.3, 0.001, 2.9

        y_error = self.target_point.y - current_y
        y_error_derivative = (y_error - self.prev_y_error) / self.dt
        self.y_error_integral += y_error * self.dt

        sway_command = (Kp * y_error +
                        Kd * y_error_derivative +
                        Ki * self.y_error_integral)
        
        self.prev_y_error = y_error
        
        return sway_command

    def roll(self, current_roll_velocity: float) -> float:
        Kp, Ki, Kd = 1.0, 0.0, 0.0

        roll_error = -current_roll_velocity
        roll_error_derivative = (roll_error - self.prev_roll_error) / self.dt
        self.roll_error_integral += roll_error * self.dt

        roll_command = (Kp * roll_error +
                        Kd * roll_error_derivative +
                        Ki * self.roll_error_integral)
        
        self.prev_roll_error = roll_error

        return roll_command
    
    def pitch(self, current_pitch_velocity: float) -> float:
        Kp, Ki, Kd = 1.0, 0.0, 0.0

        pitch_error = -current_pitch_velocity
        pitch_error_derivative = (pitch_error - self.prev_pitch_error) / self.dt
        self.pitch_error_integral += pitch_error * self.dt

        pitch_command = (Kp * pitch_error +
                         Kd * pitch_error_derivative +
                         Ki * self.pitch_error_integral)
        
        self.prev_pitch_error = pitch_error

        return pitch_command

    def normalize_angle(self, angle: float) -> float:
        while angle < -np.pi:
            angle += 2 * np.pi
        while angle > np.pi:
            angle -= 2 * np.pi
        return angle


class Drone:
    def __init__(self):
        rospy.init_node('mission_sim', anonymous=True)

        # State Subscribers
        self.odom_sub = rospy.Subscriber('/ground_truth/state', Odometry, self.odom_callback)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        
        # Control Publishers
        self.cmd_pub = rospy.Publisher('/quadrotor/cmd_force', Wrench, queue_size=10)
        self.motor_pub = rospy.Publisher('/quadrotor/motor_speeds', Float64MultiArray, queue_size=10)

        # Control signals
        self.command = Wrench()
        self.motor_msg = Float64MultiArray()
        
        # Define base motor speed
        self.base_speed = 900.0

        # Goal Publisher for plotting
        self.goal_pub = rospy.Publisher('/target_point', Point, queue_size=10)

        # Mission parameters
        self.operating_altitude = 5.0
        self.target_position = Point(4.0, 3.0, self.operating_altitude)
        self.hover_duration = 5.0

        # State variables
        self.current_position = Point()
        self.current_attitude = np.zeros(3)
        self.current_angular_velocity = Vector3()

        # Controller
        self.controller = Controller(self.target_position)
        self.control_timer = rospy.Timer(rospy.Duration(1/100), self.fly) # 100Hz update rate

        # State machine
        self.mission_state = MissionState.TAKEOFF
        self.set_state_target()
        self.state_start_time = rospy.Time.now()

        self.state_transitions = {
            MissionState.TAKEOFF: (self.at_operating_altitude, MissionState.CRUISE),
            MissionState.CRUISE: (self.at_target_xy, MissionState.HOVER),
            MissionState.HOVER: (self.hover_complete, MissionState.DESCEND),
            MissionState.DESCEND: (self.at_ground_level, MissionState.LANDED),
            MissionState.LANDED: (None, None)
        }

    def imu_callback(self, data: Imu):
        self.current_angular_velocity = data.angular_velocity
        
        #rospy.loginfo(f"Current pitch velocity: {self.current_angular_velocity.x:2f}")
        #rospy.loginfo(f"Current roll velocity: {self.current_angular_velocity.y:2f}")

    def odom_callback(self, data: Odometry):
        self.current_position = data.pose.pose.position

        quat = [data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z,
                data.pose.pose.orientation.w]
        
        self.current_attitude = euler_from_quaternion(quat)

        #rospy.loginfo(f"Current position (x,y): {self.current_position.x:.2f}, {self.current_position.y:.2f}")
        #rospy.loginfo(f"Current height: {self.current_position.z:.2f}")
        #rospy.loginfo(f"Current heading: {self.current_attitude[2]:.2f}")

    def at_operating_altitude(self):
        return abs(self.target_position.z - self.current_position.z) <= 0.1
    
    def at_target_xy(self):
        dx = self.target_position.x - self.current_position.x
        dy = self.target_position.y - self.current_position.y
        return dx <= 0.05 and dy <= 0.05
    
    def hover_complete(self):
        elapsed_time = rospy.Time.now() - self.state_start_time
        return elapsed_time.to_sec() >= self.hover_duration
    
    def at_ground_level(self):
        return self.current_position.z <= 0.15
    
    def update_mission_state(self):
        if self.mission_state == MissionState.LANDED:
            return
        
        condition, next_state = self.state_transitions[self.mission_state]

        if condition and condition():
            self.transition_to_state(next_state)

    def transition_to_state(self, next_state):
        rospy.logwarn(f"Mission state transition: {self.mission_state.value} -> {next_state.value}")

        self.mission_state = next_state
        self.state_start_time = rospy.Time.now()

        self.set_state_target()

    def set_state_target(self):
        if self.mission_state == MissionState.TAKEOFF:
            target = Point(self.current_position.x, self.current_position.y, self.operating_altitude)

        elif self.mission_state == MissionState.CRUISE:
            target = self.target_position

        elif self.mission_state == MissionState.HOVER:
            target = self.target_position

        elif self.mission_state == MissionState.DESCEND:
            target = Point(self.target_position.x, self.target_position.y, 0.0)
        
        elif self.mission_state == MissionState.LANDED:
            target = self.current_position

        self.controller.update_target(target)
        #rospy.logwarn(f"Updated target to: x={target.x:.2f}, y={target.y:.2f}, z={target.z:.2f}")

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

        # Stabilize pitch and roll using rate controllers
        self.command.torque.x = self.controller.pitch(self.current_angular_velocity.x)
        self.command.torque.y = self.controller.roll(self.current_angular_velocity.y)

        #self.motor_pub.publish(self.motor_msg)
        self.cmd_pub.publish(self.command)
        self.goal_pub.publish(self.target_position)


if __name__ == "__main__":
    try:
        drone = Drone()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass