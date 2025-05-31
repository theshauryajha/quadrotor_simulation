#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

class DroneLQR:
    def __init__(self):
        rospy.init_node('quadrotor_lqr', anonymous=True)

        self.odom_sub = rospy.Subscriber('/ground_truth/state', Odometry, self.odom_callback)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        
        self.motor_pub = rospy.Publisher('/quadrotor/motor_speeds', Float64MultiArray, queue_size=10)
        self.motor_msg = Float64MultiArray()

        self.control_timer = rospy.Timer(rospy.Duration(1/100), self.fly) # 100Hz update rate

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

    def fly(self, event):
        self.motor_msg.data = [1000.0, 1000.0, 1000.0, 1000.0]
        self.motor_pub.publish(self.motor_msg)


if __name__ == "__main__":
    try:
        drone = DroneLQR()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass