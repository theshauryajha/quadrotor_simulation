#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from apriltag_ros.msg import AprilTagDetectionArray

# Colour for Logging
CYAN = "\033[96m"
RESET = "\033[0m"


class AprilTagDetector:
    def __init__(self):
        rospy.init_node('vision_node', anonymous=True)
        
        # Subscribe to AprilTag detections
        self.tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)
        
        # Publisher for tag pose (ID 23 specifically)
        self.pose_pub = rospy.Publisher('/apriltag/pose', PoseStamped, queue_size=10)
        
        rospy.loginfo(CYAN + "AprilTag detector node started - looking for tag ID 23" + RESET)
        
    def tag_callback(self, data):
        """Process AprilTag detections and publish pose for tag ID 23"""
        for detection in data.detections:
            # Check if this is tag ID 23
            if detection.id[0] == 23:
                # Create pose message
                pose_msg = PoseStamped()
                pose_msg.header = data.header
                pose_msg.pose = detection.pose.pose.pose
                
                 # Convention issue
                pose_msg.pose.position.x *= -1
                
                # Publish the pose
                self.pose_pub.publish(pose_msg)


if __name__ == '__main__':
    try:
        detector = AprilTagDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass