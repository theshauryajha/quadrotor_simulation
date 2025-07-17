#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from apriltag_ros.msg import AprilTagDetectionArray

class AprilTagDetector:
    def __init__(self):
        rospy.init_node('apriltag_detector', anonymous=True)
        
        # Subscribe to AprilTag detections
        self.tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)
        
        # Publisher for tag pose (ID 23 specifically)
        self.pose_pub = rospy.Publisher('/apriltag_23/pose', PoseStamped, queue_size=10)
        
        rospy.loginfo("AprilTag detector node started - looking for tag ID 23")
        
    def tag_callback(self, data):
        """Process AprilTag detections and publish pose for tag ID 23"""
        for detection in data.detections:
            # Check if this is tag ID 23
            if detection.id[0] == 23:
                # Create pose message
                pose_msg = PoseStamped()
                pose_msg.header = data.header
                pose_msg.pose = detection.pose.pose.pose
                
                # Publish the pose
                self.pose_pub.publish(pose_msg)
                
                rospy.loginfo(f"AprilTag 23 detected at position: "
                            f"x={pose_msg.pose.position.x:.3f}, "
                            f"y={pose_msg.pose.position.y:.3f}, "
                            f"z={pose_msg.pose.position.z:.3f}")

if __name__ == '__main__':
    try:
        detector = AprilTagDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass