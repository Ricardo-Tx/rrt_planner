#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class EKFWithPath:
    def __init__(self):
        rospy.init_node('amcl_with_path_node', anonymous=True)

        # Create a publisher for the Path
        self.path_pub = rospy.Publisher('/amcl_path', Path, queue_size=10)

        # Initialize a Path message
        self.path = Path()
        self.path.header.frame_id = "map"  # Adjust if necessary
        
        # Subscribe to the /odometry/filtered topic
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.odometry_callback)

    def odometry_callback(self, msg):
        # Extract the position from the Odometry message
        position = msg.pose.pose.position  # Getting position from the Odometry message

        # Create a PoseStamped message for the Path
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()  # Set the current timestamp
        pose.header.frame_id = self.path.header.frame_id

        # Set the position and orientation of the pose
        pose.pose.position.x = position.x
        pose.pose.position.y = position.y
        pose.pose.position.z = position.z
        pose.pose.orientation = msg.pose.pose.orientation  # Use orientation from Odometry

        # Append the pose to the Path
        self.path.poses.append(pose)

        # Limit the number of poses stored in the Path (optional)
        MAX_PATH_LENGTH = 1000  # Adjust as necessary
        if len(self.path.poses) > MAX_PATH_LENGTH:
            self.path.poses.pop(0)  # Remove the oldest pose

        # Update the header timestamp of the path
        self.path.header.stamp = rospy.Time.now()

        # Publish the Path immediately after receiving the odometry message
        self.path_pub.publish(self.path)

if __name__ == '__main__':
    try:
        ekf_with_path = EKFWithPath()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
