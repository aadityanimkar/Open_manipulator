#! /usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import cv2
from rospy import Subscriber


class CameraTransformPublisher:
    def __init__(self):
        # Initialize the ROS Node
        rospy.init_node('camera_transform_publisher', anonymous=True)

        # Create a TF2 broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Initialize CvBridge to convert ROS images to OpenCV
        self.bridge = CvBridge()

        # Subscribe to the camera feed (change topic name as per your setup)
        self.image_sub = Subscriber("/camera/image_raw", Image, self.image_callback)

        # Set rate to 10 Hz for publishing transforms
        self.rate = rospy.Rate(10)

        # Initialize the camera frame transform data
        self.camera_frame = "camera_link"
        self.parent_frame = "world"  # Use the parent frame for your setup

    def image_callback(self, msg):
        """
        This callback function is called when a new image message is received.
        """
        try:
            # Convert the ROS image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Display the image in an OpenCV window (for debugging)
            cv2.imshow("Camera Feed", cv_image)
            cv2.waitKey(1)  # This ensures the image window is updated

        except Exception as e:
            rospy.logerr("Error converting image: %s", str(e))

    def publish_transform(self):
        """
        Publish the transform from the camera frame to the world frame.
        """
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.parent_frame  # Parent frame (e.g., "world")
        t.child_frame_id = self.camera_frame  # Camera frame (e.g., "camera_link")

        # Set translation and rotation for the camera
        t.transform.translation.x = 0.5  # Example position of the camera
        t.transform.translation.y = 0.0
        t.transform.translation.z = 1.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

    def run(self):
        """
        Run the node to continuously publish transforms and process images.
        """
        rospy.loginfo("Camera Transform Publisher Node started")

        while not rospy.is_shutdown():
            self.publish_transform()  # Publish the transform every loop
            self.rate.sleep()


if __name__ == '__main__':
    try:
        # Instantiate and run the camera transform publisher node
        camera_transform_publisher = CameraTransformPublisher()
        camera_transform_publisher.run()

    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted. Shutting down.")
    finally:
        cv2.destroyAllWindows()  # Clean up OpenCV windows when exiting
