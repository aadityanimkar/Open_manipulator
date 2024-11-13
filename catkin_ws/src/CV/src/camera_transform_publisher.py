#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped, PoseStamped
import cv2
from cv_bridge import CvBridge
import image_transport


class CameraTransformPublisher:
    def __init__(self):
        # Initialize the ROS Node
        rospy.init_node('camera_transform_publisher', anonymous=True)

        # Create a TF2 broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Initialize the CvBridge to convert ROS images to OpenCV
        self.bridge = CvBridge()

        # Subscribe to the camera feed (adjust topic as needed)
        self.image_sub = image_transport.ImageTransport(rospy).subscribe("/camera/image_raw", 1, self.image_callback)

        # Publish transforms every 10 Hz
        self.rate = rospy.Rate(10)

        # Initialize the camera frame transform data
        self.camera_frame = "camera_link"
        self.parent_frame = "world"  # Or use any other parent frame (e.g., robot base)

    def image_callback(self, msg):
        """
        This callback function will be called when a new image message is received.
        """
        try:
            # Convert the ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # You can now use the `cv_image` (OpenCV format) for any further processing

            # (Optional) Display the image in a window (just to show you received it)
            cv2.imshow("Camera Feed", cv_image)
            cv2.waitKey(1)  # Display the frame

        except Exception as e:
            rospy.logerr("Error converting image: %s", str(e))

    def publish_transform(self):
        """
        This function publishes the transform from the camera to the world frame.
        """
        # Create a TransformStamped message
        t = TransformStamped()

        # Set the parent frame (this is typically the robot's base or "world")
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.parent_frame  # World frame
        t.child_frame_id = self.camera_frame  # Camera frame

        # Define a simple transform (you can change this depending on your setup)
        # For example, assume the camera is at a fixed position relative to the world
        t.transform.translation.x = 0.5
        t.transform.translation.y = 0.0
        t.transform.translation.z = 1.0

        # Set the rotation (identity quaternion for no rotation)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

    def run(self):
        """
        Run the node, continuously broadcasting the transform and handling camera feed.
        """
        rospy.loginfo("Camera Transform Publisher Node started")
        
        while not rospy.is_shutdown():
            self.publish_transform()  # Continuously publish the camera transform
            self.rate.sleep()


if __name__ == '__main__':
    try:
        # Create and run the CameraTransformPublisher node
        camera_transform_publisher = CameraTransformPublisher()
        camera_transform_publisher.run()

    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted. Shutting down.")
    finally:
        cv2.destroyAllWindows()  # Close the OpenCV window when exiting
